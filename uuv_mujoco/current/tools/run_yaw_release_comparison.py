#!/usr/bin/env python3
"""Compare identical recorded final PWM in actual MuJoCo plants, without arming.

No controller runs here. RC and measured angular rate are annotations; recorded
PWM is held constant between packets. This cannot establish closed-loop parity.
"""
from pathlib import Path
from types import SimpleNamespace
import argparse
import contextlib
import csv
import io
import hashlib
import json
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(Path(__file__).resolve().parent))
from check_research_pool_physics import _build_runtime
from yaw_release_trace import extract_tlog, validate_trace
from physics.thruster_mapping import ARDUSUB_VECTORED_6DOF_SERVO_MAP as MAP, ARDUSUB_VECTORED_6DOF_SERVO_SIGNS as SIGNS
from sim.runtime.sitl_servo_pwm import packet_commands_from_pwm
from sim.physics.thruster_performance_loader import load_thruster_performance_config


def run(trace, profile, dt, voltage, force_model="t200"):
    """Replay final PWM with the measured T200 model or explicit fallback."""
    import mujoco
    with contextlib.redirect_stdout(io.StringIO()):
        runtime=_build_runtime(mujoco,profile_name=profile,fluid_model='legacy',use_custom_hydrodynamics=True)
        perf = (load_thruster_performance_config(ROOT/'config/thruster_performance.json',requested_voltage=voltage,direct=True)
                if force_model == 't200' else runtime.thruster_actuator.perf_cfg)
    if force_model == 't200' and not perf['active']: raise ValueError('T200 curve unavailable')
    model,data=runtime.model,runtime.data
    actuator=runtime.thruster_actuator; actuator.perf_cfg=perf
    model.opt.timestep=dt
    # Explicit identical environmental and initial-state assumptions for both plants.
    environment=SimpleNamespace(velocity_world=lambda p,t:np.zeros(3),surface_height_world_m=lambda p,t:0.)
    runtime.hydrodynamics.water_environment_runtime=environment
    actuator.set_current_velocity_sampler(environment.velocity_world)
    actuator.set_surface_height_sampler(environment.surface_height_world_m)
    q=int(runtime.state.world_qpos_adr);v=int(runtime.state.world_qvel_adr);bid=int(runtime.state.base_id)
    data.qpos[q+2]=-1.5;data.qpos[q+3:q+7]=[1,0,0,0];data.qvel[:]=0
    data.qvel[v+5]=trace['initial_yaw_rate_flu_radps']
    mujoco.mj_forward(model,data)
    input_rows=trace['rows'];times=np.array([r['t_s'] for r in input_rows]); rows=[]
    previous_thruster_time=-.01
    for step in range(int(np.floor(times[-1]/dt))+1):
        t=float(data.time);src=input_rows[max(0,np.searchsorted(times,t+1e-10,side='right')-1)]
        actuator.target.update(packet_commands_from_pwm(all_thruster_names=actuator.all_thruster_names,
            raw_map=MAP,servo_signs=SIGNS,pwm_values=src['servo_pwm']))
        # Match the 100 Hz JSON actuator path used by the GUI.
        if t-previous_thruster_time >= .01-1e-10:
            actuator.update_forces(t-previous_thruster_time,base_id=bid);previous_thruster_time=t
        runtime.underwater.apply(dt)
        velocity=np.zeros(6);mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_BODY,bid,velocity,1)
        hyd=runtime.underwater.last_distributed_hydrodynamics_result
        row=dict(t_s=t,rc4_pwm=src['rc4_pwm'],input_sample_t_s=src['t_s'],
            observed_yaw_rate_flu_radps=src['observed_yaw_rate_flu_radps'],
            yaw_rate_flu_radps=float(velocity[2]),
            thrust_yaw_nm=float((actuator.last_torque_body+actuator.last_reaction_torque_body)[2]),
            residual_yaw_nm=float(hyd.residual_damping_wrench_body[5]),
            patch_yaw_nm=float(np.cross(hyd.positions_world_m-hyd.wrench_reference_position_world_m,
                hyd.form_drag_forces_world_n+hyd.skin_drag_forces_world_n).sum(axis=0)[2]),
            added_mass_yaw_nm=float(runtime.underwater.last_full_matrix_wrench_body[5]))
        for i,name in enumerate(MAP):
            row[f'servo{i+1}_pwm']=src['servo_pwm'][i]
            row[f'{name}_force_n']=actuator.force_cmd[name]
            row[f'{name}_command']=actuator.state[name]
        rows.append(row)
        mujoco.mj_step(model,data)
        if not np.all(np.isfinite(data.qvel)): raise FloatingPointError('nonfinite plant state')
    release=trace['release_s'];after=[r for r in rows if r['t_s']>=release]
    before=[r for r in rows if r['t_s']<release]
    direction=float(np.sign(before[-1]['yaw_rate_flu_radps']))
    def first(predicate): return next((r['t_s']-release for r in after if predicate(r)),None)
    metrics=dict(profile=profile,dt_s=dt,force_model=force_model,
        selected_voltage_v=float(perf['selected_voltage']) if perf.get('active') else None,
        peak_yaw_rate_radps=max(abs(r['yaw_rate_flu_radps']) for r in rows),
        release_yaw_rate_radps=before[-1]['yaw_rate_flu_radps'],
        reverse_thrust_after_release_s=first(lambda r:direction*r['thrust_yaw_nm']<-.01),
        reverse_rate_after_release_s=first(lambda r:direction*r['yaw_rate_flu_radps']<-.001),
        reverse_excursion_rad=float(np.trapezoid([max(0.,-direction*r['yaw_rate_flu_radps']) for r in after],
                                               [r['t_s'] for r in after])),
        final_yaw_rate_radps=rows[-1]['yaw_rate_flu_radps'])
    return rows,metrics


def main():
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True);source.add_argument('--tlog',type=Path);source.add_argument('--trace',type=Path)
    p.add_argument('--release-index',type=int,default=-1);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--force-model',choices=('configured','t200'),default='t200')
    p.add_argument('--dt',type=float,default=.005);p.add_argument('--voltage',type=float,default=16.)
    p.add_argument('--repeats',type=int,default=2);args=p.parse_args()
    if not 0<args.dt<=.01 or args.repeats<1: p.error('dt must be (0,.01], repeats >=1')
    trace=extract_tlog(args.tlog,release_index=args.release_index) if args.tlog else validate_trace(json.loads(args.trace.read_text()))
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/'input_trace.json').write_text(json.dumps(trace,indent=2))
    summary=[]
    for profile in ('research_pool_distributed','research_pool_distributed_hybrid'):
        reference=None
        for repeat in range(args.repeats):
            rows,metrics=run(trace,profile,args.dt,args.voltage,args.force_model)
            if reference is not None:
                np.testing.assert_array_equal([r['yaw_rate_flu_radps'] for r in rows],reference)
            reference=[r['yaw_rate_flu_radps'] for r in rows]
            with (args.out/f'{profile}_{repeat+1}.csv').open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        summary.append(metrics)
    observed_before=[r for r in trace['rows'] if r['t_s']<trace['release_s']][-1]
    observed_direction=float(np.sign(observed_before['observed_yaw_rate_flu_radps']))
    observed_crossing=next((r['t_s']-trace['release_s'] for r in trace['rows']
        if r['t_s']>=trace['release_s'] and observed_direction*r['observed_yaw_rate_flu_radps']<-.02),None)
    report=dict(observed_reverse_after_release_s=observed_crossing,experiment='offline recorded PWM plant replay; not live Stabilize',
        repeats=args.repeats,repeatability='identical yaw-rate samples' if args.repeats>1 else 'not repeated',source_sha256=trace.get('sha256'),
        input_source=trace.get('source'),
        configuration_sha256={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (ROOT/'config/sim_profiles.json',ROOT/'config/thruster_params.json',
                         ROOT/'config/thruster_performance.json',ROOT/'scenes/research_pool_slam_scene.xml')},
        assumptions=trace.get('assumptions',[]),results=summary)
    (args.out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':main()
