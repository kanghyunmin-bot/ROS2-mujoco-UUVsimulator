# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""Plot existing response records and independently check reported errors."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
audit = {'source_sha256': {}, 'checks': [], 'new_simulation_runs': 0,
         'method': 'Receipt-time alignment; no fitted lag, scale, smoothing or time warp. Valid DVL samples only. Surge baseline is nominal; release baseline is clock-corrected.'}
def source(path):
    path = ROOT / path
    audit['source_sha256'][str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return path

def read(path):
    return json.loads(source(path).read_text())

def support(t, grid, gap):
    ix = np.searchsorted(t, grid, side='right')
    valid = (ix > 0) & (ix < len(t))
    ix = np.clip(ix, 1, len(t)-1)
    return valid & (t[ix]-t[ix-1] <= gap), ix

nom = read('docs/assets/real2sim-application-20260915/comparison.json')
final = read('docs/assets/yaw-release-20260915/full_comparison.json')
release = read('docs/assets/yaw-release-20260915/comparison.json')
plt.rcParams.update({'font.family':'Liberation Serif', 'font.size':8, 'axes.titlesize':8.5,
 'axes.labelsize':8, 'xtick.labelsize':7.5, 'ytick.labelsize':7.5, 'pdf.fonttype':42,
 'axes.spines.top':False, 'axes.spines.right':False})
fig, axes = plt.subplots(1,3,figsize=(7,1.95),gridspec_kw={'width_ratios':[1.2,1.2,.9]})
fig.subplots_adjust(left=.065,right=.99,bottom=.23,top=.76,wspace=.42)
colors=['#263445','#d66b38','#008473']
release_values=[]
for j,bag in enumerate(['primary','auxiliary']):
    cfg=read(f'outputs/yaw-release-real2sim-20260915/{bag}_commands.json')
    profile=read(cfg['source']+'/profile.json')
    data=np.load(source(cfg['source']+'/numeric.npz'))
    t0=min(x['start'] for x in profile.values())
    dvl=data['__dvl__data']; td=dvl[:,0]-t0
    start,end=final['bags'][bag]['evaluation_s']; grid=np.arange(start,end,.1)
    valid,ix=support(td,grid,.3)
    valid &= (dvl[ix,5]>.5)&(dvl[ix-1,5]>.5)
    real=np.interp(grid,td,dvl[:,1])
    axes[j].plot(grid,np.where(valid,real,np.nan),color=colors[0],lw=1.05,label='Real DVL')
    rms=[]
    for k,(path,expected,label) in enumerate([
        (f'outputs/real2sim-application-20260915/{bag}_nominal',nom['bags'][bag]['cases']['nominal']['surge_rmse_mps'],'Nominal'),
        (f'outputs/yaw-release-real2sim-20260915/{bag}_validated',final['bags'][bag]['cases']['yaw_response']['surge_rmse_mps'],'Latest calibrated')]):
        run=read(path+'/replay.json'); assert run['complete']
        f=np.genfromtxt(source(path+'/forces.csv'),delimiter=',',names=True)
        tf=f['sim_time']-run['origin_sim_time']+cfg['begin']
        u=np.interp(grid,tf,f['lin_vel_body_x']); mask=valid&support(tf,grid,.2)[0]
        measured=float(np.sqrt(np.mean((u[mask]-real[mask])**2)))
        assert abs(measured-expected)<1e-10,(bag,measured,expected)
        rms.append(measured)
        audit['checks'].append({'bag':bag,'metric':'surge_rmse_mps','case':label,'value':measured,'matches_recorded_summary':True})
        axes[j].plot(grid,np.where(mask,u,np.nan),color=colors[k+1],lw=1,ls='--' if k==0 else '-',label=label)
    axes[j].set(title=f'({chr(97+j)}) Run {"A" if j==0 else "B"}: surge',xlabel='Bag time (s)',ylabel='Velocity (m/s)',xlim=(start,end),ylim=(-1,1.5))
    axes[j].set_xticks([start,(start+end)/2,end]); axes[j].grid(alpha=.2,lw=.5)
    axes[j].text(.03,.97,f'RMSE {rms[0]:.3f} → {rms[1]:.4f} m/s',transform=axes[j].transAxes,va='top',fontsize=7.3)
    imu=data['__mavros__imu__data']; ti=imu[:,0]-t0; real_rate=np.rad2deg(imu[:,7])
    vals=[]
    for case,path in [('baseline',f'outputs/yaw-timing-audit-20260915/{bag}_round'),('validated',f'outputs/yaw-release-real2sim-20260915/{bag}_validated')]:
        run=read(path+'/replay.json'); f=np.genfromtxt(source(path+'/forces.csv'),delimiter=',',names=True)
        tf=f['sim_time']-run['origin_sim_time']+cfg['begin']; errors=[]
        for event in release['bags'][bag]['events']:
            t=event['release_s']; mask=(ti>=t)&(ti<t+2.5)
            assert support(tf,ti[mask],.2)[0].all()
            errors.extend(np.interp(ti[mask],tf,np.rad2deg(f['ang_vel_body_z']))-real_rate[mask])
        value=float(np.sqrt(np.mean(np.square(errors))))
        assert abs(value-release['bags'][bag]['cases'][case]['release_rmse_deg_s'])<1e-10
        vals.append(value); audit['checks'].append({'bag':bag,'metric':'release_rmse_deg_s','case':case,'value':value,'matches_recorded_summary':True})
    release_values.append(vals)
ax=axes[2]; v=np.array(release_values)
for k,label in enumerate(['Clock corrected','Response calibrated']):
    bars=ax.bar(np.arange(2)+(k-.5)*.32,v[:,k],width=.30,color=colors[k+1],label=label)
    ax.bar_label(bars,fmt='%.2f',padding=2,fontsize=7)
ax.set(title='(c) Yaw release error',ylabel='Rate RMSE (deg/s)',xticks=[0,1],xticklabels=['Run A','Run B'],ylim=(0,9))
ax.grid(axis='y',alpha=.2); ax.set_axisbelow(True)
fig.legend(*axes[0].get_legend_handles_labels(),loc='upper left',bbox_to_anchor=(.05,1.025),ncol=3,frameon=False,fontsize=7.5,handlelength=1.6,columnspacing=1)
ax.legend(loc='upper center',bbox_to_anchor=(.5,1.53),frameon=False,fontsize=7,handlelength=1)
for ext in ['png','pdf','svg']: fig.savefig(HERE/'figures'/f'fig3_response_comparison.{ext}',dpi=400)
(HERE/'response_verification.json').write_text(json.dumps(audit,indent=2)+'\n')
print('Verified 8 response RMSE values from raw records.')
