import json,time,subprocess
from pathlib import Path
from isolation import IsolatedSimulator
root=Path(__file__).resolve().parents[2];out=root/'outputs/rl-isolation-check-v2'
w=IsolatedSimulator(root,out,0)
try:
 w.start()
 for _ in range(30):
  try:w.api();break
  except subprocess.CalledProcessError:time.sleep(1)
 result=w.api('stack_start',sim_preset='research_pool_yaw_stable')
 for _ in range(120):
  s=w.api()
  if (s['processes'].get('sim_running') and s['processes'].get('mavros_running')
      and s['telemetry'].get('connected') and s['telemetry'].get('imu_age_s') is not None):
   print('PASS isolated simulation with connected MAVROS and IMU',flush=True);break
  time.sleep(1)
 else:raise RuntimeError('Isolated simulation startup did not complete')
 (out/'status.json').write_text(json.dumps(s,indent=2))
finally:
 log=subprocess.run(['docker','logs',w.name],capture_output=True,text=True)
 (out/'container.log').write_text(log.stdout+log.stderr)
 w.close()
