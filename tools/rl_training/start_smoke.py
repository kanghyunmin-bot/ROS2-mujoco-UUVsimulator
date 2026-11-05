import sys,json,subprocess,time
from pathlib import Path
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'tools/vla_gui'))
from rl_config import defaults
r=root/'outputs'/('rl-smoke-'+time.strftime('%Y%m%d-%H%M%S'));r.mkdir()
c=defaults();c.update(models=['success-59-10000'],environments_per_model=1,episodes_per_environment=1,episode_seconds=2.,model_paths={'success-59-10000':'outputs/training-59-sequential-20260921/current59/model'})
if '--repeat' in sys.argv:c['episodes_per_environment']=2
if '--six' in sys.argv:
 c['models'].append('success-mixed-84-15000');c['environments_per_model']=3
 c['model_paths']['success-mixed-84-15000']='outputs/training-59-sequential-20260921/mixed84/model'
if '--precision_v4' in sys.argv:
 c=json.loads((root/'outputs/vla-gui/rl_config.json').read_text())
 if c['reward_version']!='precision_v4':raise ValueError('Save an explicit precision_v4 configuration first')
 name,group=next(iter(c['policy_groups'].items()))
 c.update(policy_groups={name:{**group,'environments':1}},environments_per_model=1,
          episodes_per_environment=1,evaluation_repeats=1,episode_seconds=2.,
          model_paths={'success-135-110-25000':'outputs/training-135-emphasis110-20260921/all135/model'})
(r/'config.json').write_text(json.dumps(c))
with (r/'run.log').open('w') as f:
 p=subprocess.Popen(['/home/khm/miniforge3/envs/gr00t/bin/python','-u',str(root/'tools/rl_training/run.py'),str(r)],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
(r/'pid').write_text(str(p.pid));print(r,p.pid)
