"""Isolated parallel VLA rollouts and synchronized residual PPO updates."""
import os,sys,json,time,signal,subprocess,traceback,fcntl,copy
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import torch
from isolation import IsolatedSimulator
from residual_policy import ResidualActorCritic,advantages,update,save_checkpoint
from ipc import read,write,status
from transition import attach_feedback
from evaluation import summary, assessment
from recovery import physics_failure, FailureIsolation, failure_kind, find_resume
from schedule import round_plan

ROOT=Path(__file__).resolve().parents[2]
RUN=Path(sys.argv[1]).resolve();config=json.loads((RUN/'config.json').read_text())
groups=config.get('policy_groups', {})
if groups:
 config={**config, 'models':list(groups),
         'model_paths':{k:config['model_paths'][g['base_model']] for k,g in groups.items()}}
 status(RUN/'resolved-config.json',config)
counts={k:groups[k]['environments'] if groups else config['environments_per_model'] for k in config['models']}
eval_repeats=config.get('evaluation_repeats',1)
cycle_length=1+2*eval_repeats
run_lock=(ROOT/'outputs/rl-training.lock').open('a')
try:fcntl.flock(run_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:
 status(RUN/'status.json',{'phase':'error','error':'다른 강화학습 실행이 진행 중입니다.','updates':0,'episodes':[],'workers':[]})
 raise SystemExit(1)
os.environ.update(HF_HUB_OFFLINE='1',TRANSFORMERS_OFFLINE='1',TOKENIZERS_PARALLELISM='false',NO_ALBUMENTATIONS_UPDATE='1')
sys.path[:0]=[str(ROOT/'outputs/vla-transfer-audit-20260910/upstream/auv_vla'),str(ROOT/'rospkg/src/auv_vla_data_collector')]
from gr00t.model.policy import Gr00tPolicy
from kmu26_auv_vla_data_collector.transfer_config import Kmu26TransferDataConfig
from kmu26_auv_vla_data_collector.deployment_config import validate_deployment_contract
stop=False
def cancel(*_):
 global stop
 stop=True
signal.signal(signal.SIGTERM,cancel);signal.signal(signal.SIGINT,cancel)
state={'episode_seconds':config['episode_seconds'],'environment_count':sum(counts.values()),'policy_environment_counts':counts,'reward_version':config.get('reward_version','legacy'),'phase':'loading','updates':0,'update_attempts':0,'promotions':0,'episodes':[],'workers':[],'error':None}
def report(**kw):
 state.update(kw);status(RUN/'status.json',state)
def check_stop():
 if stop:raise InterruptedError('사용자가 중지했습니다.')
models={};actors={};optimizers={};workers=[];children=[];logs=[]
cleanup_pool=ThreadPoolExecutor(max_workers=3)
isolation=FailureIsolation()

def launch_rollout(worker,key,wave,suffix=''):
 folder=worker.directory/'outputs'/f'rollout-{wave:05d}{suffix}';folder.mkdir()
 (folder/'config.json').write_text(json.dumps({**{k:v for k,v in config.items() if k!='model_paths'},'_final_wave':wave==cycle_length*config['episodes_per_environment']}))
 inside=f'/workspace/outputs/{folder.name}'
 script='source /opt/ros/humble/setup.bash && source /workspace/rospkg/install/setup.bash && export PYTHONPATH=/workspace/rospkg/src/auv_vla_data_collector:$PYTHONPATH && python3 /workspace/tools/rl_training/worker.py '+inside
 log=(folder/'worker.log').open('w');logs.append(log)
 process=subprocess.Popen(['docker','exec',worker.name,'bash','-lc',script],stdout=log,stderr=subprocess.STDOUT);children.append(process)
 return folder,process,key

keys=('prev_command','dvl_velocity','angular_velocity','linear_acceleration','attitude','depth','altitude','validity')
try:
 torch.set_num_threads(4);torch.manual_seed(42)
 for key in config['models']:
  check_stop();report(phase='loading',message=key+' 모델 로딩')
  path=ROOT/config['model_paths'][key];validate_deployment_contract(path)
  same_path=next((k for k in models if config['model_paths'][k]==config['model_paths'][key]),None)
  dc=Kmu26TransferDataConfig()
  models[key]=(models[same_path] if same_path else Gr00tPolicy(model_path=str(path),modality_config=dc.modality_config(),modality_transform=dc.transform(),embodiment_tag='new_embodiment',device='cuda',denoising_steps=4))
  models[key].model.eval().requires_grad_(False)
  # Share only a bitwise-identical frozen backbone. Different action heads and
  # per-checkpoint observation normalization remain independent.
  prior_keys=[k for k in models if k!=key]
  if prior_keys and not same_path:
   reference=models[prior_keys[0]].model.backbone
   candidate=models[key].model.backbone
   left=reference.state_dict();right=candidate.state_dict()
   same=(models[key].model.config.backbone_cfg==models[prior_keys[0]].model.config.backbone_cfg
         and left.keys()==right.keys() and all(torch.equal(left[k],right[k]) for k in left))
   del left,right
   if same:
    models[key].model.backbone=reference
    del candidate
    torch.cuda.empty_cache()
    state.setdefault('shared_backbones',[]).append(key)
    report(message=key+' 동일한 고정 백본 메모리 공유')
  actors[key]=ResidualActorCritic(learned_brake=bool(groups));optimizers[key]=torch.optim.Adam(actors[key].parameters(),lr=config.get('ppo_learning_rate',3e-4))
  if groups:
   candidate=Path(groups[key]['checkpoint']).resolve(strict=True)
   checkpoint=torch.load(candidate,map_location='cpu',weights_only=False)
   if checkpoint.get('schema')!='uuv.residual_rl.v1' or (ROOT/checkpoint['base_model']).resolve()!=path.resolve():
    raise ValueError('정밀 학습 시작 정책과 원본 VLA가 호환되지 않습니다.')
   actors[key].seed_legacy(checkpoint['policy'])
   actors[key].residual_limit=checkpoint['residual_limit']
   actors[key].source_checkpoint=str(candidate)
   actors[key].source_lineage_updates=checkpoint.get('lineage_updates',checkpoint['updates'])
   state.setdefault('initial_policies',{})[key]={'path':str(candidate),'source_updates':checkpoint['updates'],
      'source_lineage_updates':actors[key].source_lineage_updates,'optimizer':'reset for precision_v3','critic':'reset for precision_v3','brake':'zero mean; legacy deterministic action preserved'}
  elif config.get('resume_candidate'):
   if key != 'success-135-110-25000':raise ValueError('12회 후보는 135개 기반 모델에만 적용됩니다.')
   candidate,checkpoint,exact_resume=find_resume(ROOT,key,config)
   if checkpoint['schema']!='uuv.residual_rl.v1' or (ROOT/checkpoint['base_model']).resolve()!=path.resolve():raise ValueError('보정 정책과 원본 모델이 다릅니다.')
   actors[key].load_state_dict(checkpoint['policy'])
   actors[key].residual_limit=checkpoint['residual_limit']
   if exact_resume:
    optimizers[key].load_state_dict(checkpoint['optimizer'])
   else:
    torch.nn.init.zeros_(actors[key].critic.weight);torch.nn.init.zeros_(actors[key].critic.bias)
   state['initial_policy']={'path':str(candidate),'updates':checkpoint['updates'],'optimizer':'restored' if exact_resume else 'fresh for changed reward','critic':'restored' if exact_resume else 'reset for changed reward'}
   actors[key].inherited_updates=checkpoint.get('lineage_updates',checkpoint['updates']) if exact_resume else 0
   state['inherited_updates']=actors[key].inherited_updates
   report(message='최근 호환 학습 가중치·학습 상태 복원 완료' if exact_resume else '보존한 12회 후보 로드 완료')

 for key in config['models']:
  for local in range(counts[key]):
   check_stop();index=len(workers);worker=IsolatedSimulator(ROOT,RUN,index)
   report(phase='starting',message=f'환경 {index+1} 시작')
   worker.start();workers.append((worker,key))
   for _ in range(60):
    check_stop()
    try:worker.api();break
    except subprocess.CalledProcessError:time.sleep(.5)
   else:raise RuntimeError('Worker GUI unavailable')
   worker.api('stack_start',sim_preset='research_pool_yaw_stable')
 incumbents={};baseline_scores={};candidates={};changed_models=set();training_updates=0
 model_updates={k:0 for k in config['models']};evaluation_episodes={}
 for wave in range(1+cycle_length*config['episodes_per_environment']):
  round_kind,repeat_index=round_plan(wave,eval_repeats)
  evaluation = round_kind != 'training'
  if wave and evaluation and not changed_models:
   continue  # An identical policy is not a candidate and needs no evaluation.
  active_keys = changed_models if wave and evaluation else set(config['models'])
  if round_kind == 'incumbent_evaluation':
   for key in active_keys:
    actors[key].load_state_dict(incumbents[key][0]);optimizers[key].load_state_dict(incumbents[key][1])
  elif round_kind == 'candidate_evaluation':
   for key in active_keys:
    actors[key].load_state_dict(candidates[key][0]);optimizers[key].load_state_dict(candidates[key][1])
  state['round_kind']=round_kind
  state['evaluation_repeat']=repeat_index+1 if evaluation else None
  if evaluation and repeat_index==0:evaluation_episodes={k:[] for k in active_keys}
  check_stop();report(phase='collecting',message=f'{wave+1}번째 병렬 수집',wave=wave+1)
  for key in active_keys:
   save_checkpoint(RUN/key/f'collection-{wave+1:05d}.pt',actors[key],optimizers[key],base_model=config['model_paths'][key],updates=model_updates[key],config=config)
  active={};trajectories={i:[] for i in range(len(workers))}
  for index,(worker,key) in enumerate(workers):
   if key not in active_keys or index in isolation.retired:continue
   active[index]=launch_rollout(worker,key,wave)
  deadline=time.monotonic()+max(600,config['episode_seconds']*40)
  last_report=0.
  while active:
   check_stop()
   if time.monotonic()>deadline:raise RuntimeError('Rollout watchdog timeout')
   for index,(folder,process,key) in list(active.items()):
    request=folder/'request.pkl'
    error_path=folder/'error.json'
    exited=process.poll() is not None and not request.exists()
    if error_path.exists() or exited:
     error=json.loads(error_path.read_text()) if error_path.exists() else {'error':'Worker process exited'}
     worker,_=workers[index]
     fault=physics_failure(worker.directory/'uuv_mujoco/current/logs')
     kind=fault['kind'] if fault else (failure_kind(error.get('original_error',error['error'])) if error_path.exists() else 'worker_exit')
     detail=fault['detail'] if fault else error['error']
     if kind not in isolation.recoverable:raise RuntimeError(f'환경 {index+1}: '+detail)
     state.setdefault('infrastructure_failures',[]).append(dict(worker=index,wave=wave+1,recording=str(folder),excluded_from_training=True,discarded_transitions=len(trajectories[index]),kind=kind,detail=detail))
     status(RUN/'infrastructure-failures.json',state['infrastructure_failures'])
     trajectories[index]=[]  # Drop the entire incomplete trajectory, including pending actions.
     del active[index]
     cleanup_pool.submit(worker.close)
     try:
      isolation.discard(index,kind)
     finally:
      state['excluded_workers']=sorted(isolation.retired)
      state['active_environment_count']=len(workers)-len(isolation.retired)
     report(message=f'환경 {index+1} 제외 · 나머지 환경에서 계속 학습')
     if not any(k==key and i not in isolation.retired for i,(_,k) in enumerate(workers)):
      raise RuntimeError(key+': 남은 환경이 없어 중단')
     continue
    if not request.exists():continue
    data=read(request);request.unlink()
    trajectory=trajectories[index]
    attach_feedback(trajectory,data)
    if data.get('done'):
     state['episodes'].append(dict(worker=index,model=key,wave=wave+1,round_kind=round_kind,reward=data['total_reward'],success=data['success'],released=data.get('released',data['success']),release_tier=data.get('release_tier','unknown'),precision_best=data.get('precision_best'),policy=str(RUN/key/f'collection-{wave+1:05d}.pt'),recording=str(folder/'episode.json')))
     process.wait(timeout=15)
     if process.returncode:raise RuntimeError('Rollout cleanup failed')
     del active[index];continue
    obs=data['observation']
    with torch.inference_mode():
     if evaluation:torch.manual_seed(91000+repeat_index*1000000+index*10000+data['id'])
     action,_=models[key].get_action(obs)
    base=np.asarray(action['action.motion']).reshape(-1,4)[0].clip(-1,1)
    if not np.isfinite(base).all():raise ValueError('Nonfinite VLA action')
    if data.get('warmup'):
     write(folder/'response.pkl',dict(id=data['id'],action=np.zeros(4)));continue
    sensor=np.concatenate([np.asarray(obs['state.'+k]).reshape(-1) for k in keys]).astype(np.float32)
    sensor_tensor=torch.from_numpy(sensor).unsqueeze(0);base_tensor=torch.tensor(base,dtype=torch.float32).unsqueeze(0)
    action,raw,logp,value=actors[key].act(sensor_tensor,base_tensor,deterministic=evaluation)
    trajectory.append(dict(state=sensor_tensor[0],base_action=base_tensor[0],raw_action=raw[0],old_log_prob=logp[0],value=value[0]))
    write(folder/'response.pkl',dict(id=data['id'],action=action[0].numpy()))
   if time.monotonic()-last_report>=.5:
    report(workers=[dict(worker=i,model=k,**(json.loads((f/'progress.json').read_text()) if (f/'progress.json').exists() else {})) for i,(f,p,k) in active.items()])
    last_report=time.monotonic()
   time.sleep(.005)
  state['completed_rounds']=wave+1
  state['active_environment_count']=len(workers)-len(isolation.retired)
  if evaluation:
   for key in active_keys:
    evaluation_episodes[key].extend(e for e in state['episodes'] if e['wave']==wave+1 and e['model']==key)
   if round_kind!='baseline_evaluation' and repeat_index<eval_repeats-1:
    report(message='반복 평가 수집 중 · 정책 가중치 고정')
    continue
   decisions={}
   for key in active_keys:
    score=summary(evaluation_episodes[key])
    if round_kind == 'incumbent_evaluation':
     baseline_scores[key]=score
     decisions[key]={'role':'fresh_baseline','score':score}
     continue
    initial = round_kind == 'baseline_evaluation'
    outcome='initial_baseline' if initial else assessment(baseline_scores[key],score,parameter_changed=key in changed_models,require_outcome_gain=bool(groups))
    accepted=initial or outcome=='promote'
    decisions[key]={'accepted':accepted,'candidate':score,'baseline':None if initial else baseline_scores[key],
                    'role':'initial_baseline' if initial else 'candidate','parameter_changed':not initial,'outcome':outcome}
    if accepted:
     incumbents[key]=(copy.deepcopy(actors[key].state_dict()),copy.deepcopy(optimizers[key].state_dict()))
     save_checkpoint(RUN/key/'accepted.pt',actors[key],optimizers[key],base_model=config['model_paths'][key],updates=model_updates[key],config=config)
     if not initial:state['promotions']+=1
    elif outcome=='rollback':
     actors[key].load_state_dict(incumbents[key][0]);optimizers[key].load_state_dict(incumbents[key][1])
    save_checkpoint(RUN/key/'training-latest.pt',actors[key],optimizers[key],base_model=config['model_paths'][key],updates=model_updates[key],config=config)
   state.setdefault('evaluations',[]).append({'round':wave+1,'kind':round_kind,'decisions':decisions})
   status(RUN/f'evaluation-{wave+1:05d}.json',decisions)
   report(message='현 기준 정책 재평가 완료' if round_kind=='incumbent_evaluation' else '평가 완료 · 최고 모델 선정과 학습 진행을 구분합니다.',evaluation=decisions)
   continue
  report(phase='updating',message='수집 완료 · PPO 업데이트')
  changed_models=set();candidates={}
  state['update_attempts']+=1
  for key in config['models']:
   batches=[]
   for index,(_,worker_key) in enumerate(workers):
    if worker_key!=key or index in isolation.retired:continue
    trajectory=trajectories[index]
    if not trajectory or any('feedback' not in x for x in trajectory):raise RuntimeError('Incomplete transition rewards')
    rewards=torch.tensor([x['feedback']['reward'] for x in trajectory])[:,None]
    values=torch.stack([x['value'] for x in trajectory])[:,None]
    durations=torch.tensor([x['feedback']['duration'] for x in trajectory])[:,None]
    terminal=torch.zeros_like(rewards,dtype=torch.bool);terminal[-1]=True
    next_values=torch.cat([values[1:],torch.zeros_like(values[:1])])
    adv,ret=advantages(rewards,values,next_values,terminal,terminal,durations,gamma=config.get('ppo_gamma',.99),lam=config.get('ppo_lambda',.95))
    batch={name:torch.stack([x[name] for x in trajectory]) for name in ('state','base_action','raw_action','old_log_prob')}
    batch.update(advantage=adv[:,0],**{'return':ret[:,0]});batches.append(batch)
   batch={name:torch.cat([b[name] for b in batches]) for name in batches[0]}
   torch.save(batch,RUN/key/f'rollout-batch-{wave+1:05d}.pt')
   before=copy.deepcopy(actors[key].state_dict())
   metrics=update(actors[key],optimizers[key],batch,clip=config.get('ppo_clip',.2),kl_limit=config.get('ppo_kl_limit',.01),drift_limit=config.get('ppo_drift_limit',.005),entropy_coefficient=config.get('ppo_entropy',0.0))
   changed=any(not torch.equal(value,before[name]) for name,value in actors[key].state_dict().items() if not name.startswith('critic.'))
   if changed:
    model_updates[key]+=1
    changed_models.add(key)
    candidates[key]=(copy.deepcopy(actors[key].state_dict()),copy.deepcopy(optimizers[key].state_dict()))
   state.setdefault('training_metrics',{})[key]={**metrics,'parameter_changed':changed}
   save_checkpoint(RUN/key/f'update-{wave+1:05d}.pt',actors[key],optimizers[key],base_model=config['model_paths'][key],updates=model_updates[key],config=config)
   status(RUN/key/'latest.json',dict(update=wave+1,parameter_changed=changed,transitions=len(batch['state']),metrics=metrics))
  training_updates+=int(bool(changed_models))
  report(updates=training_updates,model_updates=model_updates,message='가중치 변경 완료 · 기존 정책과 후보를 새로 평가합니다.' if changed_models else '가중치 변경 없음 · 평가 생략')
 report(phase='completed',message='학습 완료')
except InterruptedError as e:report(phase='stopped',message=str(e))
except Exception as e:
 report(phase='error',error=str(e),message='학습 오류');traceback.print_exc()
finally:
 stop=True
 cleanup_pool.shutdown(wait=True)
 for worker,key in workers:
  try:worker.close()
  except Exception:traceback.print_exc()
 for p in children:
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:p.terminate()
 for log in logs:log.close()
