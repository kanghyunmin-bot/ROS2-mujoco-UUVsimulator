"""Dedicated network/PID/IPC namespaces for simulator training workers."""
from pathlib import Path
import json
import shutil
import subprocess


class IsolatedSimulator:
    def __init__(self, repository, run_directory, index):
        self.repository=Path(repository).resolve()
        self.directory=Path(run_directory).resolve()/f'env-{index}'
        self.directory.mkdir(parents=True,exist_ok=True)
        self.name=f'uuv-rl-{self.directory.parent.name}-{index}'
        self.owned=False

    def start(self):
        mounts=[]
        # SITL writes EEPROM and metadata into its working directory. Give it a
        # private root while sharing read-only source/build subdirectories.
        for relative in ('ardupilot_sub_stable', 'uuv_mujoco/current'):
            target=self.directory/relative
            target.mkdir(parents=True,exist_ok=True)
            for source in (self.repository/relative).iterdir():
                destination=target/source.name
                if destination.exists() or destination.is_symlink():
                    continue
                if source.name in ('generated','logs','__pycache__'):
                    destination.mkdir()
                elif source.is_dir() and (relative=='ardupilot_sub_stable' or source.name=='assets'):
                    destination.symlink_to('/source/'+relative+'/'+source.name,target_is_directory=True)
                elif source.is_dir():
                    shutil.copytree(source,destination,ignore=shutil.ignore_patterns('__pycache__'))
                else:
                    shutil.copy2(source,destination)
            mounts.extend(['-v',f'{target}:/workspace/{relative}'])
        # Repository is read-only; generated artifacts and logs are private.
        for relative in ('outputs','uuv_mujoco/current/generated','uuv_mujoco/current/logs'):
            target=self.directory/relative;target.mkdir(parents=True,exist_ok=True)
            mounts.extend(['-v',f'{target}:/workspace/{relative}'])
        command=['docker','run','-d','--name',self.name,'--network','none','--gpus','all',
            '--init','--shm-size','512m','--user','root','--entrypoint','bash',
            '-e','UUV_RL_REWARD_TELEMETRY=1','-e','OPENBLAS_NUM_THREADS=1','-e','OMP_NUM_THREADS=1','-e','UUV_GUI_MUJOCO_VIEWER=0','-e','MUJOCO_GL=egl','-e','ROS_DOMAIN_ID=42',
            '-v',f'{self.repository}:/workspace:ro','-v',f'{self.repository}:/source:ro',*mounts,
            'khm/ros2-mujoco-uuv:dev','-lc',
            'source /opt/ros/humble/setup.bash && source /workspace/rospkg/install/setup.bash && '
            'cd /workspace && python3 uuv_mujoco/current/gui/web_control_gui.py --host 127.0.0.1 --port 8878']
        subprocess.run(command,check=True,capture_output=True,text=True)
        self.owned=True
        info=json.loads(subprocess.check_output(['docker','inspect',self.name]))[0]
        if info['HostConfig']['NetworkMode']!='none':
            self.close();raise RuntimeError('Worker network isolation failed')

    def close(self):
        if self.owned:
            subprocess.run(['docker','rm','-f',self.name],check=True,capture_output=True)
            self.owned=False

    def api(self, command=None, **kwargs):
        payload=None if command is None else {'command':command,**kwargs}
        script='''import json,sys,urllib.request
payload=json.loads(sys.argv[1])
data=None if payload is None else json.dumps(payload).encode()
url='http://127.0.0.1:8878'+('/api/status' if data is None else '/api/command')
with urllib.request.urlopen(urllib.request.Request(url,data=data,headers={'Content-Type':'application/json'}),timeout=10) as r: print(r.read().decode())
'''
        response=subprocess.check_output(['docker','exec',self.name,'python3','-c',script,json.dumps(payload)],timeout=15,text=True)
        return json.loads(response)
