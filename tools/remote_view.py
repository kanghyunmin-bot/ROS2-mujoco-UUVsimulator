"""Read-only mjviser mirror of the live UUV simulator, rendered in the browser."""
import argparse
from pathlib import Path
import time

import mujoco
import numpy as np
import viser
from mjviser import ViserMujocoScene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state_dir', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8890)
    args = parser.parse_args()
    server = viser.ViserServer(host='127.0.0.1', port=args.port)
    server.scene.set_up_direction('+z')
    status = server.gui.add_markdown('시뮬레이터 연결 대기 중…')
    server.gui.add_markdown('**UUV 원격 3D · 읽기 전용**\n\n마우스로 회전·확대할 수 있습니다. 조종은 기존 8878 GUI에서 하세요.')
    scene = None
    generation = None
    last_stamp = None
    last_status = 0

    @server.on_client_connect
    def connected(client):
        client.camera.position = (3.0, -4.0, 2.5)
        client.camera.look_at = (0.0, 0.0, -0.5)

    try:
        while True:
            try:
                with np.load(args.state_dir / 'frame.npz', allow_pickle=False) as saved:
                    frame = {k: saved[k] for k in saved.files}
                age = time.time() - float(frame['wall_time'])
                if age > 2:
                    if time.monotonic() - last_status > 1:
                        status.content = f'**연결 중단/대기** · 마지막 상태 {age:.1f}초 전'
                        last_status = time.monotonic()
                    time.sleep(.1)
                    continue
                current = str(frame['generation'])
                if current != generation:
                    if scene is not None:
                        # Restarting the simulator changes the scene definition.
                        server.scene.reset()
                    model = mujoco.MjModel.from_binary_path(str(args.state_dir / 'model.mjb'))
                    scene = ViserMujocoScene(server, model, num_envs=1)
                    generation = current
                    last_stamp = None
                stamp = float(frame['wall_time'])
                if stamp != last_stamp:
                    scene.update_from_arrays(frame['xpos'][None],
                        frame['xmat'].reshape(1, -1, 3, 3),
                        frame['mocap_pos'][None], frame['mocap_quat'][None])
                    last_stamp = stamp
                if time.monotonic() - last_status > .5:
                    status.content = f"**실시간 연결** · 시뮬 {float(frame['sim_time']):.1f}초 · 상태 나이 {age*1000:.0f}ms · 최대 15fps"
                    last_status = time.monotonic()
            except (FileNotFoundError, EOFError, ValueError) as error:
                status.content = f'시뮬레이터 연결 대기: {type(error).__name__}'
                time.sleep(.2)
            time.sleep(1 / 30)
    finally:
        server.stop()


if __name__ == '__main__':
    main()
