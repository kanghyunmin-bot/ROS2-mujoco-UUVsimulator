# ArduSub Docker Backend

이 백엔드는 ArduSub SITL만 Ubuntu 컨테이너에서 실행하고, MuJoCo/GUI/ROS2/QGC는 Mac 호스트에서 실행한다.

포트 흐름:

- ArduSub JSON servo: Docker -> Mac `host.docker.internal:9002`
- MuJoCo JSON sensor: Mac -> Docker가 보낸 servo packet의 source socket으로 reply
- QGC MAVLink: Docker -> Mac `host.docker.internal:14550`
- ROS/MAVROS MAVLink: Docker -> Mac `host.docker.internal:14551`
- MuJoCo SERVO_OUTPUT_RAW: Docker -> Mac `host.docker.internal:14660`

컨테이너 entrypoint는 `host.docker.internal`을 IPv4 숫자 주소로 변환해서 ArduSub에 넘긴다. ArduPilot 4.1.2 SITL의 JSON socket은 hostname을 직접 해석하지 못한다.

직접 실행:

```bash
cd /Users/kanghyunmin/Desktop/uuv_sim
docker compose -f docker/ardusub/docker-compose.yml up --build
```

GUI에서 Docker 백엔드를 쓰려면:

```bash
cd /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current
UUV_SITL_BACKEND=docker ./run_control_gui.sh
```
