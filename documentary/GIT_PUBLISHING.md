# Git 게시 가이드

현재 작업공간을 그대로 `git add .` 하면 안 된다. 소스 외에 수 GB의 빌드 결과,
외부 체크아웃과 로그가 있고 `rospkg/src`에는 중첩 Git 저장소가 존재한다.

## 권장 저장소 경계

첫 공개는 하나의 상위 저장소에서 자체 개발 소스를 관리하는 monorepo가 가장
검토하기 쉽다. 단, upstream을 그대로 추적할 패키지는 submodule로 두는 편이
낫다.

| 범주 | 권장 처리 |
| --- | --- |
| `sim/current`, 루트 런처, `docs` | 상위 저장소에서 직접 추적 |
| 팀 소유 ROS 패키지 | monorepo로 흡수하거나 기존 팀 저장소를 submodule로 유지 |
| `robot_localization`, `dvl_msgs` 등 upstream | `.repos`/submodule로 고정 |
| `sim/current/assets/yolo/best.pt` | Git LFS |
| ArduPilot, QGC, DEB/ZIP, build/install/log | Git에서 제외 |

## 현재 중첩 저장소

다음 경로에는 자체 `.git`이 있다.

- `rospkg/src/dvl_msgs`
- `rospkg/src/kmu26_auv`
- `rospkg/src/kmu26_auv_buoy_vision_control`
- `rospkg/src/kmu26_auv_hydrophone`
- `rospkg/src/kmu26_auv_msg`
- `rospkg/src/kmu26_auv_web_gui`
- `rospkg/src/kmu26_pinger_homing`
- `rospkg/src/robot_localization`

일부 팀 패키지는 아직 로컬 수정 상태다. 이 `.git` 디렉터리를 무작정 삭제하면
브랜치와 복구 정보가 사라질 수 있으므로 자동으로 정리하지 않았다.

## 방식 A: monorepo

1. 각 중첩 저장소의 remote, branch, status와 필요한 commit을 백업한다.
2. 필요한 변경을 기존 원격에 먼저 commit/push하거나 patch로 보존한다.
3. 각 패키지의 내부 `.git`만 제거한 뒤 상위 저장소에서 파일로 추가한다.
4. upstream 출처와 기준 commit은 `rospkg/real_robot.repos`에 유지한다.

내부 `.git` 삭제는 복구가 어려운 작업이므로 대상과 백업을 확인한 뒤 수동으로
수행한다.

## 방식 B: submodule

1. 각 팀 패키지의 로컬 변경을 해당 원격 브랜치에 먼저 push한다.
2. 상위 저장소에서 패키지를 원격 URL과 commit이 고정된 submodule로 등록한다.
3. clone 안내에 `git clone --recurse-submodules`를 명시한다.

upstream과 팀 패키지의 독립 릴리스가 중요하면 이 방식이 적합하다.

## 최초 게시 전 점검

```bash
git lfs install
git init
git status --short --ignored
git add .gitattributes .gitignore README.md docs
git add sim/current rospkg/real_robot.repos
git status --short
```

`rospkg/src`는 위의 monorepo/submodule 결정을 마친 뒤 추가한다. commit 전에
다음 대용량 파일 검사를 권장한다.

```bash
find . -type f -size +50M \
  -not -path './.git/*' \
  -not -path './sim/ardupilot/*'
```

GitHub 일반 파일 제한을 넘는 AppImage/DEB가 staged되지 않았는지, `.pt`가 Git
LFS pointer로 처리되는지 확인한다.

## 게시 전 기능 점검

```bash
bash -n run_control_gui.sh sim/current/start_sitl_mujoco_mj311.sh
python3 -m py_compile sim/current/run_uuv_mujoco.py
python3 sim/current/tools/check_gui_start_contract.py
python3 sim/current/tools/check_dist_rc_override_path.py
```

배포 ZIP/DEB는 source repository release asset으로 올리고 Git history에는 넣지
않는다.
