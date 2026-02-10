#!/bin/bash

# --- ROS 2용 hit25_interpreter 실행 래퍼 ---
# 1. PulseAudio를 안전하게 중지합니다.
# 2. ROS 2 Launch 파일을 실행합니다.
# 3. 스크립트가 종료될 때 PulseAudio를 다시 시작합니다.

cleanup() {
    echo ""
    echo "--- ROS 노드 종료. PulseAudio를 다시 시작합니다... ---"
    systemctl --user start pulseaudio.socket pulseaudio.service
}

trap cleanup EXIT

echo "--- 하이드로폰 사용을 위해 PulseAudio를 중지합니다... ---"
systemctl --user stop pulseaudio.socket pulseaudio.service

sleep 1

echo "--- HIT25 Interpreter Launch 파일을 실행합니다... ---"
ros2 launch hit25_interpreter interpreter.launch.py
