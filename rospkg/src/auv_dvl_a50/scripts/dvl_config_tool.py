#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
[Water Linked DVL A50 설정 및 상태 확인 도구]
=============================================================================

이 스크립트는 ROS 실행 여부와 관계없이, TCP/IP 소켓 통신을 사용하여
DVL A50의 현재 상태를 조회하거나 Range Mode(탐지 범위)를 설정합니다.

[사용 방법 예시]

1. 현재 DVL 설정 상태 조회 (기본 IP: 192.168.194.95)
   $ python3 dvl_config_tool.py --get

2. Range Mode를 'Auto'(자동)로 설정 (권장)
   $ python3 dvl_config_tool.py --set auto

3. Range Mode를 특정 모드로 고정 (예: Mode 1 = 0.3m ~ 3.0m)
   $ python3 dvl_config_tool.py --set =1
   * 참고: 쉘(Shell)에 따라 등호(=) 처리를 위해 따옴표("=1")를 쓰는 것이 좋습니다.

4. 탐색 범위를 특정 구간으로 제한 (예: Mode 2 이상, Mode 3 이하)
   $ python3 dvl_config_tool.py --set "2<=3"

5. DVL IP 주소가 다른 경우 (예: 192.168.2.95)
   $ python3 dvl_config_tool.py --ip 192.168.2.95 --get

=============================================================================
"""

import socket
import json
import argparse
import sys
import time

def send_command(ip, port, command_dict, timeout=2.0):
    """
    [통신 함수]
    JSON 형태의 명령어를 소켓으로 전송하고 응답을 받아옵니다.
    
    :param ip: DVL IP 주소
    :param port: DVL TCP 포트 (기본 16171)
    :param command_dict: 전송할 명령어 딕셔너리 (예: {"command": "get_config"})
    :return: 응답 JSON 객체 (Dictionary) 또는 None (실패 시)
    """
    # 중요: DVL 프로토콜은 줄바꿈 문자(\n)를 명령어의 끝으로 인식합니다.
    payload = json.dumps(command_dict) + "\n"
    
    sock = None
    try:
        # 소켓 생성 (IPv4, TCP)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((ip, port))
        
        # 명령어 전송
        sock.sendall(payload.encode('utf-8'))
        
        # 응답 수신 처리
        # 데이터가 조각나서 들어올 수 있으므로 한 줄(\n)이 완성될 때까지 읽습니다.
        received_data = ""
        start_time = time.time()
        while True:
            if time.time() - start_time > timeout:
                raise TimeoutError("응답 대기 시간 초과 (DVL이 켜져 있는지 확인하세요)")
                
            chunk = sock.recv(1).decode('utf-8', errors='ignore')
            if not chunk:
                raise ConnectionError("서버(DVL)와 연결이 끊어졌습니다.")
            
            received_data += chunk
            if chunk == '\n': # 줄바꿈 문자를 만나면 수신 종료
                break
                
        # 수신된 문자열을 JSON 객체로 변환
        return json.loads(received_data)

    except Exception as e:
        print(f"[Error] 통신 중 오류 발생: {e}")
        return None
    finally:
        if sock:
            sock.close()

def get_config(ip, port):
    """
    [조회 함수]
    'get_config' 명령을 보내 현재 DVL의 모든 설정을 확인합니다.
    """
    print(f"\n--- DVL 설정 조회 중... ({ip}:{port}) ---")
    cmd = {"command": "get_config"}
    
    response = send_command(ip, port, cmd)
    
    if response:
        if response.get("success"):
            # 전체 JSON 내용을 보기 좋게(Indented) 출력
            print("\n[전체 응답 데이터]")
            print(json.dumps(response, indent=4, ensure_ascii=False))
            
            # 사용자가 자주 확인하는 핵심 정보만 요약해서 출력
            result = response.get("result", {})
            print("\n" + "="*30)
            print(" [핵심 설정 요약]")
            print(f" * Range Mode (범위 모드) : {result.get('range_mode')}")
            print(f" * Speed of Sound (음속)  : {result.get('speed_of_sound')} m/s")
            print(f" * Acoustic Enabled       : {result.get('acoustic_enabled')}")
            print("="*30 + "\n")
        else:
            print(f"[실패] 설정을 가져오지 못했습니다. 메시지: {response.get('error_message')}")
    else:
        print("[오류] 응답을 받지 못했습니다.")

def set_range_mode(ip, port, mode):
    """
    [설정 함수]
    'set_config' 명령을 통해 range_mode 파라미터를 변경합니다.
    
    :param mode: 설정할 모드 문자열 (예: 'auto', '=1', '2<=3')
    """
    print(f"\n--- Range Mode 변경 요청: '{mode}' ---")
    
    # DVL 프로토콜에 맞춘 JSON 구조 생성
    cmd = {
        "command": "set_config",
        "parameters": {
            "range_mode": mode
        }
    }
    
    response = send_command(ip, port, cmd)
    
    if response:
        if response.get("success"):
            print(f"[성공] Range Mode가 정상적으로 '{mode}'(으)로 변경되었습니다.")
        else:
            # 입력값이 잘못되었거나 DVL이 명령을 거부한 경우
            print(f"[실패] 설정 변경 실패: {response.get('error_message')}")
            print(" -> 오타가 없는지, 지원하는 모드인지 확인해주세요.")
    else:
        print("[오류] 응답을 받지 못했습니다.")

def main():
    # 터미널 인자 파서 설정
    parser = argparse.ArgumentParser(description="Water Linked DVL A50 설정 제어 도구")
    
    # 1. IP 주소 옵션 (기본값은 사용자 환경에 맞춘 192.168.194.95)
    parser.add_argument("--ip", default="192.168.194.95", help="DVL IP 주소 (기본값: 192.168.194.95)")
    
    # 2. 포트 번호 옵션 (DVL 기본 포트 16171)
    parser.add_argument("--port", type=int, default=16171, help="TCP 포트 번호 (기본값: 16171)")
    
    # 3. 실행 모드 (조회 vs 설정) - 둘 중 하나는 반드시 선택해야 함
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--get", action="store_true", help="현재 DVL 설정을 조회합니다.")
    group.add_argument("--set", type=str, help="Range Mode를 변경합니다. (값: auto, =1, =2, 2<=3 등)")

    args = parser.parse_args()

    # 선택된 기능 실행
    if args.get:
        get_config(args.ip, args.port)
    elif args.set:
        set_range_mode(args.ip, args.port, args.set)
        
        # 설정 변경 후, 실제로 잘 적용되었는지 확인하기 위해 조회 기능 자동 실행
        print("\n[확인] 변경된 설정이 적용되었는지 확인합니다...")
        time.sleep(0.5) # DVL 처리 시간을 위해 0.5초 대기
        get_config(args.ip, args.port)

if __name__ == "__main__":
    main()