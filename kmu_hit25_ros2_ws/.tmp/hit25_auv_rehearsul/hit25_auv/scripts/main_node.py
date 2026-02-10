#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
main_node.py

AUV 제어 시스템의 메인 엔트리 포인트(Entry Point) 노드입니다.
이 노드는 다음과 같은 핵심 컴포넌트 클래스들을 초기화하고 실행합니다.

1. MapOdomBroadcaster: 'map' -> 'odom' TF를 담당 (map_odom_broadcaster.py)
2. PositionController: 위치/자세 제어 및 'odom' -> 'base_link' -> 'dvl' TF를 담당 (controller.py)

각 컴포넌트의 활성화 여부는 ROS 파라미터를 통해 제어할 수 있습니다.
"""

import rospy
import sys # sys 모듈 추가
import os
import rospkg

# --- 최상단 임포트 ---
# 이곳에서는 rospy와 같이 거의 항상 성공하는 기본 모듈만 둡니다.
# 프로젝트별 모듈 임포트는 main() 함수 내부로 이동시킵니다.

# (기존 try...except ImportError 블록은 여기서 삭제합니다)


def main():
    """
    hit25_auv 메인 노드.
    - MapOdomBroadcaster (map->odom TF)
    - PositionController (x,y,z,yaw 추종 + odom->base_link, base_link->dvl TF)
    를 초기화하고 ROS 스핀을 수행한다.
    """
    
    try:
        # --- 1. ROS 노드 초기화 ---
        # anonymous=False를 사용하여 이 노드가 단독으로 실행됨을 보장합니다.
        rospy.init_node("hit25_auv_main", anonymous=False)
        rospy.loginfo("Starting [hit25_auv_main] node...")

        # --- 2. 파라미터 로드 ---
        enable_map_odom = rospy.get_param("~enable_map_odom_broadcaster", True)
        enable_controller = rospy.get_param("~enable_controller", True)
        log_level = rospy.get_param("~log_level", "INFO") # 단순 로깅용

        # --- 3. 핵심 모듈 임포트 ---
        # rospy.init_node() 이후에 임포트하여, 실패 시 logfatal이 동작하도록 합니다.
        try:
            try:
                package_dir = rospkg.RosPack().get_path("hit25_auv")
            except rospkg.ResourceNotFound as e:
                raise ImportError(f"hit25_auv package not found: {e}")
            scripts_dir = os.path.join(package_dir, "scripts")
            if scripts_dir not in sys.path:
                sys.path.insert(0, scripts_dir)

            from controller import PositionController
            from map_odom_broadcaster import MapOdomBroadcaster

        except ImportError as e:
            rospy.logfatal(f"필수 제어 모듈 임포트 실패: {e}")
            rospy.logfatal("controller.py 또는 map_odom_broadcaster.py가")
            rospy.logfatal("main_node.py와 같은 'scripts/' 폴더에 있는지 확인하세요.")
            return # main 함수 종료
        except NameError as e:
            # 이 경우는 임포트 자체는 성공했으나, 내부의 다른 의존성이 실패한 경우일 수 있습니다.
            rospy.logfatal(f"모듈 로드 중 NameError 발생: {e}")
            return # main 함수 종료


        # --- 4. 시작 정보 로깅 ---
        rospy.loginfo("[hit25_auv_main] node started.")
        rospy.loginfo(f"  - enable_map_odom_broadcaster: {enable_map_odom}")
        rospy.loginfo(f"  - enable_controller: {enable_controller}")
        rospy.loginfo(f"  - (Configured log_level: {log_level})")


        # --- 5. 컴포넌트 인스턴스 생성 ---
        map_odom = None
        controller = None

        # MapOdomBroadcaster 생성
        if enable_map_odom:
            try:
                map_odom = MapOdomBroadcaster()
                rospy.loginfo("MapOdomBroadcaster created.")
            except Exception as e:
                rospy.logfatal(f"Failed to create MapOdomBroadcaster: {e}")
                rospy.signal_shutdown("Fatal error during MapOdomBroadcaster init")
                return # main 함수 종료
        else:
            rospy.loginfo("MapOdomBroadcaster is disabled by parameter.")

        # PositionController 생성
        if enable_controller:
            try:
                controller = PositionController()
                rospy.loginfo("PositionController created.")
            except Exception as e:
                rospy.logfatal(f"Failed to create PositionController: {e}")
                rospy.signal_shutdown("Fatal error during PositionController init")
                return # main 함수 종료
        else:
            rospy.loginfo("PositionController is disabled by parameter.")
            
        # --- 6. 컴포넌트 활성화 상태 확인 ---
        if not enable_map_odom and not enable_controller:
            rospy.logwarn(
                "No components enabled (both MapOdomBroadcaster and "
                "PositionController are disabled). Node will still spin "
                "but do nothing."
            )
            # 노드는 아무것도 하지 않더라도 spin()을 호출하여
            # ROS 마스터와의 연결을 유지하고, 종료 시그널을 받을 수 있도록 합니다.
        
        # --- 7. ROS Spin --- (주석 번호 수정)
        # 컴포넌트 내부의 rospy.Timer 콜백들이 이제 자동으로 동작합니다.
        # 이 노드는 rospy.spin()을 호출하여 종료 시그널을 기다립니다.
        rospy.loginfo("[hit25_auv_main] Initialization complete. Spinning...")
        
        rospy.spin() # Ctrl+C 또는 종료 시그널이 올 때까지 대기

    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt received, shutting down")
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException received, shutting down")
    except Exception as e:
        # 초기화 과정(rospy.init_node, get_param 등)에서 발생한 예외 처리
        rospy.logfatal(f"Unhandled exception during main node execution: {e}")
        import traceback
        traceback.print_exc()

    rospy.loginfo("[hit25_auv_main] node terminated.")


if __name__ == "__main__":
    """
    스크립트의 메인 실행 블록
    """
    try:
        main()
    except rospy.ROSInterruptException:
        # main() 함수가 시작되기 전(예: import 중)에 
        # ROS가 중단된 경우를 처리합니다.
        pass
