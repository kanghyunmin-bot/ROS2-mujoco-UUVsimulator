#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as ticker  # 눈금 조정을 위한 모듈 임포트
from sensor_msgs.msg import BatteryState
import threading  # 1. 스레딩 라이브러리 임포트

# 데이터 저장 리스트
times = []
voltages = []
currents = []
battery_percentage = 0.0  # 배터리 잔량 (%)

# 적응형 그래프 범위 조정 변수
max_points = 100  # 그래프에 표시할 최대 데이터 개수

##### 사용자 정의 배터리 설정 #####
V_PER_CELL_FULL = 4.2  # (100%) 셀당 완충 전압
V_PER_CELL_EMPTY = 3.8 # (0%) 셀당 방전 전압
###################################

# 2. 스레드 동기화(데이터 보호)를 위한 Lock 객체 생성
data_lock = threading.Lock()

# 3. Figure와 Axes 객체를 전역으로 미리 생성
fig = plt.figure()
ax1 = fig.add_subplot(2, 1, 1)  # 전압 그래프용
ax2 = fig.add_subplot(2, 1, 2)  # 전류 그래프용

# 4. 배터리 텍스트 객체도 미리 생성 (위치 조정)

##### (수정) 실시간 V/A 텍스트 객체 추가 (창 상단) #####
voltage_text = fig.text(0.35, 0.96, "Voltage: N/A V", 
                         fontsize=12, color="blue", ha='center', va='top')
current_text = fig.text(0.65, 0.96, "Current: N/A A", 
                         fontsize=12, color="red", ha='center', va='top')
####################################################

# (창 하단)
percent_text = fig.text(0.35, 0.01, f"Battery: {battery_percentage:.1f}%", 
                         fontsize=12, color="green", ha='center')
cell_count_text = fig.text(0.65, 0.01, "Cells: N/A", 
                         fontsize=12, color="blue", ha='center')

# 5. 자동 감지된 셀 개수를 저장할 전역 변수 (0 = 아직 모름)
detected_cell_count = 0

def update_graph(i):
    # 6. 데이터 읽기/복사 전에 Lock을 획득
    with data_lock:
        local_times = list(times)
        local_voltages = list(voltages)
        local_currents = list(currents)
        local_percentage = battery_percentage
        local_cell_count = detected_cell_count
    
    # 7. 전체 Figure 대신 각 Axes만 클리어
    ax1.clear()
    ax2.clear()

    # 전압 그래프
    ax1.plot(local_times, local_voltages, label="Voltage (V)", color='blue')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Voltage (V)")
    ax1.legend(loc='upper left')
    
    # Y축 세부 눈금 설정
    ax1.yaxis.set_major_locator(ticker.MultipleLocator(5)) 
    ax1.yaxis.set_minor_locator(ticker.MultipleLocator(1))
    ax1.grid(True, which='both', linestyle='--', linewidth=0.5) 
    ax1.grid(True, which='major', linestyle='-', linewidth=0.7)
    
    ax1.set_ylim(0, 30) # Y축 고정

    # 전류 그래프
    ax2.plot(local_times, local_currents, label="Current (A)", color='red')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Current (A)")
    ax2.legend(loc='upper left')

    # Y축 세부 눈금 설정
    ax2.yaxis.set_major_locator(ticker.MultipleLocator(10))
    ax2.yaxis.set_minor_locator(ticker.MultipleLocator(1))
    ax2.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax2.grid(True, which='major', linestyle='-', linewidth=0.7)
    
    ax2.set_ylim(0, 80) # Y축 고정

    # 8. 텍스트 객체의 내용만 업데이트
    
    # (창 하단)
    percent_text.set_text(f"Battery: {local_percentage:.1f}%")
    cell_text = f"Cells: {local_cell_count}S" if local_cell_count > 0 else "Cells: N/A"
    cell_count_text.set_text(cell_text)

    ##### (수정) 실시간 V/A 텍스트 업데이트 #####
    if local_voltages: # 리스트가 비어있지 않다면
        voltage_text.set_text(f"Voltage: {local_voltages[-1]:.2f} V")
        current_text.set_text(f"Current: {local_currents[-1]:.2f} A")
    else: # 리스트가 비어있다면 (시작 직후)
        voltage_text.set_text("Voltage: N/A V")
        current_text.set_text("Current: N/A A")
    ##########################################

    # 9. 그래프 레이아웃 자동 조정 (텍스트와 겹치지 않도록)
    # (수정) 상단(0.95)과 하단(0.03)에 여백을 주도록 rect 수정
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])


def battery_callback(msg):
    global times, voltages, currents, battery_percentage
    global detected_cell_count 

    current_time = msg.header.stamp.to_sec()
    current_v = msg.voltage
    current_i = abs(msg.current)
    
    temp_detected_cell_count = detected_cell_count

    # 1. 셀 개수를 아직 모르는 경우(0)에만 감지 로직 실행
    if temp_detected_cell_count == 0:
        if len(msg.cell_voltage) > 0:
            temp_detected_cell_count = len(msg.cell_voltage)
            rospy.loginfo(f"Battery visualizer: Detected {temp_detected_cell_count}S battery via cell_voltage list.")
        elif current_v > V_PER_CELL_EMPTY: 
            avg_cell_v = (V_PER_CELL_FULL + V_PER_CELL_EMPTY) / 2.0
            estimated_cells = int(round(current_v / avg_cell_v))
            if estimated_cells > 0:
                temp_detected_cell_count = estimated_cells
                rospy.loginfo(f"Battery visualizer: Assuming {temp_detected_cell_count}S battery based on voltage {current_v:.2f}V.")
        else:
            rospy.logwarn_throttle(5.0, "Battery visualizer: Waiting for valid voltage to detect cell count...")

    # 2. 셀 개수가 성공적으로 감지된 경우(> 0)에만 백분율 계산
    if temp_detected_cell_count > 0:
        v_full = V_PER_CELL_FULL * temp_detected_cell_count
        v_empty = V_PER_CELL_EMPTY * temp_detected_cell_count
        
        if (v_full - v_empty) > 0: 
            percentage_calc = (current_v - v_empty) / (v_full - v_empty)
        else:
            percentage_calc = 0.0
            
        percentage_calc = max(0.0, min(1.0, percentage_calc)) 
        local_battery_percentage = percentage_calc * 100
    
    # 3. (안전장치) 셀 개수를 아직 모르는 경우 0%로 표시
    else:
        local_battery_percentage = 0.0

    # 10. Lock을 잡고 모든 전역 변수를 한 번에 업데이트
    with data_lock:
        times.append(current_time)
        voltages.append(current_v)
        currents.append(current_i)
        
        battery_percentage = local_battery_percentage 
        detected_cell_count = temp_detected_cell_count 

        if len(times) > max_points:
            times.pop(0)
            voltages.pop(0)
            currents.pop(0)

def listener():
    rospy.init_node('battery_visualizer', anonymous=True)
    rospy.Subscriber('/battery', BatteryState, battery_callback)

    # 11. rospy.spin()을 별도 스레드에서 실행
    spin_thread = threading.Thread(target=rospy.spin)
    spin_thread.daemon = True  
    spin_thread.start()

    # 실시간 그래프 애니메이션 실행 (9Hz)
    ani = animation.FuncAnimation(fig, update_graph, interval=111) 
    
    # 12. 메인 스레드에서 plt.show() 실행 (GUI 차단)
    plt.show()

if __name__ == '__main__':
    listener()