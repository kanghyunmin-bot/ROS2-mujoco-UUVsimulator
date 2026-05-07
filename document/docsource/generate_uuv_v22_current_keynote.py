#!/usr/bin/env python3
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DOCSRC = Path(__file__).resolve().parent
DOC_DIR = ROOT / "document"
FIG_DIR = DOCSRC / "figures_v22_latest"
OUT_KEY = DOC_DIR / "uuv_mujoco_v22_current_keynote_kr.key"
OUT_PDF = DOC_DIR / "uuv_mujoco_v22_current_keynote_kr.pdf"
METRICS_PATH = DOCSRC / "uuv_v22_latest_report_metrics.json"
FOCUS_METRICS_PATH = DOCSRC / "uuv_v22_current_focus_metrics.json"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def fmt(value: float, digits: int = 2) -> str:
    return f"{value:.{digits}f}"


def as_applescript_string(text: str) -> str:
    parts = text.split("\n")
    if not parts:
        return '""'
    quoted = ['"' + part.replace("\\", "\\\\").replace('"', '\\"') + '"' for part in parts]
    return " & return & ".join(quoted)


def text_call(slide_var: str, text: str, x: int, y: int, w: int, h: int, size: int) -> str:
    return f"my addText({slide_var}, {as_applescript_string(text)}, {x}, {y}, {w}, {h}, {size})"


def image_call(slide_var: str, path: Path, x: int, y: int, w: int) -> str:
    return f'my addImage({slide_var}, "{path}", {x}, {y}, {w})'


def notes_call(slide_var: str, text: str) -> str:
    return f"my addNotes({slide_var}, {as_applescript_string(text)})"


def build_script() -> str:
    metrics = load_json(METRICS_PATH)
    focus = load_json(FOCUS_METRICS_PATH)

    current_avg = metrics["averages"]["current"]
    step = metrics["current_step_summary"]
    four = focus["variants"]["four_point"]
    center = focus["variants"]["center"]
    actual_ref = focus["actual_reference"]

    title_sub = "MuJoCo built-in ellipsoid fluid + Python thrust/hydrostatics\nlatest actual rosbag 기준 current 모델 설명"

    slide1_notes = "\n".join(
        [
            "오늘 발표는 UUV MuJoCo v2.2의 current 모델만 설명합니다.",
            "legacy는 왜 current 구조로 넘어왔는지 보여주는 배경 정도로만 짧게 언급하고, 본론은 current가 실제로 어떤 물리 구조로 짜여 있는지에 둡니다.",
            "핵심 질문은 세 가지입니다. 첫째, MuJoCo built-in이 어디까지 맡는지. 둘째, Python 커스텀이 어떤 물리를 추가로 구현하는지. 셋째, 그 설계가 latest actual rosbag과 비교했을 때 어느 정도 타당한지입니다.",
            "오늘 설명의 결론을 먼저 말하면, current는 물속 항력과 강체 적분은 MuJoCo ellipsoid fluid 모델에 맡기고, thrust 처리와 buoyancy, restoring torque는 Python에서 제어하는 분업 구조입니다.",
        ]
    )

    slide2_body = "\n".join(
        [
            "• 이번 발표는 current v2.2 모델만 설명",
            "• legacy는 구조 차이만 짧게 언급",
            "• 핵심 질문: built-in과 custom이 어디서 갈리고, thrust와 buoyancy를 왜 이렇게 구현했는가",
            "• 기준 데이터: latest actual rosbag + current repository snapshot",
            "",
            f"current actual 평균 path {fmt(current_avg['path_m'], 3)} m / horizontal {fmt(current_avg['horizontal_m'], 4)} m",
            f"current actual 평균 pitch {fmt(current_avg['pitch_deg'], 4)} deg / roll {fmt(current_avg['roll_deg'], 4)} deg",
        ]
    )
    slide2_notes = "\n".join(
        [
            "이 장에서는 왜 보고서를 current 중심으로 다시 구성했는지 설명하면 됩니다.",
            "legacy와 current를 같은 비중으로 섞어 설명하면 current의 설계 의도가 오히려 흐려집니다. 그래서 legacy는 구조 변화의 배경으로만 두고, current를 독립된 모델로 설명하는 구성을 택했습니다.",
            f"actual 기준선도 분명히 있습니다. latest rosbag 기준 current actual 평균 path는 {fmt(current_avg['path_m'], 3)}미터, horizontal drift는 {fmt(current_avg['horizontal_m'], 4)}미터, 평균 pitch와 roll은 각각 {fmt(current_avg['pitch_deg'], 4)}도, {fmt(current_avg['roll_deg'], 4)}도입니다.",
            "숫자 자체를 외울 필요는 없지만, 이 발표가 감각적인 설명이 아니라 실제 측정 기준을 가진 설명이라는 점을 여기서 먼저 잡아두면 됩니다.",
        ]
    )

    slide4_body = "\n".join(
        [
            "• MuJoCo built-in",
            "  - ellipsoid fluid drag",
            "  - viscous fluid force / torque",
            "  - rigid-body integration, mj_step()",
            "",
            "• Python custom",
            "  - thrust shaping and mixing",
            "  - buoyancy / restoring torque",
            "  - runtime mass / CoM / inertia",
            "",
            "• 핵심: drag proxy와 buoyancy point를 분리해서 씀",
        ]
    )
    slide3_notes = "\n".join(
        [
            "이 슬라이드는 전체 시스템에서 데이터가 어떻게 흘러가는지를 보여줍니다.",
            "입력은 QGC, ROS2, SITL servo 쪽에서 들어오고, ArduSub가 조종 모드와 모터 출력을 결정합니다.",
            "그 다음 MuJoCo 런타임의 Python 레이어가 이 출력을 실제 thruster command와 hydrostatics force로 바꿉니다.",
            "MuJoCo 본체는 ellipsoid fluid 기반 유체력과 rigid-body integration을 수행하고, 그 결과 pose, velocity, sensor truth가 다시 ROS2와 SITL 쪽으로 전달됩니다.",
            "즉 이 시스템은 MuJoCo 단독 구조가 아니라, ArduSub의 제어 출력과 Python 물리 래퍼를 MuJoCo built-in dynamics에 접합한 구조입니다.",
        ]
    )
    slide4_notes = "\n".join(
        [
            "이 장이 current 모델의 핵심 구조입니다.",
            "오른쪽 MuJoCo built-in은 ellipsoid fluid drag, viscous force와 torque, 그리고 최종 mj_step 적분을 담당합니다.",
            "왼쪽 Python 커스텀은 thrust shaping과 mixing, buoyancy, restoring torque, runtime mass와 CoM, inertia 세팅을 담당합니다.",
            "왜 이렇게 나눴냐 하면, drag와 rigid-body integration은 MuJoCo built-in이 더 일관되고 안정적이기 때문이고, thrust와 buoyancy는 실제 실험값에 맞게 우리가 직접 조정해야 하기 때문입니다.",
            "즉 current는 모든 것을 직접 구현한 시뮬레이터가 아니라, MuJoCo가 잘하는 부분은 맡기고 우리가 꼭 통제해야 하는 부분만 커스텀한 구조라고 설명하면 됩니다.",
        ]
    )

    slide5_body = "\n".join(
        [
            "• normalized command -> lag -> deadzone / clamp -> force curve -> data.ctrl",
            "• vertical_thruster_gain_scale, yaw_torque_scale가 뒤에서 추가 보정",
            "• current default 경로는 simple polynomial thrust model",
        ]
    )
    slide5_notes = "\n".join(
        [
            "쓰러스터는 입력값을 바로 힘으로 바꾸지 않습니다.",
            "normalized command가 먼저 들어오고, 여기에 deadzone 0.002와 1차 지연이 적용됩니다. rise와 decay는 각각 tau_up 0.04, tau_down 0.06으로 다르게 처리합니다.",
            "그 다음 forward와 reverse polynomial curve, gain_scale_all 2.9, reverse_asymmetry 0.7이 적용되고, 수직 thruster는 vertical_thruster_gain_scale 1.45로 추가 보정됩니다.",
            "yaw는 마지막에 yaw_torque_scale 1.5가 따로 걸립니다.",
            "중요한 점은 current 기본 SITL 경로가 performance table 직구동이 아니라 simple polynomial thrust model이라는 점입니다.",
        ]
    )

    slide6_body = "\n".join(
        [
            "• 좌측: 16V performance table 전체 곡선을 normalized command로 표현",
            "• 우측: 1500us 근처 center zoom",
            "• 핵심: 0-force plateau만 줄이고 1900us 끝단 값은 유지",
            "• 즉 초기 deadband만 줄였고 high-end thrust ceiling은 건드리지 않음",
        ]
    )
    slide6_notes = "\n".join(
        [
            "이 장은 runtime polynomial과의 비교가 아니라, performance table 자체를 설명하는 장입니다.",
            "왼쪽은 16볼트 performance table을 normalized command 축으로 옮겨서 전체 형상을 보여줍니다.",
            "여기서 중요한 것은 high-end thrust 값이 그대로 살아 있다는 점입니다. 1900마이크로초 끝단 값은 약 51.4뉴턴으로 유지됩니다.",
            "오른쪽은 1500마이크로초 근처를 확대해서 0-force plateau가 어디까지인지 보여줍니다.",
            "현재 테이블은 0뉴턴 구간을 1496마이크로초에서 1504마이크로초까지만 두고, 그 바깥은 즉시 작은 음수 또는 양수 thrust가 나오도록 줄였습니다.",
            "즉 이 장의 메시지는 최대 thrust를 줄였다는 것이 아니라, 중립 근처 deadband만 줄였다는 것입니다.",
        ]
    )

    slide7_body = "\n".join(
        [
            "• single-point center: 총 buoyancy를 한 점에 모아서 적용",
            "• current 4-point: 같은 총 buoyancy를 4개 upper point에 분산",
            "• 기대 효과: 과한 fake pitch 감소, restoring behavior 개선, heave/attitude 응답 안정화",
        ]
    )
    slide7_notes = "\n".join(
        [
            "이 슬라이드는 왜 current에서 buoyancy를 4-point로 나눴는지를 설명하는 장입니다.",
            "single-point center 방식은 총 buoyancy를 한 점에 몰아주기 때문에 모델은 단순하지만, 전진하거나 자세 교란이 들어왔을 때 비현실적인 attitude response가 쉽게 나옵니다.",
            "반면 current는 같은 총 buoyancy를 hull 상부의 네 점에 분산해서 적용합니다.",
            "여기에 buoyancy_scale 1.008, cob_x_offset 0.009미터, cob_z_offset 0.010미터가 더해져 실제적인 restoring behavior를 만들고 있습니다.",
            "핵심 메시지는 총 부력량보다도 부력의 공간 분포가 동역학에 훨씬 큰 영향을 준다는 점입니다.",
        ]
    )

    slide8_body = "\n".join(
        [
            "• same current scene / same thrust model / same scripted harness",
            "• single-point center는 forward에서 과한 pitch가 남음",
            "• 4-point는 release 안정성과 forward posture가 훨씬 정돈됨",
        ]
    )
    slide8_notes = "\n".join(
        [
            "이 비교는 buoyancy 구성만 바뀐 공정 비교라는 점을 먼저 강조해야 합니다.",
            "같은 current scene, 같은 thrust model, 같은 scripted harness에서 buoyancy 구성만 바꿨습니다.",
            f"single-point center는 forward step에서 peak pitch가 약 {fmt(center['forward_step']['peak_pitch_deg'], 2)}도까지 튀고, release 뒤에도 자세가 오래 흔들립니다.",
            f"반면 current 4-point는 forward peak pitch가 약 {fmt(four['forward_step']['peak_pitch_deg'], 2)}도 수준으로 내려오고, release 뒤 settling time도 약 {fmt(four['attitude_release']['settling_time_s'], 3)}초로 매우 짧습니다.",
            "즉 4-point buoyancy는 보기 좋은 그래픽 효과가 아니라, 과도한 nose-up과 느린 복원 문제를 줄이는 실제적인 설계 변경입니다.",
        ]
    )

    slide9_body = "\n".join(
        [
            f"• current 4-point gap index: {fmt(four['actual_gap_index_pct'], 2)}%",
            f"• single-point center gap index: {fmt(center['actual_gap_index_pct'], 2)}%",
            "",
            f"• 4-point forward surge {fmt(four['forward_step']['peak_surge_mps'], 3)} m/s",
            f"• 4-point forward pitch {fmt(four['forward_step']['peak_pitch_deg'], 2)} deg",
            f"• 4-point heave delta {fmt(four['heave_step']['depth_delta_mm'], 2)} mm",
            "",
            "• 결론: 4-point buoyancy는 cosmetic이 아니라 similarity를 만드는 핵심 설계",
        ]
    )
    slide9_notes = "\n".join(
        [
            "이 장에서는 similarity를 숫자로 정리합니다.",
            f"latest actual 기준 gap index를 보면 current 4-point는 {fmt(four['actual_gap_index_pct'], 2)}퍼센트이고, single-point center는 {fmt(center['actual_gap_index_pct'], 2)}퍼센트입니다.",
            "즉 전체 유사도 관점에서는 4-point가 압도적으로 actual에 가깝습니다.",
            f"세부적으로 4-point의 forward surge는 {fmt(four['forward_step']['peak_surge_mps'], 3)}미터퍼세컨드로 actual {fmt(actual_ref['forward_peak_surge_mps'], 3)}보다 약간 빠르고, forward pitch는 {fmt(four['forward_step']['peak_pitch_deg'], 2)}도로 actual {fmt(actual_ref['forward_peak_pitch_deg'], 2)}보다 조금 큽니다.",
            f"하지만 heave depth delta는 {fmt(four['heave_step']['depth_delta_mm'], 2)}밀리미터로 actual {fmt(actual_ref['heave_depth_delta_mm'], 2)}밀리미터에 매우 가깝고, center 모델에서 보이던 비현실적인 자세 붕괴도 없습니다.",
            "발표에서는 일부 단일 값만 보면 center가 덜 나빠 보일 수 있지만, 전체 posture stability와 similarity를 합치면 4-point가 훨씬 낫다고 정리하면 됩니다.",
        ]
    )

    slide10_body = "\n".join(
        [
            f"• stabilize forward peak surge {fmt(step['stabilize_forward_step']['surge_mps']['max'], 3)} m/s",
            f"• stabilize forward peak pitch {fmt(actual_ref['forward_peak_pitch_deg'], 2)} deg",
            f"• manual yaw peak rate {fmt(max(abs(step['manual_yaw_step']['yaw_rate_rad_s']['max']), abs(step['manual_yaw_step']['yaw_rate_rad_s']['min'])), 3)} rad/s",
            f"• manual heave depth delta {fmt(actual_ref['heave_depth_delta_mm'], 2)} mm",
            "",
            "• current 분석의 실제 기준선은 이 latest rosbag",
        ]
    )
    slide10_notes = "\n".join(
        [
            "이 장은 시뮬레이터가 맞춰야 하는 actual 기준선이 무엇인지 보여주는 장입니다.",
            f"stabilize forward에서 peak surge는 약 {fmt(step['stabilize_forward_step']['surge_mps']['max'], 3)}미터퍼세컨드, peak pitch는 약 {fmt(actual_ref['forward_peak_pitch_deg'], 2)}도입니다.",
            f"manual yaw에서는 peak yaw rate가 약 {fmt(max(abs(step['manual_yaw_step']['yaw_rate_rad_s']['max']), abs(step['manual_yaw_step']['yaw_rate_rad_s']['min'])), 3)}라디안퍼세컨드, manual heave에서는 depth delta가 약 {fmt(actual_ref['heave_depth_delta_mm'], 2)}밀리미터입니다.",
            "이 값들은 리포트 장식이 아니라 current 모델 튜닝의 기준선입니다.",
            "즉 시뮬레이션이 보기 좋게 움직이는 것보다, 이 actual 응답의 범위와 특성을 얼마나 따라가는지가 더 중요하다는 메시지를 주면 됩니다.",
        ]
    )

    slide11_body = "\n".join(
        [
            "• current = MuJoCo ellipsoid fluid + Python thrust/hydrostatics",
            "• thrust는 simple polynomial model, buoyancy는 4-point distribution이 핵심",
            "• 4-point buoyancy가 latest actual에 훨씬 가깝다",
            "• 다음 튜닝 포인트: thrust authority, drag proxy, mode-dependent response",
        ]
    )
    slide11_notes = "\n".join(
        [
            "마지막 정리는 구조, 선택, 결론 순서로 가져가면 됩니다.",
            "current v2.2는 MuJoCo built-in ellipsoid fluid와 Python thrust와 hydrostatics를 결합한 구조입니다.",
            "drag와 rigid-body integration은 MuJoCo에 맡기고, thrust shaping과 buoyancy, restoring torque는 Python이 맡는 분업 구조가 핵심입니다.",
            "그리고 current에서 가장 중요한 모델링 선택은 4-point buoyancy입니다. 이 선택이 forward posture, release stability, heave similarity를 latest actual에 훨씬 가깝게 만들었습니다.",
            "앞으로 남은 튜닝 포인트는 thrust authority, drag proxy 계수, mode별 응답 차이지만, 구조적인 방향 자체는 지금 current 쪽이 맞다고 정리하면 됩니다.",
        ]
    )

    lines = [
        'tell application "Keynote"',
        "activate",
        'set theDoc to make new document with properties {document theme:theme "기본 흰색", width:1920, height:1080}',
        'tell current slide of theDoc',
        f'set object text of default title item to {as_applescript_string("UUV MuJoCo v2.2 current 모델 발표")}',
        'set size of object text of default title item to 46',
        'set font of object text of default title item to "Apple SD Gothic Neo"',
        f'set object text of default body item to {as_applescript_string(title_sub)}',
        'set size of object text of default body item to 28',
        'set font of object text of default body item to "Apple SD Gothic Neo"',
        notes_call("current slide of theDoc", slide1_notes),
        'end tell',
        '',
        'tell theDoc',
        'set s2 to make new slide with properties {base slide:master slide "빈 페이지"}',
        'set s3 to make new slide with properties {base slide:master slide "빈 페이지"}',
        'set s4 to make new slide with properties {base slide:master slide "빈 페이지"}',
        'set s5 to make new slide with properties {base slide:master slide "빈 페이지"}',
        'set s6 to make new slide with properties {base slide:master slide "빈 페이지"}',
        'set s7 to make new slide with properties {base slide:master slide "빈 페이지"}',
        'set s8 to make new slide with properties {base slide:master slide "빈 페이지"}',
        'set s9 to make new slide with properties {base slide:master slide "빈 페이지"}',
        'set s10 to make new slide with properties {base slide:master slide "빈 페이지"}',
        'set s11 to make new slide with properties {base slide:master slide "빈 페이지"}',
        'end tell',
        '',
        text_call("s2", "2. 목적과 범위", 78, 34, 1500, 60, 52),
        text_call("s2", slide2_body, 92, 170, 1660, 640, 31),
        text_call("s2", "Slide 2 / 11", 82, 1006, 260, 30, 18),
        notes_call("s2", slide2_notes),
        '',
        text_call("s3", "3. 시스템 전체 구조", 78, 34, 1500, 60, 52),
        text_call("s3", "QGC / ArduSub / MuJoCo / ROS2 데이터 흐름", 82, 102, 900, 34, 24),
        image_call("s3", FIG_DIR / "uuv_v22_latest_system_block_diagram.png", 86, 156, 1720),
        text_call("s3", "Slide 3 / 11", 82, 1006, 260, 30, 18),
        notes_call("s3", slide3_notes),
        '',
        text_call("s4", "4. current 물리엔진 구조", 78, 34, 1500, 60, 52),
        text_call("s4", slide4_body, 86, 170, 660, 760, 28),
        image_call("s4", FIG_DIR / "uuv_v22_latest_current_engine_block_diagram.png", 812, 142, 1000),
        text_call("s4", "Slide 4 / 11", 82, 1006, 260, 30, 18),
        notes_call("s4", slide4_notes),
        '',
        text_call("s5", "5. current thrust handling path", 78, 34, 1500, 60, 52),
        text_call("s5", slide5_body, 86, 120, 1700, 120, 26),
        image_call("s5", FIG_DIR / "uuv_v22_latest_thruster_model_detail.png", 90, 250, 1710),
        text_call("s5", "Slide 5 / 11", 82, 1006, 260, 30, 18),
        notes_call("s5", slide5_notes),
        '',
        text_call("s6", "6. Thruster performance reference", 78, 34, 1500, 60, 52),
        text_call("s6", slide6_body, 86, 120, 1700, 126, 26),
        image_call("s6", FIG_DIR / "uuv_v22_latest_thruster_performance_curve.png", 90, 262, 1700),
        text_call("s6", "Slide 6 / 11", 82, 1006, 260, 30, 18),
        notes_call("s6", slide6_notes),
        '',
        text_call("s7", "7. buoyancy model design", 78, 34, 1500, 60, 52),
        text_call("s7", slide7_body, 86, 120, 1700, 126, 26),
        image_call("s7", FIG_DIR / "uuv_v22_latest_buoyancy_layout_compare.png", 90, 258, 1700),
        text_call("s7", "Slide 7 / 11", 82, 1006, 260, 30, 18),
        notes_call("s7", slide7_notes),
        '',
        text_call("s8", "8. center vs 4-point response 비교", 78, 34, 1500, 60, 52),
        image_call("s8", FIG_DIR / "uuv_v22_latest_buoyancy_variant_responses.png", 86, 158, 1180),
        text_call("s8", slide8_body, 1320, 188, 500, 330, 29),
        text_call("s8", "4-point는 release stability와 forward posture에서 확실히 더 정돈됨", 1320, 560, 500, 150, 29),
        text_call("s8", "Slide 8 / 11", 82, 1006, 260, 30, 18),
        notes_call("s8", slide8_notes),
        '',
        text_call("s9", "9. similarity metrics", 78, 34, 1500, 60, 52),
        image_call("s9", FIG_DIR / "uuv_v22_latest_buoyancy_variant_metrics.png", 86, 164, 1180),
        text_call("s9", slide9_body, 1320, 184, 500, 520, 29),
        text_call("s9", "Slide 9 / 11", 82, 1006, 260, 30, 18),
        notes_call("s9", slide9_notes),
        '',
        text_call("s10", "10. latest actual reference", 78, 34, 1500, 60, 52),
        image_call("s10", FIG_DIR / "uuv_v22_latest_actual_step_responses.png", 86, 168, 1180),
        text_call("s10", slide10_body, 1320, 190, 500, 420, 29),
        text_call("s10", "Slide 10 / 11", 82, 1006, 280, 30, 18),
        notes_call("s10", slide10_notes),
        '',
        text_call("s11", "11. takeaways", 78, 34, 1500, 60, 52),
        text_call("s11", slide11_body, 92, 180, 1640, 320, 34),
        text_call("s11", "핵심 한 줄: current는 drag는 MuJoCo에, thrust와 hydrostatics는 Python에 맡긴 구조다.", 92, 560, 1640, 90, 34),
        text_call("s11", "Slide 11 / 11", 82, 1006, 280, 30, 18),
        notes_call("s11", slide11_notes),
        '',
        f'save theDoc in POSIX file "{OUT_KEY}"',
        f'export theDoc to POSIX file "{OUT_PDF}" as PDF',
        'close theDoc saving no',
        'end tell',
        '',
        "on addText(theSlide, theText, posX, posY, itemW, itemH, ptSize)",
        'tell application "Keynote"',
        "tell theSlide",
        "set t to make new text item with properties {object text:theText, position:{posX, posY}, width:itemW, height:itemH}",
        'set font of object text of t to "Apple SD Gothic Neo"',
        "set size of object text of t to ptSize",
        "end tell",
        "end tell",
        "end addText",
        '',
        "on addImage(theSlide, imgPath, posX, posY, itemW)",
        'tell application "Keynote"',
        "tell theSlide",
        "make new image with properties {file:POSIX file imgPath, position:{posX, posY}, width:itemW}",
        "end tell",
        "end tell",
        "end addImage",
        '',
        "on addNotes(theSlide, noteText)",
        'tell application "Keynote"',
        "set presenter notes of theSlide to noteText",
        "end tell",
        "end addNotes",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    if OUT_KEY.exists():
        if OUT_KEY.is_dir():
            shutil.rmtree(OUT_KEY)
        else:
            OUT_KEY.unlink()
    if OUT_PDF.exists():
        OUT_PDF.unlink()

    script = build_script()
    subprocess.run(["osascript"], input=script, text=True, check=True)
    print(f"saved {OUT_KEY}")
    print(f"saved {OUT_PDF}")


if __name__ == "__main__":
    main()
