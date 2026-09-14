"""Render source-backed report figures; requires matplotlib and a Korean font."""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

ASSETS = Path(__file__).resolve().parents[1] / "assets/uuv-update-20260914"
FONT = Path("/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
if FONT.exists():
    font_manager.fontManager.addfont(str(FONT))
plt.rcParams.update({"font.family": "NanumGothic", "axes.unicode_minus": False})
INK, MUTED, BLUE, TEAL = "#173047", "#526a7b", "#246ac1", "#14877f"


def canvas(title, subtitle):
    fig, ax = plt.subplots(figsize=(14, 8), dpi=150)
    fig.patch.set_facecolor("#f5f8fc")
    ax.set(xlim=(0, 14), ylim=(0, 8))
    ax.axis("off")
    ax.text(0.45, 7.5, title, fontsize=23, weight="bold", color=INK)
    ax.text(0.45, 7.03, subtitle, fontsize=12, color=MUTED)
    return fig, ax


def box(ax, x, y, w, h, title, body, color=BLUE):
    ax.add_patch(
        FancyBboxPatch(
            (x, y),
            w,
            h,
            boxstyle="round,pad=0.06",
            facecolor="white",
            edgecolor="#dce4ed",
            linewidth=1.2,
        )
    )
    ax.text(
        x + 0.22, y + h - 0.4, title, fontsize=15, color=color, weight="bold", va="top"
    )
    ax.text(
        x + 0.22, y + h - 0.95, body, fontsize=12, color=INK, va="top", linespacing=1.65
    )


def arrow(ax, start, end):
    ax.add_patch(
        FancyArrowPatch(
            start, end, arrowstyle="-|>", mutation_scale=17, color=MUTED, linewidth=1.7
        )
    )


def save(fig, name):
    fig.savefig(ASSETS / name, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def physics():
    fig, ax = canvas(
        "01  물리 모델 개선과 시각 효과의 경계",
        "MuJoCo + ArduSub 기반 ROV  |  누적 개발 및 2026-09-14 검증 상태",
    )
    box(
        ax,
        0.5,
        3.75,
        3.8,
        2.7,
        "분산 유체력 · 105점",
        "위치별 물살·잠김 비율 조회\n패치별 부력·항력 → 힘·토크 합산\n일괄 조회로 기존 수식 결과 보존",
    )
    box(
        ax,
        5.0,
        3.75,
        3.8,
        2.7,
        "동역학 · 제어 시간 정렬",
        "물리 간격 최대 0.5 ms\n400 Hz FCU 주기당 물리 5스텝\n초기 정렬 → 사용자 ARM → 해제",
    )
    box(
        ax,
        9.5,
        3.75,
        3.8,
        2.7,
        "접촉 · 줄 · 자석",
        "줄 6관절 × 부표 3개\n20 N 반력 분리 + 1 ms 확인\n수치 발산 시 진단 저장 후 정지",
    )
    arrow(ax, (4.35, 5.1), (4.94, 5.1))
    arrow(ax, (8.86, 5.1), (9.43, 5.1))
    box(
        ax,
        0.5,
        1.15,
        6.0,
        2.1,
        "조건부 실물 bag 응답 보정",
        "선택형 프로필 · 수평 추력/항력 유효 계수\n후기 속도 RMSE 0.5722 → 0.0538 m/s\n동일 bag 후보 선택 구간 · 독립 실물 검증 아님",
        TEAL,
    )
    box(
        ax,
        7.0,
        1.15,
        6.3,
        2.1,
        "별도 시각 경로 · 물리값 보존",
        "수면 파문·윤슬·깊이에 따른 조명 / 흰 밧줄·선택 초록색\n시각 변경 전후 기존 812개 물리 geom 및 질량·관성 동일\n물 입자 / 정밀 반사·굴절 / 실측 광학 모델은 미구현",
        TEAL,
    )
    ax.text(
        0.5,
        0.5,
        "20 N·줄 armature·마찰·유체 계수에는 가정이 포함됨. CFD나 실물 성능 인증을 뜻하지 않음.",
        fontsize=11,
        color=MUTED,
    )
    save(fig, "physics-overview.png")


def sensors():
    fig, ax = canvas(
        "02  센서 계약과 VLA 기록 흐름",
        "실물 인터페이스 호환  |  source 시각·수신 시각·좌표계·발행자 소유권 구분",
    )
    box(
        ax,
        0.5,
        3.45,
        3.7,
        3,
        "센서 · 실측 source 주기",
        "전방 / 손 영상     각 14.992 Hz\n원시 IMU                50 Hz\nAHRS                      10 Hz\n수심 / DVL            약 10 Hz",
    )
    box(
        ax,
        5.0,
        3.45,
        3.6,
        3,
        "시간 · 유효성 검사",
        "시뮬: ROS /clock 기준\n실물: 기존 wall clock 기준\n정지·역행 시 이전 캐시 무효화\nAHRS와 raw IMU 시각 분리",
    )
    box(
        ax,
        9.4,
        3.45,
        3.9,
        3,
        "10 Hz 수집 → 모델 입력",
        "state 23 / action 4\n설정·출처·종료 사유 보존\nLeRobot 변환 → U0 inspection\nconnection_check는 학습 거절",
    )
    arrow(ax, (4.28, 5.0), (4.93, 5.0))
    arrow(ax, (8.68, 5.0), (9.33, 5.0))
    box(
        ax,
        0.5,
        1.1,
        6.0,
        1.85,
        "명령 · 추정 · 정답 분리",
        "RC override → MAVROS → ArduSub → MuJoCo\n시뮬 GT는 평가용 / EKF·정책 입력으로 우회하지 않음",
        TEAL,
    )
    box(
        ax,
        7.0,
        1.1,
        6.3,
        1.85,
        "검증된 범위",
        "비무장 · 0입력 · 5 sim초 · 51행 · 로더 36청크\n실제 작업 시연 / 정책 학습 / 실물 전이는 아직 미검증",
        TEAL,
    )
    ax.text(
        0.5,
        0.48,
        "주기: 원시 IMU 수정 후 30초 관측. 전체 RTF 0.203인 당시 환경의 기록이며 현재 속도 보장값이 아님.",
        fontsize=10.5,
        color=MUTED,
    )
    save(fig, "sensor-contract.png")


def water_cost():
    rows = json.loads((ASSETS / "water-cost-summary.json").read_text())
    fig, ax = plt.subplots(figsize=(12, 6.9), dpi=160)
    fig.patch.set_facecolor("#f5f8fc")
    ax.set_facecolor("#f5f8fc")
    fig.subplots_adjust(left=0.13, right=0.94, top=0.77, bottom=0.23)
    groups = [
        ("stereo_left", "off"),
        ("stereo_left", "profile"),
        ("stereo_right", "off"),
        ("stereo_right", "profile"),
    ]
    labels = [
        "전방 · 감쇠/산란",
        "전방 · 전체 물 효과",
        "손 · 감쇠/산란",
        "손 · 전체 물 효과",
    ]
    for idx, (cam, mode) in enumerate(groups):
        vals = [
            r["total"]
            for r in rows
            if r["camera"] == cam and r["trial"].endswith("_" + mode)
        ]
        mean = sum(vals) / len(vals)
        ax.barh(idx, mean, height=0.55, color=TEAL if mode == "off" else BLUE)
        ax.scatter(vals, [idx] * len(vals), color=INK, s=28, zorder=3)
        ax.text(
            max(vals) + 0.22,
            idx,
            f"{min(vals):.2f}–{max(vals):.2f} ms",
            va="center",
            fontsize=12,
            color=INK,
        )
    ax.set_yticks(range(4), labels, fontsize=12)
    ax.invert_yaxis()
    ax.set_xlim(0, 14.4)
    ax.set_xlabel("카메라 한 프레임 캡처 + 후처리 시간 [ms]", fontsize=12)
    ax.spines[["top", "right", "left"]].set_visible(False)
    ax.xaxis.grid(True, alpha=0.15)
    ax.set_axisbelow(True)
    fig.text(
        0.06,
        0.91,
        "03  물 효과의 추가 처리 비용",
        fontsize=23,
        weight="bold",
        color=INK,
    )
    fig.text(
        0.06,
        0.855,
        "MuJoCo 3.12 · GLFW · RTX 5080 · 640×360 · 40프레임 × 2회 / warmup 8",
        fontsize=11.5,
        color=MUTED,
    )
    fig.text(
        0.06,
        0.13,
        "막대 = 두 실행 p50의 산술평균 / 점 = 각 실행 p50. 표기 범위는 반복 간 범위이며 신뢰구간이 아님.",
        fontsize=10.5,
        color=MUTED,
    )
    fig.text(
        0.06,
        0.075,
        "추가 약 7–8 ms/프레임. 윤슬·파문·입사광 감쇠 묶음의 비용이며, 전체 시뮬레이션 감속률이나 개별 효과 비용은 아님.",
        fontsize=10.5,
        color=MUTED,
    )
    save(fig, "water-cost.png")


if __name__ == "__main__":
    ASSETS.mkdir(parents=True, exist_ok=True)
    physics()
    sensors()
    water_cost()
