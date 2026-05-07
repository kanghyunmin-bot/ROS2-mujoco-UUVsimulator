#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import textwrap
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DOCSRC = Path(__file__).resolve().parent
TEX_PATH = DOCSRC / "uuv_mujoco_v22_latest_report_kr.tex"
METRICS_PATH = DOCSRC / "uuv_v22_latest_report_metrics.json"
FOCUS_METRICS_PATH = DOCSRC / "uuv_v22_current_focus_metrics.json"
SIM_PROFILES_PATH = ROOT / "uuv_mujoco" / "v2.2" / "config" / "sim_profiles.json"
THRUSTER_PARAMS_PATH = ROOT / "uuv_mujoco" / "v2.2" / "config" / "thruster_params.json"
CURRENT_MEAS_PATH = DOCSRC / "measurement_current_mode_path_latest_v30.json"
CURRENT_STEP_SUMMARY_PATH = DOCSRC / "measurement_summary_current_heavefix_latest_v30.json"

MODE_ORDER = ("manual", "stabilize", "alt_hold", "poshold")
MODE_LABELS = {
    "manual": "MANUAL",
    "stabilize": "STABILIZE",
    "alt_hold": "ALT\\_HOLD",
    "poshold": "POSHOLD",
}


def load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def fmt(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}"


def fmt_trim(value: float, digits: int = 4) -> str:
    out = f"{value:.{digits}f}".rstrip("0").rstrip(".")
    return out if out else "0"


def fmt_pct(value: float, digits: int = 2) -> str:
    return f"{value:.{digits}f}"


def vec(values: list[float], digits: int = 3) -> str:
    return "(" + ", ".join(fmt_trim(float(v), digits) for v in values) + ")"


def vec_math(values: list[float], digits: int = 3) -> str:
    return "$" + vec(values, digits) + "$"


def vec_sci_math(values: list[float], digits: int = 3) -> str:
    return "$(" + ", ".join(f"{float(v):.{digits}e}" for v in values) + ")$"


def code(text: str) -> str:
    return r"\code{" + text + "}"


def build_current_mode_rows(current_meas: dict) -> str:
    rows: list[str] = []
    for mode in MODE_ORDER:
        summary = current_meas["modes"][mode]["summary"]
        rows.append(
            f"{MODE_LABELS[mode]} & "
            f"{fmt(float(summary['trajectory_path_length_m']), 3)} & "
            f"{fmt(float(summary['horizontal_distance_m']), 3)} & "
            f"{fmt(float(summary['max_abs_pitch_deg']), 3)} & "
            f"{fmt(float(summary['max_abs_roll_deg']), 3)} \\\\"
        )
    return "\n".join(rows)


def build_scene_rows(scene_geoms: list[dict]) -> str:
    rows: list[str] = []
    for geom in scene_geoms:
        fluidcoef = geom["fluidcoef"]
        coef_text = (
            r"\shortstack[l]{"
            + f"{fmt(fluidcoef[0], 3)}, {fmt(fluidcoef[1], 3)},\\\\"
            + f"{fmt(fluidcoef[2], 3)}, {fmt(fluidcoef[3], 0)}, {fmt(fluidcoef[4], 0)}"
            + "}"
        )
        rows.append(
            f"{code(geom['name'])} & {vec_math(geom['pos'], 3)} & {vec_math(geom['size'], 3)} & {coef_text} \\\\"
        )
    return "\n".join(rows)


def build_tex() -> str:
    metrics = load_json(METRICS_PATH)
    focus = load_json(FOCUS_METRICS_PATH)
    profiles = load_json(SIM_PROFILES_PATH)
    thruster_params = load_json(THRUSTER_PARAMS_PATH)
    current_meas = load_json(CURRENT_MEAS_PATH)
    current_step = load_json(CURRENT_STEP_SUMMARY_PATH)["summary"]

    current_profile = profiles["current"]
    thr_global = thruster_params["global"]
    current_avg = metrics["averages"]["current"]
    reductions = metrics["reductions_percent"]
    scene = metrics["scene"]
    current_mass = metrics["runtime_mass_properties"]["current"]
    current_focus = focus["variants"]["four_point"]
    center_focus = focus["variants"]["center"]
    actual_ref = focus["actual_reference"]
    comparison_cmd = focus["comparison_commands"]

    manual_forward_max = float(current_step["manual_forward_step"]["surge_mps"]["max"])
    stabilize_forward_max = float(current_step["stabilize_forward_step"]["surge_mps"]["max"])
    stabilize_pitch_peak_deg = math.degrees(float(current_step["stabilize_forward_step"]["pitch_rad"]["max"]))
    stabilize_roll_peak_deg = math.degrees(float(current_step["stabilize_forward_step"]["roll_rad"]["max"]))
    manual_yaw_peak = max(
        abs(float(current_step["manual_yaw_step"]["yaw_rate_rad_s"]["max"])),
        abs(float(current_step["manual_yaw_step"]["yaw_rate_rad_s"]["min"])),
    )
    manual_heave_delta_mm = abs(float(current_step["manual_heave_step"]["depth_m"]["delta"])) * 1000.0
    alt_hold_heave_delta_mm = abs(float(current_step["alt_hold_heave_step"]["depth_m"]["delta"])) * 1000.0

    mode_rows = build_current_mode_rows(current_meas)
    scene_rows = build_scene_rows(scene["fluid_geoms"])

    template = textwrap.dedent(
        r"""
        \documentclass[11pt,a4paper]{article}

        \usepackage{fontspec}
        \usepackage[margin=21mm]{geometry}
        \usepackage{amsmath,amssymb}
        \usepackage{booktabs,longtable,array,tabularx}
        \usepackage{graphicx}
        \usepackage{caption}
        \usepackage{subcaption}
        \usepackage{xcolor}
        \usepackage{hyperref}
        \usepackage{enumitem}
        \usepackage{float}

        \defaultfontfeatures{Ligatures=TeX,Scale=MatchLowercase}
        \IfFontExistsTF{Noto Sans CJK KR}{
          \setmainfont{Noto Sans CJK KR}[
            ItalicFeatures={FakeSlant=0.18},
            BoldItalicFeatures={FakeSlant=0.18}
          ]
          \setsansfont{Noto Sans CJK KR}[
            ItalicFeatures={FakeSlant=0.18},
            BoldItalicFeatures={FakeSlant=0.18}
          ]
        }{
          \IfFontExistsTF{Apple SD Gothic Neo}{
            \setmainfont{Apple SD Gothic Neo}[
              ItalicFeatures={FakeSlant=0.18},
              BoldItalicFeatures={FakeSlant=0.18}
            ]
            \setsansfont{Apple SD Gothic Neo}[
              ItalicFeatures={FakeSlant=0.18},
              BoldItalicFeatures={FakeSlant=0.18}
            ]
          }{
            \setmainfont{NanumGothic}[
              ItalicFeatures={FakeSlant=0.18},
              BoldItalicFeatures={FakeSlant=0.18}
            ]
            \setsansfont{NanumGothic}[
              ItalicFeatures={FakeSlant=0.18},
              BoldItalicFeatures={FakeSlant=0.18}
            ]
          }
        }
        \IfFontExistsTF{DejaVu Sans Mono}{
          \setmonofont{DejaVu Sans Mono}
        }{
          \IfFontExistsTF{Menlo}{
            \setmonofont{Menlo}
          }{
            \setmonofont{Liberation Mono}
          }
        }
        \XeTeXlinebreaklocale "ko"
        \XeTeXlinebreakskip = 0pt plus 1pt

        \hypersetup{
          colorlinks=true,
          linkcolor=blue!50!black,
          urlcolor=blue!50!black,
          citecolor=blue!50!black,
          pdftitle={UUV MuJoCo v2.2 current-focus technical report},
          pdfauthor={OpenAI Codex}
        }

        \setlength{\parskip}{0.52em}
        \setlength{\parindent}{0pt}
        \setlength{\emergencystretch}{3em}
        \renewcommand{\arraystretch}{1.22}
        \newcolumntype{Y}{>{\raggedright\arraybackslash}X}
        \newcommand{\code}[1]{\nolinkurl{#1}}
        \captionsetup{
          font=small,
          labelfont=bf,
          justification=raggedright,
          singlelinecheck=false,
          skip=6pt
        }

        \title{UUV MuJoCo v2.2 current 모델 중심 기술 보고서\\
        \large docsource 최신 actual 데이터와 현재 repository 설정을 함께 반영한 재정리본}
        \author{}
        \date{@@REPORT_DATE@@}

        \begin{document}
        \maketitle
        \tableofcontents
        \clearpage

        \section{문서 범위와 데이터 기준}

        이번 문서는 \textbf{legacy를 길게 비교하는 보고서가 아니라, current 모델을 해설하는 문서}로 다시 정리했다. 따라서 legacy는 구조적 차이만 짧게 적고, 본문 대부분은 current runtime의 물리엔진 분해, thrust path, buoyancy model 비교, latest actual reference에 집중한다.

        직접 사용한 입력은 아래와 같다.

        \begin{itemize}[leftmargin=2em]
          \item @@METRICS_JSON@@
          \item @@FOCUS_JSON@@
          \item @@CURRENT_JSON@@
          \item @@STEP_JSON@@
          \item @@SIM_PROFILES@@
          \item @@THRUSTER_PARAMS@@
        \end{itemize}

        여기서 숫자는 두 그룹으로 구분했다.

        \begin{enumerate}[leftmargin=2em]
          \item \textbf{latest actual reference}: @@MEASUREMENT_DATE@@ rosbag에서 뽑은 current mode-path / step summary
          \item \textbf{current repository snapshot}: 보고서 생성 시점의 \code{sim\_profiles.json}, \code{thruster\_params.json}, \code{tank\_current\_scene.xml}
        \end{enumerate}

        \section{한 장 요약}

        \begin{itemize}[leftmargin=2em]
          \item current v2.2는 \textbf{MuJoCo built-in ellipsoid fluid} 와 \textbf{Python custom thrust / hydrostatics} 를 결합한 하이브리드 구조다.
          \item legacy 대비 current latest actual 평균값은 trajectory path @@PATH_REDUCTION@@\%, horizontal distance @@HORIZ_REDUCTION@@\%, max pitch @@PITCH_REDUCTION@@\%, max roll @@ROLL_REDUCTION@@\% 감소다. 여기서는 이 숫자를 결과 컨텍스트로만 사용하고, 본문 분석은 current에 집중한다.
          \item current snapshot의 핵심 값은 \code{thruster\_force\_max} = @@THRUSTER_FORCE_MAX@@\,N, \code{vertical\_thruster\_gain\_scale} = @@VERT_GAIN@@, \code{yaw\_torque\_scale} = @@YAW_TORQUE_SCALE@@, \code{buoyancy\_scale} = @@BUOYANCY_SCALE@@, \code{cob\_torque\_scale} = @@COB_TORQUE_SCALE@@ 다.
          \item thrust path는 \textbf{simple polynomial thruster model} 이고, shaping 파라미터는 \code{deadzone} = @@DEADZONE@@, \code{tau\_up} = @@TAU_UP@@\,s, \code{tau\_down} = @@TAU_DOWN@@\,s, \code{gain\_scale\_all} = @@GAIN_SCALE@@, \code{reverse\_asymmetry} = @@REVERSE_ASYM@@ 다.
          \item same current scene / same current thrust model / same scripted harness에서 비교하면 4-point buoyancy의 latest actual gap index는 @@FOUR_GAP@@\%, single-point center 모델은 @@CENTER_GAP@@\% 로 계산됐다.
        \end{itemize}

        \section{legacy와 current의 구조적 차이}

        legacy는 이 보고서의 중심이 아니므로 구조적 차이만 남긴다.

        \begin{center}
        \small
        \begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.26\linewidth}YY}
        \toprule
        항목 & legacy 구조 & current 구조 \\
        \midrule
        수중 유체력 주체 & custom 6-DOF hydrodynamics 중심 & MuJoCo ellipsoid fluid 중심 \\
        active hydro terms & added mass, Coriolis, linear / quadratic damping 사용 & 위 항들은 기본 current 경로에서 비활성화 \\
        hydrostatics & component / CoB 기반 복원토크 중심 & distributed 4-point buoyancy + restoring torque \\
        thrust path & Python shaping + actuator 적용 & Python shaping + actuator 적용 \\
        drag proxy scene & legacy scene / custom tuning & \code{tank\_current\_scene.xml} + ellipsoid fluid geoms \\
        \bottomrule
        \end{tabularx}
        \end{center}

        즉 current를 짧게 정의하면 \textbf{drag는 MuJoCo에 맡기고, thrust와 hydrostatics는 Python이 감싼 구조}다.

        \section{current 물리엔진 블록 다이어그램}

        \begin{figure}[H]
          \centering
          \includegraphics[width=0.88\linewidth]{figures_v22_latest/uuv_v22_latest_current_engine_block_diagram.png}
          \caption{current 모델의 물리엔진 블록 다이어그램. 입력 명령에서 thrust shaping을 거쳐 \code{data.ctrl} 와 extra yaw torque가 만들어지고, hydrostatics와 MuJoCo ellipsoid fluid가 합쳐진 뒤 \code{mj\_step()} 으로 적분된다.}
        \end{figure}

        \begin{center}
        \small
        \begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.28\linewidth}YY}
        \toprule
        블록 & 엔진 & current에서 실제로 하는 일 \\
        \midrule
        Input command & 외부 입력 & QGC / ROS2 / SITL servo가 thrust request를 만든다 \\
        Python custom Thruster Model & Python & deadzone, lag, force mapping, horizontal mixing, vertical gain, extra yaw torque \\
        Python custom Hydrostatics & Python & buoyancy, 4-point force distribution, restoring torque, runtime mass / CoM / inertia 재설정 \\
        Ellipsoid Fluid & MuJoCo built-in & ellipsoid fluid drag, viscous fluid force / torque \\
        Integration / Sensors & MuJoCo built-in & \code{mj\_step()}, rigid-body integration, 기본 센서 primitive \\
        \bottomrule
        \end{tabularx}
        \end{center}

        \begin{figure}[H]
          \centering
          \includegraphics[width=0.88\linewidth]{figures_v22_latest/uuv_v22_latest_ellipsoid_layout.png}
          \caption{current scene의 ellipsoid drag proxy, buoyancy point, mass component, CoM, CoB 배치. current는 drag proxy와 buoyancy application point를 분리해서 쓰는 구조다.}
        \end{figure}

        \begin{center}
        \small
        \begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.30\linewidth}YY}
        \toprule
        항목 & current 값 & 의미 \\
        \midrule
        proxy total volume & @@PROXY_VOLUME@@\,m$^3$ & MuJoCo ellipsoid drag 계산에 쓰이는 proxy 부피 합 \\
        neutral volume & @@NEUTRAL_VOLUME@@\,m$^3$ & 15\,kg 기체의 중성부력 기준량 \\
        CoM & @@CURRENT_COM@@ & runtime mass synthesis 결과 \\
        CoB & @@CURRENT_COB@@ & current hydrostatics 기준 부력 중심 \\
        runtime inertia & @@CURRENT_INERTIA_SCALED@@ & current runtime에서 실제 적용되는 관성 \\
        \bottomrule
        \end{tabularx}
        \end{center}

        \begin{center}
        \small
        \begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.28\linewidth}YYY}
        \toprule
        Geom & position [m] & semi-axis [m] & fluidcoef \\
        \midrule
        @@SCENE_ROWS@@
        \bottomrule
        \end{tabularx}
        \end{center}

        \section{current thrust model}

        \begin{figure}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{figures_v22_latest/uuv_v22_latest_thruster_model_detail.png}
          \caption{current thrust path 상세. 위는 pipeline, 아래는 command shaping / force mapping / lag behavior를 current 설정값으로 그린 것이다.}
        \end{figure}

        \begin{figure}[H]
          \centering
          \includegraphics[width=0.96\linewidth]{figures_v22_latest/uuv_v22_latest_thruster_performance_curve.png}
          \caption{thruster performance reference graph. 좌측은 \code{thruster\_performance.json} 의 16V PWM-force 테이블이고, 우측은 그 테이블을 normalized command로 펼친 곡선과 current simple polynomial runtime model을 겹친 것이다. 즉 performance table은 reference 데이터이고, current default SITL path는 오른쪽의 simple polynomial model로 읽는 것이 맞다.}
        \end{figure}

        \begin{center}
        \small
        \begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.42\linewidth}Y}
        \toprule
        thrust 항목 & current 값 \\
        \midrule
        \code{deadzone} & @@DEADZONE@@ \\
        \code{tau\_up}, \code{tau\_down} & @@TAU_UP@@\,s / @@TAU_DOWN@@\,s \\
        \code{reverse\_asymmetry} & @@REVERSE_ASYM@@ \\
        \code{gain\_scale\_all} & @@GAIN_SCALE@@ \\
        \code{command\_limit} & @@COMMAND_LIMIT@@ \\
        \code{forward\_poly} & @@FORWARD_POLY@@ \\
        \code{reverse\_poly} & @@REVERSE_POLY@@ \\
        \code{vertical\_thruster\_gain\_scale} & @@VERT_GAIN@@ \\
        \code{yaw\_torque\_scale} & @@YAW_TORQUE_SCALE@@ \\
        \bottomrule
        \end{tabularx}
        \end{center}

        thrust 처리 순서는 다음 한 줄로 정리하면 충분하다.

        \begin{quote}
        normalized command $\rightarrow$ first-order lag $\rightarrow$ deadzone / saturation $\rightarrow$ forward-reverse polynomial force curve $\rightarrow$ per-axis gain $\rightarrow$ \code{data.ctrl} + extra yaw torque
        \end{quote}

        \section{중심부력 vs 4-point 부력 비교}

        current는 기본적으로 4-point buoyancy를 사용한다. 비교를 위해 같은 current scene, 같은 current thrust model, 같은 runtime mass / inertia, 같은 scripted harness에서 \textbf{single-point center} 와 \textbf{current 4-point} 를 나란히 돌렸다.

        \begin{figure}[H]
          \centering
          \includegraphics[width=0.96\linewidth]{figures_v22_latest/uuv_v22_latest_buoyancy_layout_compare.png}
          \caption{single-point center와 4-point buoyancy의 작용점 차이. center 모델은 총 buoyancy를 한 점에 모아서 적용하고, current 4-point는 같은 총 buoyancy를 네 점에 나눠 적용한다.}
        \end{figure}

        이번 비교에서 쓴 direct harness command는 forward @@COMPARE_FORWARD@@, heave @@COMPARE_HEAVE@@ 이다. forward는 latest step의 nominal command를 그대로 사용했고, heave는 direct harness가 ArduSub RC/servo mapping을 건너뛰기 때문에 latest manual depth delta를 기준으로 4-point 모델이 맞는 수준으로 보정했다.

        latest actual reference는 아래 네 값을 썼다.

        \begin{itemize}[leftmargin=2em]
          \item forward peak surge: latest \code{stabilize\_forward\_step}
          \item forward peak pitch / roll: latest \code{stabilize\_forward\_step}
          \item heave depth delta: latest \code{manual\_heave\_step}
        \end{itemize}

        \begin{center}
        \small
        \begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.28\linewidth}YYY}
        \toprule
        항목 & single-point center & current 4-point & latest actual ref \\
        \midrule
        forward peak surge [m/s] & @@CENTER_SURGE@@ & @@FOUR_SURGE@@ & @@REF_SURGE@@ \\
        forward peak $|$pitch$|$ [deg] & @@CENTER_PITCH@@ & @@FOUR_PITCH@@ & @@REF_PITCH@@ \\
        forward peak $|$roll$|$ [deg] & @@CENTER_ROLL@@ & @@FOUR_ROLL@@ & @@REF_ROLL@@ \\
        heave depth delta [mm] & @@CENTER_HEAVE@@ & @@FOUR_HEAVE@@ & @@REF_HEAVE@@ \\
        \bottomrule
        \end{tabularx}
        \end{center}

        \begin{center}
        \small
        \begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.30\linewidth}YY}
        \toprule
        stability / similarity 지표 & single-point center & current 4-point \\
        \midrule
        attitude release RMS angle [deg] & @@CENTER_RMS@@ & @@FOUR_RMS@@ \\
        attitude release settling time [s] & @@CENTER_SETTLE@@ & @@FOUR_SETTLE@@ \\
        actual gap index [\%] & @@CENTER_GAP@@ & @@FOUR_GAP@@ \\
        \bottomrule
        \end{tabularx}
        \end{center}

        \begin{figure}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{figures_v22_latest/uuv_v22_latest_buoyancy_variant_responses.png}
          \caption{same current harness에서 center / 4-point buoyancy response 비교. 4-point는 forward pitch와 attitude release가 훨씬 빨리 수렴하고, heave에서도 latest actual reference와 같은 order의 depth delta를 만든다.}
        \end{figure}

        \begin{figure}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{figures_v22_latest/uuv_v22_latest_buoyancy_variant_metrics.png}
          \caption{center / 4-point / latest actual reference의 key metric 비교와 gap summary. current 4-point의 mean gap이 center보다 크게 낮다.}
        \end{figure}

        \section{latest current actual reference}

        actual current latest bag만 따로 보면 평균 motion envelope는 아래와 같다.

        \begin{center}
        \small
        \begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.36\linewidth}Y}
        \toprule
        current actual 평균값 & 값 \\
        \midrule
        평균 trajectory path length & @@CURRENT_PATH@@\,m \\
        평균 horizontal distance & @@CURRENT_HORIZ@@\,m \\
        평균 max $|$pitch$|$ & @@CURRENT_PITCH@@$^\circ$ \\
        평균 max $|$roll$|$ & @@CURRENT_ROLL@@$^\circ$ \\
        \bottomrule
        \end{tabularx}
        \end{center}

        \begin{center}
        \small
        \begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.18\linewidth}YYYY}
        \toprule
        모드 & path [m] & horizontal [m] & pitch [deg] & roll [deg] \\
        \midrule
        @@CURRENT_MODE_ROWS@@
        \bottomrule
        \end{tabularx}
        \end{center}

        \begin{center}
        \small
        \begin{tabularx}{\linewidth}{>{\raggedright\arraybackslash}p{0.40\linewidth}YY}
        \toprule
        latest step summary 항목 & 값 & 해석 \\
        \midrule
        manual forward peak surge & @@MANUAL_FORWARD@@\,m/s & current manual forward authority \\
        stabilize forward peak surge & @@STAB_FORWARD@@\,m/s & stabilize에서도 surge authority는 유지 \\
        stabilize forward peak pitch & @@STAB_PITCH@@$^\circ$ & forward 입력 시 posture coupling \\
        stabilize forward peak roll & @@STAB_ROLL@@$^\circ$ & roll coupling은 pitch보다 작음 \\
        manual yaw peak rate & @@MANUAL_YAW@@\,rad/s & yaw authority \\
        manual heave depth delta & @@MANUAL_HEAVE_MM@@\,mm & latest manual heave response \\
        alt-hold heave depth delta & @@ALT_HEAVE_MM@@\,mm & alt-hold heave response \\
        \bottomrule
        \end{tabularx}
        \end{center}

        \begin{figure}[H]
          \centering
          \includegraphics[width=0.98\linewidth]{figures_v22_latest/uuv_v22_latest_actual_step_responses.png}
          \caption{latest heavefix step bag 기반 current actual response. current 해석에서 직접 reference로 삼은 값들은 이 측정에서 왔다.}
        \end{figure}

        \begin{figure}[H]
          \centering
          \includegraphics[width=0.96\linewidth]{figures_v22_latest/uuv_v22_latest_actual_mode_response.png}
          \caption{manual / stabilize / alt\_hold forward response 비교. current actual에서 mode별 response imbalance를 직접 확인할 수 있는 그림이다.}
        \end{figure}

        \section{정리}

        current를 발표용으로 줄이면 아래 세 문장만 남기면 된다.

        \begin{enumerate}[leftmargin=2em]
          \item current v2.2는 \textbf{MuJoCo ellipsoid fluid로 drag와 적분을 처리하고, Python runtime이 thrust model과 hydrostatics를 감싼 하이브리드 구조}다.
          \item thrust path는 simple polynomial model이고, hydrostatics는 current에서 \textbf{4-point buoyancy + restoring torque + runtime mass / inertia synthesis} 가 핵심이다.
          \item same current harness 기준으로 \textbf{4-point buoyancy가 single-point center보다 latest actual current reference에 훨씬 가깝고, release stability도 더 좋다}. 이것이 current 문서에서 4-point buoyancy를 따로 강조해야 하는 이유다.
        \end{enumerate}

        \end{document}
        """
    ).strip() + "\n"

    replacements = {
        "@@REPORT_DATE@@": metrics["measurement_date"],
        "@@MEASUREMENT_DATE@@": metrics["measurement_date"],
        "@@METRICS_JSON@@": code(str(METRICS_PATH.relative_to(ROOT))),
        "@@FOCUS_JSON@@": code(str(FOCUS_METRICS_PATH.relative_to(ROOT))),
        "@@CURRENT_JSON@@": code(str(CURRENT_MEAS_PATH.relative_to(ROOT))),
        "@@STEP_JSON@@": code(str(CURRENT_STEP_SUMMARY_PATH.relative_to(ROOT))),
        "@@SIM_PROFILES@@": code(str(SIM_PROFILES_PATH.relative_to(ROOT))),
        "@@THRUSTER_PARAMS@@": code(str(THRUSTER_PARAMS_PATH.relative_to(ROOT))),
        "@@PATH_REDUCTION@@": fmt_pct(float(reductions["path_m"]), 2),
        "@@HORIZ_REDUCTION@@": fmt_pct(float(reductions["horizontal_m"]), 2),
        "@@PITCH_REDUCTION@@": fmt_pct(float(reductions["pitch_deg"]), 2),
        "@@ROLL_REDUCTION@@": fmt_pct(float(reductions["roll_deg"]), 2),
        "@@THRUSTER_FORCE_MAX@@": fmt_trim(float(current_profile["thruster_force_max"]), 1),
        "@@VERT_GAIN@@": fmt_trim(float(current_profile["vertical_thruster_gain_scale"]), 2),
        "@@YAW_TORQUE_SCALE@@": fmt_trim(float(current_profile["yaw_torque_scale"]), 2),
        "@@BUOYANCY_SCALE@@": fmt_trim(float(current_profile["buoyancy_scale"]), 3),
        "@@COB_TORQUE_SCALE@@": fmt_trim(float(current_profile["cob_torque_scale"]), 2),
        "@@DEADZONE@@": fmt_trim(float(thr_global["deadzone"]), 3),
        "@@TAU_UP@@": fmt_trim(float(thr_global["tau_up"]), 3),
        "@@TAU_DOWN@@": fmt_trim(float(thr_global["tau_down"]), 3),
        "@@GAIN_SCALE@@": fmt_trim(float(thr_global["gain_scale_all"]), 2),
        "@@REVERSE_ASYM@@": fmt_trim(float(thr_global["reverse_asymmetry"]), 2),
        "@@COMMAND_LIMIT@@": fmt_trim(float(thr_global["command_limit"]), 2),
        "@@FORWARD_POLY@@": r"$" + vec(thr_global["forward_poly"], 3) + "$",
        "@@REVERSE_POLY@@": r"$" + vec(thr_global["reverse_poly"], 3) + "$",
        "@@CURRENT_COM@@": vec_math(current_mass["com_m"], 3),
        "@@CURRENT_COB@@": vec_math(current_mass["cob_m"], 3),
        "@@CURRENT_INERTIA_SCALED@@": vec_sci_math(current_mass["inertia_scaled_kgm2"], 3),
        "@@PROXY_VOLUME@@": fmt(float(scene["proxy_volume_total_m3"]), 4),
        "@@NEUTRAL_VOLUME@@": fmt(float(scene["neutral_volume_m3"]), 4),
        "@@SCENE_ROWS@@": scene_rows,
        "@@COMPARE_FORWARD@@": fmt_trim(float(comparison_cmd["forward_cmd_norm"]), 3),
        "@@COMPARE_HEAVE@@": fmt_trim(float(comparison_cmd["heave_cmd_norm"]), 3),
        "@@CENTER_SURGE@@": fmt(float(center_focus["forward_step"]["peak_surge_mps"]), 3),
        "@@FOUR_SURGE@@": fmt(float(current_focus["forward_step"]["peak_surge_mps"]), 3),
        "@@REF_SURGE@@": fmt(float(actual_ref["forward_peak_surge_mps"]), 3),
        "@@CENTER_PITCH@@": fmt(float(center_focus["forward_step"]["peak_pitch_deg"]), 3),
        "@@FOUR_PITCH@@": fmt(float(current_focus["forward_step"]["peak_pitch_deg"]), 3),
        "@@REF_PITCH@@": fmt(float(actual_ref["forward_peak_pitch_deg"]), 3),
        "@@CENTER_ROLL@@": fmt(float(center_focus["forward_step"]["peak_roll_deg"]), 3),
        "@@FOUR_ROLL@@": fmt(float(current_focus["forward_step"]["peak_roll_deg"]), 3),
        "@@REF_ROLL@@": fmt(float(actual_ref["forward_peak_roll_deg"]), 3),
        "@@CENTER_HEAVE@@": fmt(float(center_focus["heave_step"]["depth_delta_mm"]), 2),
        "@@FOUR_HEAVE@@": fmt(float(current_focus["heave_step"]["depth_delta_mm"]), 2),
        "@@REF_HEAVE@@": fmt(float(actual_ref["heave_depth_delta_mm"]), 2),
        "@@CENTER_RMS@@": fmt(float(center_focus["attitude_release"]["rms_angle_norm_deg"]), 2),
        "@@FOUR_RMS@@": fmt(float(current_focus["attitude_release"]["rms_angle_norm_deg"]), 2),
        "@@CENTER_SETTLE@@": fmt(float(center_focus["attitude_release"]["settling_time_s"]), 3),
        "@@FOUR_SETTLE@@": fmt(float(current_focus["attitude_release"]["settling_time_s"]), 3),
        "@@CENTER_GAP@@": fmt(float(center_focus["actual_gap_index_pct"]), 2),
        "@@FOUR_GAP@@": fmt(float(current_focus["actual_gap_index_pct"]), 2),
        "@@CURRENT_PATH@@": fmt(float(current_avg["path_m"]), 3),
        "@@CURRENT_HORIZ@@": fmt(float(current_avg["horizontal_m"]), 4),
        "@@CURRENT_PITCH@@": fmt(float(current_avg["pitch_deg"]), 4),
        "@@CURRENT_ROLL@@": fmt(float(current_avg["roll_deg"]), 4),
        "@@CURRENT_MODE_ROWS@@": mode_rows,
        "@@MANUAL_FORWARD@@": fmt(float(manual_forward_max), 3),
        "@@STAB_FORWARD@@": fmt(float(stabilize_forward_max), 3),
        "@@STAB_PITCH@@": fmt(float(stabilize_pitch_peak_deg), 2),
        "@@STAB_ROLL@@": fmt(float(stabilize_roll_peak_deg), 2),
        "@@MANUAL_YAW@@": fmt(float(manual_yaw_peak), 3),
        "@@MANUAL_HEAVE_MM@@": fmt(float(manual_heave_delta_mm), 2),
        "@@ALT_HEAVE_MM@@": fmt(float(alt_hold_heave_delta_mm), 2),
    }

    tex = template
    for key, value in replacements.items():
        tex = tex.replace(key, value)
    return tex


def main() -> None:
    TEX_PATH.write_text(build_tex())


if __name__ == "__main__":
    main()
