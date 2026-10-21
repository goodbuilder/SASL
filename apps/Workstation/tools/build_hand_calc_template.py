#!/usr/bin/env python3
"""Build the independent hand-calculation workbook for stereo triangulation.

The generated ``docs/validation/hand_calc_triangulation.xlsx`` contains only
cell formulas (no Python at evaluation time). It re-derives Forward/Right/Down/
Range from the same inputs the Workstation measurement workbook stores in
``03_Calibration`` and ``91_Exact``:

* K (fx, fy, cx, cy) and D1..D8 per side        -> ``left.K11`` ... ``right.D8``
* detector pixels                               -> ``left.u``, ``left.v``, ``right.u``, ``right.v``
* OpenCV stereo extrinsics  X_right = R X_left + T  -> ``R11``..``R33``, ``T_R``, ``T_D``, ``T_F`` (metres)

Frame contract (same as ``workstation_core/triangulation.py``):
  inputs   : optical RDF (right, down, forward) of each camera
  kernel   : left-camera RDF; right origin in left = -R^T T; right ray in left = R^T ray_r
  output   : baseline-center FRD (forward, right, down); Right axis = unit vector
             from left optical centre to right optical centre; origin = midpoint.

Run with the canonical interpreter so the example row can be checked against
the live code path::

    PYTHONPATH=apps:.:common/python .venv/bin/python apps/Workstation/tools/build_hand_calc_template.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import xlsxwriter

APP_ROOT = Path(__file__).resolve().parents[1]
OUTPUT = APP_ROOT / "docs" / "validation" / "hand_calc_triangulation.xlsx"
UNDISTORT_ITERATIONS = 5  # OpenCV cv2.undistortPoints default iteration count

IN, CALC, CMP = "입력", "계산", "비교"


def q(sheet: str, cell: str) -> str:
    return f"'{sheet}'!{cell}"


# ---------------------------------------------------------------- example case
def example_case() -> dict[str, float]:
    """Known-answer example: target at left-RDF (0.3+5, 1, 30) m, right camera
    toed in 2 deg about optical down, baseline 0.6 m, mild plumb-bob distortion."""
    import cv2
    import numpy as np

    fx, fy, cx, cy = 800.0, 800.0, 320.0, 256.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], float)
    D_left = np.array([-0.30, 0.10, 0.001, -0.002, 0.0])
    D_right = np.array([-0.28, 0.09, -0.001, 0.0015, 0.0])
    baseline = 0.6
    theta = math.radians(2.0)
    c, s = math.cos(theta), math.sin(theta)
    R_rl = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])  # right -> left (rotation about RDF down)
    o_r = np.array([baseline, 0.0, 0.0])  # right optical centre in left RDF
    R_lr = R_rl.T  # OpenCV convention X_right = R_lr X_left + T
    T = -R_lr @ o_r
    P_left = np.array([baseline / 2 + 5.0, 1.0, 30.0])
    uv_l = cv2.projectPoints(P_left.reshape(1, 1, 3), np.zeros(3), np.zeros(3), K, D_left)[0][0, 0]
    rvec, _ = cv2.Rodrigues(R_lr)
    uv_r = cv2.projectPoints(P_left.reshape(1, 1, 3), rvec, T, K, D_right)[0][0, 0]
    values = {
        "L_fx": fx, "L_fy": fy, "L_cx": cx, "L_cy": cy,
        "R_fx": fx, "R_fy": fy, "R_cx": cx, "R_cy": cy,
        "L_u": float(uv_l[0]), "L_v": float(uv_l[1]), "R_u": float(uv_r[0]), "R_v": float(uv_r[1]),
        "T_R": float(T[0]), "T_D": float(T[1]), "T_F": float(T[2]),
    }
    for i in range(8):
        values[f"L_D{i + 1}"] = float(D_left[i]) if i < len(D_left) else 0.0
        values[f"R_D{i + 1}"] = float(D_right[i]) if i < len(D_right) else 0.0
    for r in range(3):
        for col in range(3):
            values[f"R{r + 1}{col + 1}"] = float(R_lr[r, col])
    values["expected_geometry"] = (30.0, 5.0, 1.0, math.sqrt(30**2 + 5**2 + 1**2))
    return values


def code_result(values: dict[str, float]) -> tuple[float, float, float, float] | None:
    """Run the live Workstation reproduce path on the example inputs."""
    try:
        from Workstation.workstation_core.measurement_reproduce import reproduce_pair_geometry
    except ImportError:
        return None
    out, _reason = reproduce_pair_geometry(
        left_camera_matrix=[[values["L_fx"], 0, values["L_cx"]], [0, values["L_fy"], values["L_cy"]], [0, 0, 1]],
        left_distortion=[values[f"L_D{i}"] for i in range(1, 6)],
        right_camera_matrix=[[values["R_fx"], 0, values["R_cx"]], [0, values["R_fy"], values["R_cy"]], [0, 0, 1]],
        right_distortion=[values[f"R_D{i}"] for i in range(1, 6)],
        left_uv=(values["L_u"], values["L_v"]),
        right_uv=(values["R_u"], values["R_v"]),
        rotation_right_from_left=[[values[f"R{r}{c}"] for c in (1, 2, 3)] for r in (1, 2, 3)],
        translation_right_from_left_m=[values["T_R"], values["T_D"], values["T_F"]],
    )
    return out


# ---------------------------------------------------------------- workbook
def build(output: Path = OUTPUT) -> Path:
    ex = example_case()
    code = code_result(ex)
    output.parent.mkdir(parents=True, exist_ok=True)
    wb = xlsxwriter.Workbook(str(output))
    f_title = wb.add_format({"bold": True, "font_size": 14})
    f_head = wb.add_format({"bold": True, "bg_color": "#DDE5F0", "border": 1})
    f_in = wb.add_format({"bg_color": "#FFF7CC", "border": 1, "num_format": "0.000000000"})
    f_num = wb.add_format({"num_format": "0.000000000", "border": 1})
    f_out = wb.add_format({"bold": True, "bg_color": "#E2F0D9", "border": 1, "num_format": "0.0000"})
    f_lbl = wb.add_format({"border": 1})
    f_sec = wb.add_format({"bold": True, "bg_color": "#F2F2F2"})
    f_wrap = wb.add_format({"text_wrap": True, "valign": "top"})

    # ---------------------------------------------------------- 설명
    ws = wb.add_worksheet("설명")
    ws.set_column(0, 0, 110)
    lines = [
        ("스테레오 삼각측량 손 계산 시트 (완전판)", f_title),
        ("", None),
        ("목적: Workstation 프로그램과 독립적으로, 같은 입력에서 Forward / Right / Down / Range 를 셀 수식만으로 다시 계산한다.", f_wrap),
        ("이 파일에는 파이썬 코드가 없다. 노란 칸(입력 시트)을 바꾸면 계산·비교 시트가 즉시 따라 바뀐다.", f_wrap),
        ("", None),
        ("사용법", f_sec),
        ("1. 프로그램이 저장한 측정 워크북(.xlsx)을 연다. 91_Exact 시트에서 pair_id / method_id 가 같은 행들을 찾는다.", f_wrap),
        ("2. 입력 시트의 '워크북 출처' 열에 적힌 field 이름의 value 를 노란 칸에 복사한다. (03_Calibration 시트의 K11.., D1.. 도 같은 값이다.)", f_wrap),
        ("3. 91_Exact 의 Forward / Right / Down / Range 값을 입력 시트 맨 아래 '프로그램 결과' 칸에 복사한다.", f_wrap),
        ("4. 비교 시트에서 차이(m)를 본다. 같은 입력이면 1e-6 m 이내로 같아야 한다.", f_wrap),
        ("", None),
        ("좌표 계약", f_sec),
        ("입력 픽셀 u,v : 왼쪽/오른쪽 검출기 픽셀 (원점 좌상단, u 오른쪽, v 아래).", f_wrap),
        ("K, D : OpenCV 핀홀 + plumb_bob/rational 왜곡 (k1,k2,p1,p2,k3,k4,k5,k6). 비어 있으면 0. 스큐(K12)는 OpenCV undistortPoints 와 같이 무시한다.", f_wrap),
        ("R, T : OpenCV 스테레오 보정 규약  X_right = R · X_left + T  (광학 RDF, T 는 m). 워크북의 R11..R33, T_R/T_D/T_F 가 이것이다.", f_wrap),
        ("계산 내부 프레임 : 왼쪽 카메라 광학 RDF (right, down, forward). 오른쪽 카메라 원점 = -Rᵀ T, 오른쪽 광선 = Rᵀ · ray_r.", f_wrap),
        ("출력 프레임 : 기선 중심 FRD (forward, right, down). Right 축 = 왼쪽 광학중심 → 오른쪽 광학중심 단위벡터, Forward = 두 광축 평균을 Right 에 직교화, Down = Forward × Right, 원점 = 두 중심의 중점. Range 는 중점에서의 거리.", f_wrap),
        ("주의 : Right 축은 카메라가 아니라 기선 방향으로 정의된다. 기선이 가로인 정상 장착에서는 카메라 오른쪽과 같다.", f_wrap),
        ("", None),
        ("계산 단계 (계산 시트와 같은 순서)", f_sec),
        ("① 정규화  x0=(u-cx)/fx, y0=(v-cy)/fy", f_wrap),
        (f"② 왜곡 제거  OpenCV undistortPoints 와 같은 고정점 반복 {UNDISTORT_ITERATIONS}회: x=(x0-Δx)·icdist, icdist=(1+k4r²+k5r⁴+k6r⁶)/(1+k1r²+k2r⁴+k3r⁶), Δx=2p1xy+p2(r²+2x²), Δy=p1(r²+2y²)+2p2xy", f_wrap),
        ("③ 단위 광선  ray = (x, y, 1)/‖(x, y, 1)‖", f_wrap),
        ("④ 외부 파라미터 뒤집기  R_rl = Rᵀ,  C_r = -Rᵀ T  (오른쪽 원점을 왼쪽 프레임으로)", f_wrap),
        ("⑤ 오른쪽 광선을 왼쪽 프레임으로  d2 = R_rl · ray_r,  오른쪽 광축 f_r = R_rl · (0,0,1)", f_wrap),
        ("⑥ 두 광선 최근접점  dot=d1·d2, cross=d1×d2, n=‖cross‖, Δ=O_l-O_r=-C_r, lp=d1·Δ, rp=d2·Δ, s_l=(dot·rp-lp)/n², s_r=(rp-dot·lp)/n², P_l=s_l d1, P_r=C_r+s_r d2, P=(P_l+P_r)/2", f_wrap),
        ("⑦ 기선 중심 FRD  e_R=C_r/‖C_r‖, fc=(f_l+f_r)/‖·‖, e_F=(fc-(fc·e_R)e_R)/‖·‖, e_D=e_F×e_R, O=C_r/2, Forward=(P-O)·e_F, Right=(P-O)·e_R, Down=(P-O)·e_D, Range=‖P-O‖", f_wrap),
        ("⑧ 참고 검산  거리 ≈ fx·B/시차  (회전·왜곡이 없을 때만 정확)", f_wrap),
        ("", None),
        ("기본으로 들어 있는 예제 입력은 '정답을 아는' 가상 장면이다: 기선 0.6 m, 오른쪽 카메라 2° 토인, 표적은 기선 중점 기준 Forward 30 / Right 5 / Down 1 m. 비교 시트에서 수식 결과·프로그램 결과·기하 정답 셋이 모두 일치해야 한다.", f_wrap),
        ("생성 스크립트: apps/Workstation/tools/build_hand_calc_template.py", f_wrap),
    ]
    for i, (text, fmt) in enumerate(lines):
        ws.write(i, 0, text, fmt) if fmt else ws.write(i, 0, text)

    # ---------------------------------------------------------- 입력
    wi = wb.add_worksheet(IN)
    wi.set_column(0, 0, 26); wi.set_column(1, 1, 22); wi.set_column(2, 2, 14); wi.set_column(3, 3, 24)
    wi.write(0, 0, "입력 (노란 칸만 수정)", f_title)
    for c, h in enumerate(("항목", "값", "단위", "워크북 출처 (91_Exact field)")):
        wi.write(2, c, h, f_head)
    addr: dict[str, str] = {}
    row = 3

    def add_input(key: str, label: str, unit: str, source: str, value: float | None):
        nonlocal row
        wi.write(row, 0, label, f_lbl)
        if value is None:
            wi.write_blank(row, 1, None, f_in)
        else:
            wi.write_number(row, 1, value, f_in)
        wi.write(row, 2, unit, f_lbl); wi.write(row, 3, source, f_lbl)
        addr[key] = f"B{row + 1}"
        row += 1

    def section(title: str):
        nonlocal row
        row += 1
        wi.write(row, 0, title, f_sec); row += 1

    for side, name in (("L", "왼쪽"), ("R", "오른쪽")):
        src = "left" if side == "L" else "right"
        section(f"{name} 카메라 내부 파라미터 (03_Calibration 또는 91_Exact)")
        add_input(f"{side}_fx", f"{name} fx", "px", f"{src}.K11", ex[f"{side}_fx"])
        add_input(f"{side}_fy", f"{name} fy", "px", f"{src}.K22", ex[f"{side}_fy"])
        add_input(f"{side}_cx", f"{name} cx", "px", f"{src}.K13", ex[f"{side}_cx"])
        add_input(f"{side}_cy", f"{name} cy", "px", f"{src}.K23", ex[f"{side}_cy"])
        for i, dn in enumerate(("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"), start=1):
            add_input(f"{side}_D{i}", f"{name} D{i} ({dn})", "-", f"{src}.D{i} (없으면 0)", ex[f"{side}_D{i}"])
    section("검출 픽셀 (91_Exact)")
    add_input("L_u", "왼쪽 u", "px", "left.u", ex["L_u"])
    add_input("L_v", "왼쪽 v", "px", "left.v", ex["L_v"])
    add_input("R_u", "오른쪽 u", "px", "right.u", ex["R_u"])
    add_input("R_v", "오른쪽 v", "px", "right.v", ex["R_v"])
    section("스테레오 외부 파라미터  X_right = R·X_left + T  (91_Exact)")
    for r in (1, 2, 3):
        for c in (1, 2, 3):
            add_input(f"R{r}{c}", f"R{r}{c}", "-", f"R{r}{c}", ex[f"R{r}{c}"])
    add_input("T_R", "T_R (right 성분)", "m", "T_R", ex["T_R"])
    add_input("T_D", "T_D (down 성분)", "m", "T_D", ex["T_D"])
    add_input("T_F", "T_F (forward 성분)", "m", "T_F", ex["T_F"])
    section("프로그램 결과 (91_Exact 의 Forward/Right/Down/Range 를 붙여 넣기)")
    for key, label in (("P_F", "Forward"), ("P_R", "Right"), ("P_D", "Down"), ("P_range", "Range")):
        add_input(key, f"프로그램 {label}", "m", label, None if code is None else code[("P_F", "P_R", "P_D", "P_range").index(key)])

    def I(key: str) -> str:  # noqa: E743 - short alias for formulas
        return q(IN, addr[key])

    def D(side: str, i: int) -> str:  # distortion coefficient, blank -> 0
        return f"N({I(f'{side}_D{i}')})"

    # ---------------------------------------------------------- 계산
    wc = wb.add_worksheet(CALC)
    wc.set_column(0, 0, 30); wc.set_column(1, 8, 16)
    wc.write(0, 0, "계산 (수정하지 않음. 모든 칸이 수식)", f_title)
    crow = 2
    cell: dict[str, str] = {}

    def put(key: str, label: str, formula: str, fmt=f_num, col: int = 1) -> str:
        """Write label + formula; return absolute address for later formulas."""
        nonlocal crow
        wc.write(crow, 0, label, f_lbl)
        wc.write_formula(crow, col, "=" + formula, fmt)
        a = f"{chr(ord('A') + col)}{crow + 1}"
        cell[key] = q(CALC, a)
        crow += 1
        return cell[key]

    def hdr(text: str):
        nonlocal crow
        crow += 1
        wc.write(crow, 0, text, f_sec); crow += 1

    # ① ② ③ per side
    for side, name in (("L", "왼쪽"), ("R", "오른쪽")):
        hdr(f"① {name} 정규화 좌표")
        x0 = put(f"{side}_x0", f"{name} x0 = (u-cx)/fx", f"({I(f'{side}_u')}-{I(f'{side}_cx')})/{I(f'{side}_fx')}")
        y0 = put(f"{side}_y0", f"{name} y0 = (v-cy)/fy", f"({I(f'{side}_v')}-{I(f'{side}_cy')})/{I(f'{side}_fy')}")
        hdr(f"② {name} 왜곡 제거 (고정점 반복 {UNDISTORT_ITERATIONS}회)   열: x | y | r² | icdist | Δx | Δy")
        for c, h in enumerate(("반복", "x", "y", "r²", "icdist", "Δx", "Δy")):
            wc.write(crow, c, h, f_head)
        crow += 1
        x_prev, y_prev = x0, y0
        for it in range(UNDISTORT_ITERATIONS + 1):
            wc.write(crow, 0, f"{name} 반복 {it}" if it else f"{name} 시작값", f_lbl)
            rr = crow + 1
            xa, ya, r2a, ica, dxa, dya = (f"{col}{rr}" for col in "BCDEFG")
            if it == 0:
                wc.write_formula(crow, 1, f"={x0}", f_num); wc.write_formula(crow, 2, f"={y0}", f_num)
            else:
                wc.write_formula(crow, 1, f"=({x0}-{dxa})*{ica}", f_num)
                wc.write_formula(crow, 2, f"=({y0}-{dya})*{ica}", f_num)
            # r², icdist, Δ use the previous iterate (x_prev, y_prev)
            wc.write_formula(crow, 3, f"={x_prev}^2+{y_prev}^2", f_num)
            wc.write_formula(crow, 4, f"=(1+{D(side,6)}*{r2a}+{D(side,7)}*{r2a}^2+{D(side,8)}*{r2a}^3)/(1+{D(side,1)}*{r2a}+{D(side,2)}*{r2a}^2+{D(side,5)}*{r2a}^3)", f_num)
            wc.write_formula(crow, 5, f"=2*{D(side,3)}*{x_prev}*{y_prev}+{D(side,4)}*({r2a}+2*{x_prev}^2)", f_num)
            wc.write_formula(crow, 6, f"={D(side,3)}*({r2a}+2*{y_prev}^2)+2*{D(side,4)}*{x_prev}*{y_prev}", f_num)
            crow += 1
            # the NEXT row's r²/Δ must use THIS row's x,y, but the x,y of row `it` are computed
            # from Δ/icdist of row `it` which use x_prev. So chain: row it uses prev; then prev := row it.
            x_prev, y_prev = q(CALC, xa), q(CALC, ya)
        # NOTE: OpenCV evaluates r², icdist, Δ from the current iterate then updates x,y.
        # Row `it` holds r²/icdist/Δ from iterate it-1 and the updated x,y → identical recursion.
        cell[f"{side}_xu"], cell[f"{side}_yu"] = x_prev, y_prev
        hdr(f"③ {name} 단위 광선 (왼쪽 프레임 기준은 ⑤에서)")
        nrm = put(f"{side}_rn", f"{name} ‖(x,y,1)‖", f"SQRT({x_prev}^2+{y_prev}^2+1)")
        put(f"{side}_ray_R", f"{name} ray R", f"{x_prev}/{nrm}")
        put(f"{side}_ray_D", f"{name} ray D", f"{y_prev}/{nrm}")
        put(f"{side}_ray_F", f"{name} ray F", f"1/{nrm}")

    # ④ invert extrinsics
    hdr("④ 외부 파라미터 뒤집기  R_rl = Rᵀ,  C_r = -Rᵀ·T")
    for r in (1, 2, 3):
        for c in (1, 2, 3):
            put(f"Rt{r}{c}", f"R_rl[{r}][{c}] = R{c}{r}", f"{I(f'R{c}{r}')}")
    for r, ax in ((1, "R"), (2, "D"), (3, "F")):
        put(f"C_{ax}", f"C_r {ax} (오른쪽 원점, 왼쪽 프레임)", f"-({cell[f'Rt{r}1']}*{I('T_R')}+{cell[f'Rt{r}2']}*{I('T_D')}+{cell[f'Rt{r}3']}*{I('T_F')})")
    put("B", "기선 길이 B = ‖C_r‖", f"SQRT({cell['C_R']}^2+{cell['C_D']}^2+{cell['C_F']}^2)")

    # ⑤ rotate right ray and right forward into left frame
    hdr("⑤ 오른쪽 광선·광축을 왼쪽 프레임으로  d2 = R_rl·ray_r,  f_r = R_rl·(0,0,1)")
    for r, ax in ((1, "R"), (2, "D"), (3, "F")):
        put(f"d2raw_{ax}", f"d2 raw {ax}", f"{cell[f'Rt{r}1']}*{cell['R_ray_R']}+{cell[f'Rt{r}2']}*{cell['R_ray_D']}+{cell[f'Rt{r}3']}*{cell['R_ray_F']}")
    n2 = put("d2n", "‖d2 raw‖", f"SQRT({cell['d2raw_R']}^2+{cell['d2raw_D']}^2+{cell['d2raw_F']}^2)")
    for ax in ("R", "D", "F"):
        put(f"d2_{ax}", f"d2 {ax} (단위)", f"{cell[f'd2raw_{ax}']}/{n2}")
    for r, ax in ((1, "R"), (2, "D"), (3, "F")):
        put(f"fr_{ax}", f"f_r {ax} (오른쪽 광축)", f"{cell[f'Rt{r}3']}")
    for ax in ("R", "D", "F"):
        cell[f"d1_{ax}"] = cell[f"L_ray_{ax}"]

    # ⑥ closest-ray
    hdr("⑥ 두 광선 최근접점 (왼쪽 프레임, O_l = 0)")
    d1, d2 = (lambda a: cell[f"d1_{a}"]), (lambda a: cell[f"d2_{a}"])
    dot = put("dot", "dot = d1·d2", f"{d1('R')}*{d2('R')}+{d1('D')}*{d2('D')}+{d1('F')}*{d2('F')}")
    put("cr_R", "cross R", f"{d1('D')}*{d2('F')}-{d1('F')}*{d2('D')}")
    put("cr_D", "cross D", f"{d1('F')}*{d2('R')}-{d1('R')}*{d2('F')}")
    put("cr_F", "cross F", f"{d1('R')}*{d2('D')}-{d1('D')}*{d2('R')}")
    n = put("crn", "n = ‖cross‖", f"SQRT({cell['cr_R']}^2+{cell['cr_D']}^2+{cell['cr_F']}^2)")
    for ax in ("R", "D", "F"):
        put(f"dl_{ax}", f"Δ {ax} = O_l - O_r = -C_r", f"-{cell[f'C_{ax}']}")
    lp = put("lp", "lp = d1·Δ", f"{d1('R')}*{cell['dl_R']}+{d1('D')}*{cell['dl_D']}+{d1('F')}*{cell['dl_F']}")
    rp = put("rp", "rp = d2·Δ", f"{d2('R')}*{cell['dl_R']}+{d2('D')}*{cell['dl_D']}+{d2('F')}*{cell['dl_F']}")
    sl = put("sl", "s_l 왼쪽 깊이 = (dot·rp - lp)/n²", f"({dot}*{rp}-{lp})/{n}^2")
    sr = put("sr", "s_r 오른쪽 깊이 = (rp - dot·lp)/n²", f"({rp}-{dot}*{lp})/{n}^2")
    for ax in ("R", "D", "F"):
        put(f"Pl_{ax}", f"P_l {ax} = s_l·d1", f"{sl}*{d1(ax)}")
    for ax in ("R", "D", "F"):
        put(f"Pr_{ax}", f"P_r {ax} = C_r + s_r·d2", f"{cell[f'C_{ax}']}+{sr}*{d2(ax)}")
    for ax in ("R", "D", "F"):
        put(f"P_{ax}", f"P {ax} = (P_l+P_r)/2  [왼쪽 RDF]", f"({cell[f'Pl_{ax}']}+{cell[f'Pr_{ax}']})/2")
    put("gap", "광선 간격 ‖P_l - P_r‖", f"SQRT(({cell['Pl_R']}-{cell['Pr_R']})^2+({cell['Pl_D']}-{cell['Pr_D']})^2+({cell['Pl_F']}-{cell['Pr_F']})^2)")
    put("angle", "광선 사이각 (deg) = atan2(n, dot)", f"DEGREES(ATAN2({dot},{n}))")

    # ⑦ baseline-center FRD
    hdr("⑦ 기선 중심 FRD 변환")
    for ax in ("R", "D", "F"):
        put(f"eR_{ax}", f"e_R {ax} = C_r/B", f"{cell[f'C_{ax}']}/{cell['B']}")
    fl = {"R": "0", "D": "0", "F": "1"}
    for ax in ("R", "D", "F"):
        put(f"fc_{ax}", f"fc {ax} = f_l + f_r", f"{fl[ax]}+{cell[f'fr_{ax}']}")
    fcn = put("fcn", "‖fc‖", f"SQRT({cell['fc_R']}^2+{cell['fc_D']}^2+{cell['fc_F']}^2)")
    fdot = put("fdot", "(fc/‖fc‖)·e_R", f"({cell['fc_R']}*{cell['eR_R']}+{cell['fc_D']}*{cell['eR_D']}+{cell['fc_F']}*{cell['eR_F']})/{fcn}")
    for ax in ("R", "D", "F"):
        put(f"fo_{ax}", f"f⊥ {ax} = fc/‖fc‖ - (·)e_R", f"{cell[f'fc_{ax}']}/{fcn}-{fdot}*{cell[f'eR_{ax}']}")
    fon = put("fon", "‖f⊥‖", f"SQRT({cell['fo_R']}^2+{cell['fo_D']}^2+{cell['fo_F']}^2)")
    for ax in ("R", "D", "F"):
        put(f"eF_{ax}", f"e_F {ax}", f"{cell[f'fo_{ax}']}/{fon}")
    eF, eR = (lambda a: cell[f"eF_{a}"]), (lambda a: cell[f"eR_{a}"])
    put("eD_R", "e_D R = (e_F × e_R) R", f"{eF('D')}*{eR('F')}-{eF('F')}*{eR('D')}")
    put("eD_D", "e_D D", f"{eF('F')}*{eR('R')}-{eF('R')}*{eR('F')}")
    put("eD_F", "e_D F", f"{eF('R')}*{eR('D')}-{eF('D')}*{eR('R')}")
    for ax in ("R", "D", "F"):
        put(f"O_{ax}", f"원점 O {ax} = C_r/2", f"{cell[f'C_{ax}']}/2")
    for ax in ("R", "D", "F"):
        put(f"PO_{ax}", f"(P - O) {ax}", f"{cell[f'P_{ax}']}-{cell[f'O_{ax}']}")
    PO = lambda a: cell[f"PO_{a}"]  # noqa: E731
    hdr("결과 (기선 중심 FRD, m)")
    put("F", "Forward = (P-O)·e_F", f"{PO('R')}*{eF('R')}+{PO('D')}*{eF('D')}+{PO('F')}*{eF('F')}", f_out)
    put("Rt", "Right = (P-O)·e_R", f"{PO('R')}*{eR('R')}+{PO('D')}*{eR('D')}+{PO('F')}*{eR('F')}", f_out)
    put("Dn", "Down = (P-O)·e_D", f"{PO('R')}*{cell['eD_R']}+{PO('D')}*{cell['eD_D']}+{PO('F')}*{cell['eD_F']}", f_out)
    put("Rg", "Range = ‖P-O‖", f"SQRT({PO('R')}^2+{PO('D')}^2+{PO('F')}^2)", f_out)
    hdr("⑧ 참고 검산 (두 카메라가 평행할 때만 유효)")
    put("disp", "시차 = 왼쪽 u - 오른쪽 u (px)", f"{I('L_u')}-{I('R_u')}")
    rot = put("rotdeg", "R 의 회전각 (deg) = acos((trace-1)/2)", f"DEGREES(ACOS(MAX(-1,MIN(1,({I('R11')}+{I('R22')}+{I('R33')}-1)/2))))")
    put("simple", "fx·B/시차 (m)  [회전각 > 0.1° 이면 무효]", f"IF({rot}>0.1,\"무효: 카메라 회전 \"&TEXT({rot},\"0.00\")&\"°\",{I('L_fx')}*{cell['B']}/{cell['disp']})", f_out)

    # ---------------------------------------------------------- 비교
    wm = wb.add_worksheet(CMP)
    wm.set_column(0, 0, 14); wm.set_column(1, 4, 20)
    wm.write(0, 0, "비교", f_title)
    for c, h in enumerate(("성분", "수식 결과 (이 파일)", "프로그램 결과 (입력 시트)", "차이 (m)", "기하 정답 (예제일 때만)")):
        wm.write(2, c, h, f_head)
    geo = ex["expected_geometry"]
    for i, (label, k, pk) in enumerate((("Forward", "F", "P_F"), ("Right", "Rt", "P_R"), ("Down", "Dn", "P_D"), ("Range", "Rg", "P_range"))):
        r = 3 + i
        wm.write(r, 0, label, f_lbl)
        wm.write_formula(r, 1, f"={cell[k]}", f_out)
        wm.write_formula(r, 2, f"={I(pk)}", f_num)
        wm.write_formula(r, 3, f"=IF(ISNUMBER(C{r + 1}),B{r + 1}-C{r + 1},\"\")", f_num)
        wm.write_number(r, 4, geo[i], f_num)
    wm.write(8, 0, "판정", f_lbl)
    wm.write_formula(8, 1, '=IF(COUNT(D4:D7)=0,"프로그램 결과 미입력",IF(MAX(ABS(D4),ABS(D5),ABS(D6),ABS(D7))<0.000001,"일치 (1e-6 m 이내)","불일치"))', f_out)
    wm.write(10, 0, "참고", f_lbl)
    wm.write(10, 1, "기하 정답 열은 기본 예제(토인 2°, 기선 0.6 m, 표적 F30/R5/D1)에만 해당. 실측 데이터를 넣으면 무시한다.")

    wb.close()
    return output


if __name__ == "__main__":
    path = build()
    print(path)
    sys.exit(0)
