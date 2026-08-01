#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pick8 — 방울토마토 수확 9분할 (DDS · 자체FK/IK · send_angles)
==============================================================
사용자 지정 9단계:
  1. base y·z 정렬    토마토와 같은 높이·좌우로 서고, x 로만 뒤에 대기. J6 는 안 돔(고정)
  2. J2·J3·J4만      바닥과 평행하게 수평 전진해 토마토에 닿음 (J1·J5·J6 고정)
  3. 그리퍼 잡기
  4. J2·J3·J4만      같은 경로로 수평 후퇴 = 1단계 자세로 복귀 (2단계의 정확한 역순)
  5. J1 + J5만     바구니 방향으로(오른쪽). J2·J3·J4 돌리기 전 최종 좌표. J6 는 절대 안 돔
  6. J2·J3·J4만    바구니 투하 위치로 (셋 중 필요한 만큼만)
  7. 그리퍼 풀기
  8. J2·J3·J4만    **6번 하기 전 자세(=5번이 끝난 자세)로 복귀** — 6번의 정확한 역동작
  9. 관측자세      (돌아도 문제 없는 자세)

■ 좌표계 — 1·2·4단계의 x,y,z 는 전부 **카메라(D435) 기준**이다 (사용자 사양 2026-07-22):
     x = RGB 화면 가로     y = RGB 화면 세로     z = 뎁스(카메라가 보는 앞뒤 거리)
   로봇 베이스 기준이 아니다. 카메라는 손목(joint6)에 달려 있어 팔과 함께 축이 돌아가므로,
   **잡는 순간의 손목 방향에서 축을 한 번 계산해** 1·2·4단계 내내 그대로 쓴다.
   그래야 ②와 ④가 같은 직선을 왕복한다.
   유일한 예외는 처짐 보정(SAG_Z) — 중력은 항상 수직이라 진짜 수직을 쓴다. 아래 설명 참조.
⛔ MoveIt 아님. send_coords 아님(뱀꼬임으로 폐기). URDF 기반 자체 FK + scipy IK → send_angles.
⛔ 3~7단계 사이에 그리퍼를 절대 열지 않는다 (과거 상승 중 떨어뜨린 사고).

전제:
  source ros_env_automato.sh        (DDS env)
  /tmp/tomato_base.txt              measure_tomato.py 또는 pick8.py measure 로 생성
  /tmp/frame_offset.npy             없으면 cp frame_offset.npy /tmp/

사용:
  python3 pick8.py measure       # 카메라로 토마토 검출 → /tmp/tomato_base.txt (로봇 안 움직임)
  python3 pick8.py plan          # 9단계 전부 IK로 계산만 (로봇 안 움직임) ← 실물 전에 필수
  python3 pick8.py teach-basket   # 지금 팔 위치를 '바구니 투하 자세'로 저장
  python3 pick8.py teach-observe  # 지금 팔 위치를 '8단계 관측(안전)자세'로 저장
  python3 pick8.py run           # 실제 9단계 실행 (단계마다 안 물어보고 자동 진행)
"""
import sys, os, json, math, time
import numpy as np
from scipy.optimize import least_squares
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

HERE = os.path.dirname(os.path.abspath(__file__))

# ── 팀원 속도 상수 (사용자 지정, 2026-07-22) ───────────────────────────
SPEED          = 30      # 관절 이동 속도 (pymycobot 1~100)
TIMEOUT        = 15      # 한 스텝 도달 대기 상한 (초)
GRIPPER_SPEED  = 50      # 그리퍼 속도
SETTLE         = 0.3     # 각 스텝 후 정착 대기 (초)
RETURN_BY_REVERSE = False  # 사이클 끝에 기록 경로를 역순으로 되짚어 복귀

TOL      = 3.0   # '정확히 도착' 기준(도)
TOL_HARD   = 8.0   # 여기까지는 통과로 인정. 서보 정밀도가 1~3°라 3°는 못 넘는 경우가 많다
STILL_MSGS = 2     # /joint_states(2Hz) 연속 2개가 같으면 정지 = 1초간 안 움직임
STILL_TOL  = 0.8   # 이 이하 변화는 '안 움직임'으로 본다(도).
                   # 0.3°로 두면 서보 미세 떨림에 카운터가 계속 리셋돼
                   # 최대대기(15초)를 다 채운다 — 4단계가 유난히 느렸던 원인(2026-07-22)
GRIP_WAIT  = 3.0   # 그리퍼 개폐 대기(초) — 다 닫히기 전에 후퇴하면 놓친다
# 사용자 사양: 한 단계 = 명령 1회. 중간에 쪼개지 않는다 (2026-07-22 지시).
N_STAGES = 9      # 사용자 사양: 총 9분할 = 명령 9회
ONE_MOVE_PER_STAGE = True
MAX_SWING = 999.0 if ONE_MOVE_PER_STAGE else 20.0
# ⚠️ 쪼개기를 끄면 관절 변화가 클 때 서보가 스톨할 수 있다(오늘 아침 실증).
#    스톨이 나면 ONE_MOVE_PER_STAGE=False 로 되돌릴 것.

# ── 캘리브 상수 (③TCP, 확정값) ─────────────────────────────────────────
TCP = np.array([-3.2, -10.3, 109.2]) / 1000.0      # joint6_flange → 손끝 (m)
# ⚠️ J6는 사양상 ±177°지만 **그리퍼 케이블 때문에 ±140° 부근에서 물리적으로 막힌다**
#    (2026-07-22 실증: -160° 명령 → -140~-147°에서 멈추고 도달 실패).
#    평행 집게는 180° 돌려도 같은 파지라, 한계를 좁혀도 잡는 데는 지장 없다.
J6_SAFE = 130.0
JLIM = [(-165,165),(-132,132),(-147,147),(-142,142),(-162,162),(-J6_SAFE,J6_SAFE)]
PVIEW = [6.24, 15.73, -77.6, 16.08, 2.9, 3.6]      # dg_control_node 의 기본 관찰자세

# 중력 처짐 보정 — 팔이 자기 무게로 명령각보다 더 숙여져 손끝이 아래로 내려간다.
# 서보의 정상상태 오차라 기다려도 안 없어진다.
# 2026-07-22 실측: 토마토 중심보다 z가 정확히 17mm 아래를 집었음(반지름 17mm = 밑동만 스침).
# → 목표 z를 미리 이만큼 올려서 명령한다. 계산은 전부 미리 끝난다(실행 중 보정 없음).
# 카메라 기준으로 말하면 **-y 방향(화면에서 위쪽)** 으로 올리는 보정이다.
# ⚠️ 카메라 z(뎁스=앞뒤)가 아니다. 다만 실제 코드는 카메라 -y 대신 **진짜 수직**을 쓴다:
#    중력은 언제나 수직인데 카메라 -y 는 손목이 고개를 숙인 만큼(실측 23°) 기울어 있고,
#    그 기울기가 자세마다 변하기 때문. 실측도 base z로 17mm 낮게 나왔다.
SAG_Z = 28.0       # mm (화면 위쪽 = 바닥→천장 방향)
# 2026-07-22 2회차 실측: +17 을 넣었더니 중심보다 8.7mm 위로 지나침
#   (명령 z 293.3 → 실제 285.0, 즉 실제 처짐은 8.3mm) → 17 에서 9 로 낮춤.
# 2026-07-23: 영상 확인 결과 아직도 살짝 아래를 딴다(사용자) + 집게축 tool-y 로 바꿔
#   손목 자세(J6)가 달라져 처짐도 달라짐 → 22 에서 28 로 올려 더 위를 겨냥.

# 뎁스축으로 얼마나 더 깊이 밀어넣을지(mm).
# 2026-07-22 2회차 실측: 손끝이 토마토 중심에 **뎁스로 6.7mm 못 미치고** 멈췄다.
# 그러면 그리퍼 손가락이 토마토를 감싸지 못하고 앞면만 스친다.
# 서보 정상상태 오차라 명령을 그만큼 더 깊이 줘야 실제로 중심에 닿는다.
DEPTH_PUSH = -13.0

# 바구니 투하 좌표도 팔이 뻗은 상태라 중력으로 처진다 → 목표 z를 위로 올려 명령.
# (2026-07-23 사용자: 바구니에 놓을 때 처져서 조금 더 위로)
BASKET_SAG = 20.0   # mm (바구니 목표를 위로)

# 그리퍼 '집게 벌어지는 축'. 0=tool-x, 1=tool-y.
# 2026-07-23 로봇 실물 판별(사용자가 수평이라 확인한 홈 자세): 바닥과 평행한 축은 tool-x(집게축).
#   (아까 영상만 보고 tool-y 로 바꿨던 건 원근 착시 때문에 틀린 판단이었음 → 0 으로 되돌림)
OPEN_AXIS = 0
LEVEL_GRIPPER = True

# 2026-07-23 사용자 확정: J6 는 '안 움직인다'. J6 를 한 값에 고정한 채 J2~J5 만 풀면
#   모든 잡는 위치에서 해가 나온다. J6 는 9단계·연속수확 내내 이 값 그대로.
# 2026-07-24 수정: 이 값을 0.0 으로 하드코딩하고 있었는데, 사용자가 손으로 맞춰둔 J6 와
#   달라서(예: 실제 1.0°) 9단계 중 J6 가 그 차이만큼 움직였다. "내가 놓은 자리에 고정" 이
#   사용자 사양이므로 관측자세에 저장된 J6 를 그대로 쓴다(없으면 0.0).
def _j6_from_observe(default=0.0):
    try:
        return float(json.load(open(os.path.join(HERE, "observe_pose.json")))["angles"][5])
    except Exception:
        return default
J6_LEVEL = _j6_from_observe()

# 한 단계를 명령 1회로 보내므로, 그 안에서 관절이 크게 휘둘리면 손끝이 큰 호를 그려
# 토마토를 쳐서 날린다(07-20 실제 사고). 1→2단계 관절 변화가 이 값을 넘지 않도록
# 접근 거리를 자동으로 줄인다.
STAGE_SWING_LIMIT = 25.0   # 도

STEP = 15.0        # 카메라 뎁스축 사다리의 한 칸(mm) — IK 연속성 확보용
APPROACH = 70.0    # 1단계: 토마토에서 **뎁스축 뒤로** 몇 mm 떨어져 조준할지
LIFT = 90.0        # 4단계: 잡은 뒤 **뎁스축 뒤로** 몇 mm 후퇴할지

# ── B안: 줄기를 피해 '옆에서' 접근 (2026-07-24, 시험 적용) ────────────────
# 배경: 줄기가 세로로 서 있고 토마토가 그 좌/우로 치우쳐 달리면, 정면으로 들어갈 때
#   그리퍼가 줄기까지 함께 문다. 치우친 쪽에서 진입하면 줄기를 피할 수 있다.
# 대가: 접근축을 틀면 J1 고정으로는 직진이 안 된다(실측: 10도만 틀어도 후퇴 70→45mm,
#   경로 7.8mm 휨). 그래서 2·4단계에 **J1 을 추가**해야 한다
#   = "2단계는 J2·J3·J4 만" 이라는 기존 사양이 깨진다.
# ⛔ 되돌리려면 이 값을 False 로. (git: git checkout a0fa860 -- pick8.py)
# ⚠️ 기구학은 검증 완료(±25도 전 구간 직진, 경로휨 0.0mm).
#    다만 '어느 쪽으로 치우쳤나' 자동판별이 아직 불완전하다 — 세로 줄기 검출이
#    토마토 자신을 줄기로 오인했다(폭 80px 짜리를 줄기로 봄).
#    그래서 자동은 끄고, /tmp/tomato_side.txt 에 각도를 직접 써 넣을 때만 동작한다.
#    (파일 없으면 yaw=0 = 기존 정면접근과 완전히 동일)
APPROACH_SIDE_MODE = True
APPROACH_YAW_MAX   = 25.0   # 접근축을 최대 몇 도까지 틀지 (0=정면, +=왼쪽, -=오른쪽)
STEM_SIDE_MIN_PX   = 12.0   # 토마토 중심과 줄기 중심이 이 화소 이상 벌어져야 '치우침'으로 판단

MULTISEED = [[28.82,-38.75,-24.6,18.89,-16.78,14.58],[0,-40,-30,20,-20,0],[40,-20,-60,40,-40,10],
             [0,-80,60,-30,30,0],[60,-30,-30,30,0,0],[-40,-50,-20,20,20,0],[20,10,-90,60,-45,10]]

BASKET_FILE  = os.path.join(HERE, "basket_pose.json")
# 바구니2 (2026-07-24 사용자 사양): ripe(익음) 는 바구니1, 나머지 전부(unripe/rotten/disease)는 바구니2.
BASKET_FILE2 = os.path.join(HERE, "basket_pose2.json")
def basket_for_class(cls):
    """클래스 → 바구니 번호. 0=ripe → 1번, 그 외(1,2,3) → 2번."""
    return 1 if cls == 0 else 2
CLASS_FILE = "/tmp/tomato_class.txt"     # measure 가 고른 토마토의 클래스 → run 이 바구니 결정에 사용
SKIP_FILE  = "/tmp/harvest_skip.json"    # 연속수확: 이미 시도한 위치(mm). 안 떨어진 토마토 무한재시도 방지
SKIP_TOL_MM = 30.0                       # 이 반경 안이면 '같은 토마토'로 보고 건너뜀
OBSERVE_FILE = os.path.join(HERE, "observe_pose.json")   # 8단계 관측(안전)자세 — teach-observe 로 기록
ESTOP_FILE  = "/tmp/pick8_ESTOP"   # estop_button.py 가 touch → 다음 waypoint 전에 즉시 중단
# 바구니 좌표 미측정 시 임시값 (auto_harvest.py 의 BASKETS["sell"], 로봇좌표 mm)
BASKET_XYZ_FALLBACK = [150.0, -120.0, 200.0]

# ── 기구학 (rpy = Rz·Ry·Rx — 이거 틀리면 FK가 457mm 어긋난다) ──────────
def rxm(a): c,s=math.cos(a),math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def rym(a): c,s=math.cos(a),math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rzm(a): c,s=math.cos(a),math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def rpy(r,p,y): return rzm(y)@rym(p)@rxm(r)
def Hm(xyz,rp,ang):
    M=np.eye(4); M[:3,:3]=rpy(*rp)@rzm(ang); M[:3,3]=xyz; return M
CHAIN=[([0,0,0],[0,0,0],None),([0,0,0.13956],[0,0,0],0),([0,0,-0.001],[0,1.5708,-1.5708],1),
       ([-0.1104,0,0],[0,0,0],2),([-0.096,0,0.06462],[0,0,-1.5708],3),
       ([0,-0.07318,-0.001],[1.5708,-1.5708,0],4),([0,0.0456,0],[-1.5708,0,0],5)]

def FK6(ang_deg):
    T=np.eye(4); a=[math.radians(x) for x in ang_deg]
    for xyz,rp,i in CHAIN: T=T@Hm(xyz,rp,0 if i is None else a[i])
    return T
def grasp_pt(ang_deg): return (FK6(ang_deg)@np.array([TCP[0],TCP[1],TCP[2],1.0]))[:3]

def solve_ik(target, seed=None):
    lo=[l for l,_ in JLIM]; hi=[h for _,h in JLIM]; best=None
    seeds = ([clamp(seed)] if seed is not None else []) + MULTISEED
    for s in seeds:
        r=least_squares(lambda a: grasp_pt(a)-target, s, bounds=(lo,hi), xtol=1e-11, ftol=1e-11)
        e=np.linalg.norm(grasp_pt(r.x)-target)*1000
        if best is None or e<best[1]: best=(np.array(r.x), e)
        if e<0.3: break
    return best

def prefer_j6(ang_g, target, cur_j6):
    """J6를 현재 각도에 가까운 등가 해로 바꾼다.
    그리퍼가 평행 집게라 J6를 180° 돌려도 **똑같이 잡힌다**.
    IK가 고른 J6가 현재에서 멀면(예: 7° → -160°, 167° 회전) 케이블이 감겨
    서보가 물리적으로 못 돌고 도달 실패한다(2026-07-22 실증: -140°에서 멈춤).
    → 등가인 J6±180 중 현재에서 가장 가까운 것으로 다시 푼다."""
    best=(np.array(ang_g,float), abs(ang_g[5]-cur_j6), 0.0)
    lo=[l for l,_ in JLIM]; hi=[h for _,h in JLIM]
    for delta in (180.0, -180.0):
        j6c = ang_g[5] + delta
        if not (JLIM[5][0] <= j6c <= JLIM[5][1]): continue
        def resid(a):
            return np.concatenate([grasp_pt(a)-target, [0.02*(a[5]-j6c)]])
        seed = clamp(list(ang_g[:5]) + [j6c])
        r = least_squares(resid, seed, bounds=(lo,hi), xtol=1e-11, ftol=1e-11)
        e = np.linalg.norm(grasp_pt(r.x)-target)*1000
        if e > 1.0: continue                       # 위치가 흐트러지면 버림
        d = abs(r.x[5]-cur_j6)
        if d < best[1]: best=(np.array(r.x), d, e)
    return best[0], best[1], best[2]

def clamp(ang):
    """시드를 관절한계 안으로 — least_squares는 x0가 bounds 밖이면 ValueError"""
    return [min(hi, max(lo, float(x))) for x,(lo,hi) in zip(ang, JLIM)]

def refine(seed, tp, Rg):
    """집는점 tp + 방향 Rg 를 함께 만족시키는 정밀 IK (seed 근처에서)"""
    seed = clamp(seed)
    def resid(a):
        T=FK6(a); gp=(T@np.array([TCP[0],TCP[1],TCP[2],1.0]))[:3]
        Re=T[:3,:3]@Rg.T; an=math.acos(max(-1,min(1,(np.trace(Re)-1)/2)))
        rv=np.zeros(3) if an<1e-6 else an/(2*math.sin(an))*np.array(
            [Re[2,1]-Re[1,2], Re[0,2]-Re[2,0], Re[1,0]-Re[0,1]])
        return np.concatenate([gp-tp, 0.15*rv])
    return least_squares(resid, seed, bounds=([l for l,_ in JLIM],[h for _,h in JLIM]),
                         xtol=1e-12, ftol=1e-12).x

def jac6(ang):
    """손끝 6D(위치3+회전3) 야코비안 — z 이동 시드 만들기용 (ik_grab.py 검증된 방식)"""
    T0=FK6(ang); p0=T0[:3,3]; R0=T0[:3,:3]; d=0.5; J=np.zeros((6,6))
    for i in range(6):
        da=list(ang); da[i]+=d; T=FK6(da)
        J[:3,i]=(T[:3,3]-p0)/math.radians(d)
        Re=T[:3,:3]@R0.T; an=math.acos(max(-1,min(1,(np.trace(Re)-1)/2)))
        J[3:,i]=(np.zeros(3) if an<1e-6 else an/(2*math.sin(an))*np.array(
            [Re[2,1]-Re[1,2],Re[0,2]-Re[2,0],Re[1,0]-Re[0,1]]))/math.radians(d)
    return J

def pose_up(a0, total_mm, axis):
    """a0에서 axis(단위벡터) 방향으로 total_mm 만큼 야코비안으로 밀어낸 관절각(시드용).
    axis = **카메라 뎁스축(z)** 을 base 좌표로 표현한 단위벡터. 로봇 베이스 z가 아니다."""
    a=np.array(a0,float)
    n=max(1,int(abs(total_mm)/STEP)); s=math.copysign(STEP/1000.0,total_mm) if total_mm else 0
    for _ in range(n):
        d=np.concatenate([axis*s, np.zeros(3)])
        a=a+np.degrees(np.linalg.pinv(jac6(a))@d)
    return a

JUMP_LIMIT = 45.0   # 사다리 한 칸에서 이만큼 넘게 관절이 튀면 다른 IK 해(엘보 플립) — 버린다

def ladder(ang_g, p_grasp, Rg, hmax, axis):
    """잡는 자세 ang_g 를 바닥으로 삼아 **카메라 뎁스축 뒤쪽**으로 STEP씩 물러난
    [(물러난거리mm, 관절각), ...]. 물러난 거리 = 카메라~토마토 거리가 그만큼 멀어진 것.
    2단계(접근)=이 사다리 끝에서 잡는 자세로 한 번에, 4단계=그 반대.
    ⚠️ axis 는 **카메라 뎁스축(z)** 이다 — 사용자 사양:
       1단계는 뎁스 거리를 그대로 두고 화면 x,y만, 2·4단계는 뎁스축으로만 움직인다.
    반드시 '잡는 자세에서 뒤로' 풀어야 한다. 앞에서부터 풀면 IK가 반대편 해로
    튀어 팔이 통째로 돌아간다(J1이 5°→160°로 점프하는 사고)."""
    axis = np.array(axis,float); axis = axis/np.linalg.norm(axis)
    out=[(0.0, np.array(ang_g,float))]
    prev=np.array(ang_g,float); h=0.0
    while h < hmax-1e-6:
        h=min(hmax, h+STEP)
        tp=p_grasp + axis*(h/1000.0)
        a=refine(pose_up(ang_g,h,axis), tp, Rg)
        e=np.linalg.norm(grasp_pt(a)-tp)*1000
        if e>8:
            a=refine(prev, tp, Rg); e=np.linalg.norm(grasp_pt(a)-tp)*1000
        if e>8: break
        if max(abs(x-y) for x,y in zip(a,prev))>JUMP_LIMIT: break
        out.append((h, a.copy())); prev=a
    return out

def grasp_orient(hin):
    """그리퍼 접근축(tool-z) = hin(수평) 인 잡는 자세 회전행렬.
    tool-z 를 수평 방향 hin 에 맞추면 '그리퍼와 토마토를 잇는 선이 바닥과 평행'해진다
    (사용자 사양 2026-07-23: 옆·위·아래에서 안 따고 완전 수평으로만)."""
    z = np.array(hin, float); z = z/np.linalg.norm(z)
    up = np.array([0, 0, 1.0])
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:        # hin 이 수직일 때(여기선 수평이라 안 걸림)
        x = np.array([1.0, 0, 0])
    x = x/np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])

def solve_grasp_horizontal(tom, hin):
    """잡는 자세를 푼다. **J1 = 토마토 방위각에 고정**(그래야 -hin 후퇴가 팔 세로평면
    안에 있어 J2·J3·J4 만으로 접근/후퇴 가능). **J6 = J6_LEVEL(≈0°, 홈) 고정** — J6 는
    안 움직인다(사용자 사양 2026-07-23). 나머지 J2~J5 로 위치(tom) + 접근축(tool-z)=hin(수평)
    을 만족 → 그리퍼 중심축(접근선)이 토마토 중심과 같은 높이로 정면 진입.
    이때 집게선(tool-x)은 자연히 바닥과 ~3° (거의 수평)."""
    j1 = math.degrees(math.atan2(tom[1], tom[0]))
    j1 = min(JLIM[0][1], max(JLIM[0][0], j1))
    idx = [1, 2, 3, 4]                 # J2·J3·J4·J5 만 푼다 (J6 는 고정)
    lo = [JLIM[i][0] for i in idx]; hi = [JLIM[i][1] for i in idx]
    def build(v): return [j1, v[0], v[1], v[2], v[3], J6_LEVEL]   # J6 = J6_LEVEL 고정
    def resid(v):
        a = build(v); T = FK6(a)
        gp = (T @ np.array([TCP[0], TCP[1], TCP[2], 1.0]))[:3]
        reach_z = (T[:3, :3] @ TCP)[2]     # J6 중심 → 손끝 벡터의 수직성분(접근축 수평)
        # 집게선(tool-OPEN_AXIS) 수평은 J6 고정만으로는 자세마다 3~5° 남는다(2026-07-24 실측).
        # J6 는 못 쓰므로 J2~J5 로 최대한 눕힌다. 가중치는 위치(20)보다 작게 — 위치가 우선.
        open_z = T[2, OPEN_AXIS] if LEVEL_GRIPPER else 0.0
        return np.concatenate([(gp - tom) * 20.0, [reach_z * 30.0], [open_z * 8.0]])
    best = None
    for sd in MULTISEED:
        seed = [min(hi[k], max(lo[k], sd[i])) for k, i in enumerate(idx)]
        r = least_squares(resid, seed, bounds=(lo, hi), xtol=1e-11, ftol=1e-11)
        a = np.array(build(r.x), float); T = FK6(a)
        e = np.linalg.norm(grasp_pt(a) - tom) * 1000
        reach = T[:3, :3] @ TCP
        tilt = math.degrees(math.asin(max(-1, min(1, abs(reach[2]) / (np.linalg.norm(reach)+1e-9)))))
        if e > 5: continue
        score = tilt + e                 # 수평이면서 위치 정확한 해 우선
        if best is None or score < best[0]: best = (score, a, e, tilt)
    if best is None:                     # 수평해 못 찾으면 위치만이라도
        for sd in MULTISEED:
            seed=[min(hi[k],max(lo[k],sd[i])) for k,i in enumerate(idx)]
            r=least_squares(lambda v: grasp_pt(build(v))-tom, seed, bounds=(lo,hi))
            a=np.array(build(r.x),float); e=np.linalg.norm(grasp_pt(a)-tom)*1000
            reach=FK6(a)[:3,:3]@TCP; tilt=math.degrees(math.asin(max(-1,min(1,abs(reach[2])/(np.linalg.norm(reach)+1e-9)))))
            if best is None or e<best[2]: best=(999,a,e,tilt)
    return best[1], best[2], best[3]

def approach_ladder(ang_g, hin, hmax, allow_j1=None):
    """잡는 자세 ang_g 에서 **-hin 방향(수평)** 으로 STEP씩 물러난 [(거리mm, 관절각), ...].

    2·4단계 규칙:
      · 기본(allow_j1=False) — **J1·J5·J6 고정, J2·J3·J4 만**. 2026-07-23 사용자 사양.
      · B안(allow_j1=True)   — 위에 **J1 을 추가**한다. 2026-07-24 실측으로 확인된 한계 때문:
            J1 을 고정하면 손끝은 팔의 세로평면 안에서만 움직이고, 그 평면 안의 수평
            방향은 '베이스→토마토 정면' 하나뿐이다. 그래서 옆에서 진입하려 하면
            (접근축을 10~30도 틀면) 후퇴 가능거리가 70→45→15mm 로 줄고 경로가
            5~8mm 휘었다. J1 을 풀어야 비스듬한 직선을 따라갈 수 있다.
      ⚠️ B안은 "2단계는 J2·J3·J4 만" 이라는 기존 사양을 깨는 것이다. 되돌리려면
         APPROACH_SIDE_MODE = False.
    반드시 '잡는 자세에서 뒤로' 풀어야 IK가 반대편 해로 안 튄다."""
    if allow_j1 is None:
        allow_j1 = APPROACH_SIDE_MODE
    p0 = grasp_pt(ang_g)
    j1, j5, j6 = ang_g[0], ang_g[4], ang_g[5]
    IDX = (0, 1, 2, 3) if allow_j1 else (1, 2, 3)
    lo = [JLIM[i][0] for i in IDX]; hi = [JLIM[i][1] for i in IDX]
    out = [(0.0, np.array(ang_g, float))]
    prev = np.array([ang_g[i] for i in IDX], float); h = 0.0

    def build(v):
        if allow_j1:
            return [v[0], v[1], v[2], v[3], j5, j6]        # J1 도 움직임 (J5·J6 고정)
        return [j1, v[0], v[1], v[2], j5, j6]              # J1·J5·J6 고정

    while h < hmax - 1e-6:
        h = min(hmax, h + STEP)
        tp = p0 - np.array(hin, float) * (h / 1000.0)
        def resid(v):
            a = build(v); T = FK6(a)
            gp = (T @ np.array([TCP[0], TCP[1], TCP[2], 1.0]))[:3]
            reach_z = (T[:3, :3] @ TCP)[2]        # J6→손끝 수평 유지
            return np.concatenate([(gp - tp) * 20.0, [reach_z * 30.0]])
        r = least_squares(resid, prev, bounds=(lo, hi), xtol=1e-11, ftol=1e-11)
        a = np.array(build(r.x), float)
        e = np.linalg.norm(grasp_pt(a) - tp) * 1000
        if e > 8: break
        out.append((h, a.copy())); prev = r.x
    return out

# ── 카메라 좌표축 (사용자 사양의 x,y,z 기준) ─────────────────────────
CAL = "/home/ane/.ros2/easy_handeye2/calibrations/jetcobot_handeye.calib"
_X = None
def handeye():
    """joint6 → camera 변환 X (②핸드아이 캘리브 결과)"""
    global _X
    if _X is None:
        import yaml
        cal=yaml.safe_load(open(CAL))
        t=cal['transform']['translation']; q=cal['transform']['rotation']
        x,y,z,w=q['x'],q['y'],q['z'],q['w']
        R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        _X=np.eye(4); _X[:3,:3]=R; _X[:3,3]=[t['x'],t['y'],t['z']]
    return _X

def fk_joint6(ang_deg):
    """base → joint6 (핸드아이 X 가 joint6 기준이라 마지막 링크 제외)"""
    T=np.eye(4); a=[math.radians(v) for v in ang_deg]
    for idx,(xyz,rp,i) in enumerate(CHAIN):
        if idx==len(CHAIN)-1: break
        T=T@Hm(xyz,rp,0 if i is None else a[i])
    return T

def cam_axes(ang_deg):
    """그 자세에서 카메라 좌표축을 base 좌표로 반환 (x=화면가로, y=화면세로, z=뎁스)"""
    R = fk_joint6(ang_deg)[:3,:3] @ handeye()[:3,:3]
    return R[:,0], R[:,1], R[:,2]

# ── ROS ────────────────────────────────────────────────────────────────
class Arm(Node):
    def __init__(self):
        super().__init__('pick8')
        self.pub=self.create_publisher(String,'/automato/manual_cmd',10)
        self.cur=None
        self.n_cmd=0          # 실제 발행한 동작명령 수 — 정확히 8회여야 한다
        self.seq=0            # /joint_states 수신 카운터 (2Hz)
        self.create_subscription(JointState,'/joint_states',self._js,10)
    def _js(self,m):
        if len(m.position)>=6:
            self.cur=[math.degrees(x) for x in m.position[:6]]
            self.seq += 1        # 새 측정값이 왔다는 표시 — 도착 판정은 이걸 기준으로
    def wait_state(self, sec=8.0):
        t0=time.time()
        while time.time()-t0<sec:
            rclpy.spin_once(self,timeout_sec=0.1)
            if self.cur is not None: return list(self.cur)
        return None
    def send(self,s):
        """⚠️ 반드시 **한 번만** 발행한다.
        로봇 dg_control_node 는 명령을 큐에 넣고 워커가 순서대로 실행하므로,
        같은 명령을 3번 보내면 큐에 3개가 쌓여 나중에 되풀이 실행된다.
        (2026-07-22 실증: 2단계·4단계가 큐에 중복되어 팔이 들어갔다 나왔다를 반복)
        토픽 QoS가 RELIABLE 이라 한 번으로 확실히 도착한다."""
        t0=time.time()
        while self.pub.get_subscription_count() < 1 and time.time()-t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)      # 로봇이 구독을 붙일 때까지만 대기
        m=String(); m.data=s; self.pub.publish(m)
        rclpy.spin_once(self, timeout_sec=0.05)
        return s
    def _goto_one(self, ang):
        """한 지점으로 이동하고 도착까지 대기.

        도착 판정 = ①목표 오차가 TOL 안 이거나 ②팔이 더 이상 안 움직임(정지).
        ⚠️ /joint_states 는 **2Hz(0.5초 간격)** 다. 0.1초 루프에서 그냥 세면
           메시지가 안 오는 사이를 '정지'로 착각해 아직 움직이는 중에 다음 단계로
           넘어간다(2026-07-22 실측). 그래서 **새 메시지가 왔을 때만** 비교하고,
           연속 STILL_MSGS 개가 같아야 정지로 인정한다.
        서보 자체 정밀도가 1~3°라 오차 기준만 쓰면 도달해도 '실패'가 되므로
        TOL_HARD 까지는 통과로 본다."""
        # 대기 상한을 이동량에 비례시킨다 — J6를 167° 돌리는 데 15초로는 모자라
        # 도착 전에 '실패'로 판정되던 문제(2026-07-22 실증). 속도 30에서 약 8°/s.
        swing = max(abs(a-b) for a,b in zip(ang, self.cur)) if self.cur else 0.0
        t_lim = max(TIMEOUT, 3.0 + swing/8.0)
        cmd="angles:%s@%d" % (json.dumps([round(float(x),1) for x in ang]), SPEED)
        self.n_cmd += 1
        self.send(cmd)

        t0=time.time(); err=999; got=None
        prev=None; same=0; moved=False; last_seq=self.seq
        while time.time()-t0<t_lim:
            rclpy.spin_once(self,timeout_sec=0.1)
            if self.seq == last_seq: continue      # ← 새 측정값이 올 때까지 판정 보류
            last_seq = self.seq
            got=list(self.cur); err=max(abs(a-b) for a,b in zip(got,ang))
            if err<=TOL: break
            if prev is not None:
                d=max(abs(a-b) for a,b in zip(got,prev))
                if d<STILL_TOL:
                    same+=1
                    # 이미 통과 기준(TOL_HARD) 안이면 더 기다릴 이유가 없다
                    if moved and same>=STILL_MSGS and err<=TOL_HARD: break
                    if moved and same>=STILL_MSGS+2: break  # 오차가 커도 멈췄으면 포기
                else:
                    same=0; moved=True
            prev=list(got)
        time.sleep(SETTLE)
        return err<=TOL_HARD, got, err

    def goto(self, ang, label=""):
        t_start=time.time()
        """관절각으로 이동. 큰 스윙은 MAX_SWING 이하로 잘게 쪼개서 보낸다.
        (한 번에 큰 각도를 보내면 J2/J4가 팔 무게로 스톨하고 도로 처진다 — 2026-07-22 실증)"""
        start = list(self.cur) if self.cur is not None else list(ang)
        swing = max(abs(a-b) for a,b in zip(ang,start))
        n = max(1, int(math.ceil(swing / MAX_SWING)))
        ok=True; got=None; err=999
        for k in range(1, n+1):
            if os.path.exists(ESTOP_FILE):
                self.send("stop"); raise KeyboardInterrupt("E-STOP")
            way = [s + (g-s)*k/n for s,g in zip(start, ang)]
            ok, got, err = self._goto_one(way)
            if not ok: break
        mark = "✅" if ok else "⚠️"
        extra = "" if n==1 else "  (스윙 %.0f° → %d분할)" % (swing, n)
        print("   %s %-24s %4.1f초  오차 %4.1f°  스윙%3.0f°  %s%s"
              % (mark, label, time.time()-t_start, err, swing,
                 [round(x,1) for x in (got or [])], extra))
        return ok, got, err
    def grip(self, val, label):
        """⚠️ 그리퍼가 다 닫히기 전에 다음 단계(후퇴)로 가면 잡은 토마토를 놓친다.
        속도 %d 로 완전히 닫히는 데 걸리는 시간을 넉넉히 기다린다."""
        self.n_cmd += 1
        t_start=time.time()
        self.send("grip:%d@%d" % (val, GRIPPER_SPEED))
        time.sleep(GRIP_WAIT)
        print("   ✅ %-24s %4.1f초  (grip:%d@%d)"
              % (label, time.time()-t_start, val, GRIPPER_SPEED))

# ── 바구니 자세 ────────────────────────────────────────────────────────
def solve_basket_split(a_lift, target_xyz, b_ang=None):
    """5·6단계를 한꺼번에 푼다.
      5단계: J1·J5 만 움직임   6단계: J2·J3·J4 만 움직임   (J6 는 전 구간 고정)
    J1·J5 는 '티칭한 바구니 자세'의 값을 그대로 쓴다(역산하지 않는다).
    그래야 ① 항상 바구니 쪽(오른쪽=J1 감소)으로만 돌고
          ② J5 가 티칭값이라 6단계의 J2·J3·J4 만으로 바구니에 정확히 닿는다(오차 ~0mm).

    ⚠️ 해가 여러 개다. 여러 시드로 풀어서 **팔이 덜 움직이고 관절한계에서 먼** 해를 고른다.
       (2026-07-22: 같은 바구니인데 J2가 -59° 해와 -130° 해가 나왔고,
        후자는 한계 -132°에서 1.6°밖에 안 남아 위험했다)"""
    # ⚠️ 2026-07-24 "초크슬램" 버그 수정 ────────────────────────────────
    # 손끝 xyz 만 맞추면 해가 두 갈래다:
    #   (a) J1 을 바구니 쪽으로 돌려 정상적으로 뻗는 해   (예: J1 25°→-138.7°, 이동 163.7°)
    #   (b) J1 을 거의 안 돌리고 팔을 어깨 뒤로 꺾어 넘기는 해 (예: J1 25°→+41.7°, 이동 122.3°)
    # 둘 다 오차 0.0mm 라 IK 는 구분 못 하고, 점수가 travel(최대 관절이동)이라
    # **덜 움직이는 (b) 가 이겨서** 팔이 뒤로 꺾이는 레슬링 자세가 나왔다.
    # → J1 은 풀지 않고 '티칭한 바구니 자세의 J1' 로 고정한다. 그러면 항상 바구니
    #   방향으로만 돌고(사용자 사양: 오른쪽으로만), 뒤로 꺾이는 해는 애초에 후보에서 사라진다.
    # 회전단계(=바구니 쪽으로 도는 단계)는 **J1·J5 만** 쓴다 (2026-07-24 사용자 확정.
    #   앞서 'J1·J6' 이라 했던 것을 J5 로 정정). J6 는 잡을 때 수평값 그대로 절대 안 돈다.
    # J1·J5 를 '티칭한 바구니 자세' 값으로 고정하면
    #   ① 항상 바구니 쪽(오른쪽=J1 감소)으로만 돌고, 뒤로 꺾이는 해가 후보에서 사라지며
    #   ② J5 가 티칭값이라 다음 단계의 J2·J3·J4 만으로 바구니에 정확히 닿는다(오차 ~0mm).
    base=np.array(a_lift,float)
    if b_ang is not None:
        j1_lock = float(np.clip(b_ang[0], JLIM[0][0], JLIM[0][1]))
        j5_lock = float(np.clip(b_ang[4], JLIM[4][0], JLIM[4][1]))
    else:
        # 티칭값이 없으면 목표 xyz 의 방위각으로 (그래도 바구니 쪽을 향한다). J5 는 유지.
        j1_lock = float(np.clip(math.degrees(math.atan2(target_xyz[1], target_xyz[0])),
                                JLIM[0][0], JLIM[0][1]))
        j5_lock = float(base[4])
    base[0] = j1_lock
    base[4] = j5_lock                 # 회전단계에서 J1·J5 가 여기까지 간 뒤 다음 단계를 푼다
    IDX=[1,2,3]                       # J1·J5 고정 → J2·J3·J4 만 푼다
    lo=[JLIM[i][0] for i in IDX]; hi=[JLIM[i][1] for i in IDX]
    def build(v):
        a=base.copy()
        for i,k in enumerate(IDX): a[k]=v[i]
        return a
    seeds=[[clamp(base)[i] for i in IDX]]
    if b_ang is not None:                       # 티칭한 바구니 자세도 시드로
        seeds.append([clamp(b_ang)[i] for i in IDX])
    seeds.append([-60.0, -35.0, 30.0])          # 자연스러운 기본 자세 (J2·J3·J4)
    best=None
    for sd in seeds:
        try:
            r=least_squares(lambda v: grasp_pt(build(v))-target_xyz, clamp6(sd,IDX),
                            bounds=(lo,hi), xtol=1e-11, ftol=1e-11)
        except Exception:
            continue
        a=build(r.x); e=np.linalg.norm(grasp_pt(a)-target_xyz)*1000
        if e>5: continue
        margin=min(min(a[i]-JLIM[i][0], JLIM[i][1]-a[i]) for i in range(6))  # 한계까지 여유
        travel=max(abs(a[i]-base[i]) for i in range(6))
        if margin < 8: continue                  # 한계 8° 안쪽으로 붙는 해는 버림
        score=travel                              # 덜 움직이는 해 우선
        if best is None or score<best[0]: best=(score,a,e,margin)
    if best is None:                              # 전부 탈락하면 기존 방식으로라도
        r=least_squares(lambda v: grasp_pt(build(v))-target_xyz, clamp6(seeds[0],IDX),
                        bounds=(lo,hi), xtol=1e-11, ftol=1e-11)
        a=build(r.x); e=np.linalg.norm(grasp_pt(a)-target_xyz)*1000
        best=(0,a,e,min(min(a[i]-JLIM[i][0], JLIM[i][1]-a[i]) for i in range(6)))
    _,a_final,e,margin=best
    a5=base.copy(); a5[0]=a_final[0]                            # 5단계: J1 만 (J6 고정)
    used=",".join("J%d"%(i+1) for i in (1,2,3) if abs(a_final[i]-a5[i])>0.5)
    return a5, a_final, e, used, margin

def clamp6(v, idx):
    return [min(JLIM[k][1], max(JLIM[k][0], float(x))) for x,k in zip(v, idx)]

def solve_j234(a_fixed, target_xyz):
    """J2·J3·J4 만 움직여 손끝을 target_xyz(m)로. J1·J5·J6은 a_fixed 값 그대로 고정.
    사용자 사양 6단계: '2,3,4 셋 중 1~3개만 필요대로'.
    → 실제로 움직인 관절 이름도 같이 돌려준다."""
    base=np.array(a_fixed,float)
    lo=[JLIM[i][0] for i in (1,2,3)]; hi=[JLIM[i][1] for i in (1,2,3)]
    def build(v):
        a=base.copy(); a[1],a[2],a[3]=v; return a
    r=least_squares(lambda v: grasp_pt(build(v))-target_xyz,
                    clamp(base)[1:4], bounds=(lo,hi), xtol=1e-11, ftol=1e-11)
    a=build(r.x); e=np.linalg.norm(grasp_pt(a)-target_xyz)*1000
    used=",".join("J%d"%(i+1) for i in (1,2,3) if abs(a[i]-base[i])>0.5)
    return a, e, used

def load_observe():
    """8단계 관측(안전)자세 — teach-observe 로 기록한 값이 있으면 그것, 없으면 PVIEW"""
    if os.path.exists(OBSERVE_FILE):
        d=json.load(open(OBSERVE_FILE))
        return np.array(d["angles"],float), d.get("note","")
    return np.array(PVIEW,float), "기본 PVIEW (teach-observe 미실행)"

def load_basket(which=1):
    """(관절각, 손끝xyz(m), 설명) — 없으면 (None, 임시xyz, ''). which=1|2"""
    path = BASKET_FILE if which == 1 else BASKET_FILE2
    if os.path.exists(path):
        d=json.load(open(path))
        ang=np.array(d["angles"],float)
        xyz=np.array(d.get("tcp_mm", (grasp_pt(ang)*1000).tolist()),float)/1000.0
        return ang, xyz, "바구니%d %s" % (which, d.get("note",""))
    if which == 2:      # 바구니2 미저장이면 조용히 1번으로 대체하지 않고 명확히 알린다
        print("⚠️ 바구니2 미저장(%s) — 조그 GUI 에서 '여기로 고정' 하세요" % BASKET_FILE2)
    return None, np.array(BASKET_XYZ_FALLBACK)/1000.0, ""

def basket_from_xyz(seed):
    tgt=np.array(BASKET_XYZ_FALLBACK)/1000.0
    ang,err=solve_ik(tgt, seed)
    return ang, err

# ── 8단계 계획 ─────────────────────────────────────────────────────────
def plan(cur_ang, basket=1, tom_meas=None, yaw_deg=None):
    """basket: 1=ripe 용, 2=그 외(unripe/rotten/disease) 용. 5·6단계 목표가 달라진다.
    tom_meas/yaw_deg 를 직접 주면 파일을 안 읽는다 — 연속수확(harvestall)이 쓴다.
    (한 번 촬영해서 여러 개를 통째로 계산해둘 때 필요)"""
    if tom_meas is None:
        if not os.path.exists('/tmp/tomato_base.txt'):
            print("❌ /tmp/tomato_base.txt 없음 — 먼저 `python3 pick8.py measure`")
            return None
        tom_meas=np.array([float(x) for x in open('/tmp/tomato_base.txt').read().split()])
    tom_meas=np.array(tom_meas, float)
    tom = tom_meas + np.array([0, 0, SAG_Z/1000.0])      # 화면 위쪽(바닥→천장). 처짐만큼 위를 겨냥
    print("토마토 g_base = [%.0f, %.0f, %.0f] mm" % tuple(tom_meas*1000))
    print("처짐 보정 +%.0fmm (화면 위쪽=바닥→천장) → 명령 목표 [%.0f, %.0f, %.0f] mm" % ((SAG_Z,)+tuple(tom*1000)))

    # ── 접근 방향: base에서 토마토로 향하는 **수평 반경 방향** (바닥과 평행) ──
    hin = np.array([tom[0], tom[1], 0.0])
    if np.linalg.norm(hin) < 1e-6:
        print("❌ 토마토가 로봇 바로 위 — 수평 접근 불가"); return None
    hin = hin / np.linalg.norm(hin)
    # B안: 토마토가 줄기 기준 한쪽으로 치우쳐 있으면 그쪽에서 진입하도록 접근축을 튼다.
    #   yaw>0 = 왼쪽(+y)에서 / yaw<0 = 오른쪽(-y)에서 들어온다.
    #   측정값은 measure 가 /tmp/tomato_side.txt 에 남긴다(없으면 0=정면, 기존과 동일).
    yaw = 0.0
    if APPROACH_SIDE_MODE:
        if yaw_deg is not None:
            yaw = float(yaw_deg)
        else:
            try:
                yaw = float(open('/tmp/tomato_side.txt').read().strip())
            except Exception:
                yaw = 0.0
        yaw = max(-APPROACH_YAW_MAX, min(APPROACH_YAW_MAX, yaw))
        if abs(yaw) > 0.5:
            t = math.radians(yaw); c, s = math.cos(t), math.sin(t)
            hin = np.array([c*hin[0] - s*hin[1], s*hin[0] + c*hin[1], 0.0])
            print("옆접근 %+.0f도 (%s에서 진입) — 줄기 회피" % (yaw, "왼쪽" if yaw > 0 else "오른쪽"))
    if DEPTH_PUSH:
        tom = tom + hin * (DEPTH_PUSH / 1000.0)      # 접근축으로 더 깊이 (서보 못 미치는 분)
    print("접근축(수평) = [%.2f, %.2f, %.2f]  ← 바닥과 평행" % tuple(hin))
    print("최종 목표 = [%.0f, %.0f, %.0f] mm (처짐+%.0f, 뎁스+%.0f 반영)"
          % (tom[0]*1000, tom[1]*1000, tom[2]*1000, SAG_Z, DEPTH_PUSH))

    # ── 잡는 자세: J1=토마토 방위각 고정 + tool-z(접근축)=hin(수평) ──
    ang_g, err_g, tilt_g = solve_grasp_horizontal(tom, hin)
    print("잡는 자세 IK 오차 %.1fmm, 접근축 기울기 %.0f° (0=완전수평)  %s  [J1=%.1f°]"
          % (err_g, tilt_g, "✅도달" if err_g < 3 else ("△경계" if err_g < 15 else "❌불가"), ang_g[0]))
    if err_g > 15:
        print("   → 리치 밖. 토마토를 로봇 쪽(base 340mm 안쪽)으로 당길 것"); return None
    if tilt_g > 15:
        print("   ⚠️ 접근축이 %.0f° 기울어짐 — 이 위치선 완전 수평 접근이 어렵다"%tilt_g)
    Rg = FK6(ang_g)[:3, :3]

    steps = []
    # ── 2·4단계 궤적: J1·J5·J6 고정, J2·J3·J4 만, tool-z 수평 유지 ──
    lad = approach_ladder(ang_g, hin, APPROACH)
    h_reach = lad[-1][0]
    if h_reach < STEP:
        print("\n❌ 접근 여유 없음 — 잡는 자세에서 수평으로 물러날 공간이 없음.")
        print("   토마토를 로봇 쪽으로 당기거나 관측자세를 뒤로.")
        return None
    h_app = min(APPROACH, h_reach)
    if h_app < APPROACH:
        print("접근거리 %.0fmm → **%.0fmm 로 자동 축소** (그 이상 J2·J3·J4로 수평 후퇴 불가)"
              % (APPROACH, h_app))
    idx_app = max(i for i, (h, _) in enumerate(lad) if h <= h_app + 1e-6)

    # [1] y·z 정렬 — 잡는 자세를 수평으로 %d mm 뒤에 세팅 (base y·z ≈ 토마토, x만 뒤)
    a1 = lad[idx_app][1]
    p1 = grasp_pt(a1)
    print("1단계 손끝 [%.0f, %.0f, %.0f] mm — 토마토와 y·z 맞춤, x로 %.0fmm 뒤"
          % (p1[0]*1000, p1[1]*1000, p1[2]*1000, lad[idx_app][0]))
    steps.append(("1. y·z 정렬·수평잡자세 (%.0fmm 뒤)" % lad[idx_app][0], a1))

    # [2] 수평 접근 — J2·J3·J4(+J6) 로 hin 따라 전진. J1·J5 고정.
    a_grasp = lad[0][1]
    steps.append(("2. 수평 접근 J2·J3·J4 (%.0fmm)" % lad[idx_app][0], a_grasp))

    # [3] 그리퍼 잡기
    steps.append(("3. 그리퍼 잡기", "GRIP_CLOSE"))

    # [4] 수평 후퇴 — 2단계의 정확한 역, 1번 끝난 자세로. J2·J3·J4 만.
    steps.append(("4. 수평 후퇴 → 1번 자세 (J2·J3·J4)", a1.copy()))

    # 바구니 자세
    b_ang, b_xyz, note = load_basket(basket)
    if b_ang is None:
        b_ang, b_err = basket_from_xyz(a1)
        note = "⚠️ 바구니%d 미측정 임시값 %s mm — 조그 GUI 에서 '여기로 고정' 할 것" % (
            basket, BASKET_XYZ_FALLBACK)
    b_xyz = np.array(b_xyz, float) + np.array([0, 0, BASKET_SAG/1000.0])   # 처짐만큼 위로
    print("바구니: 손끝 [%.0f, %.0f, %.0f] mm (처짐+%.0f 반영)  %s"
          % (b_xyz[0]*1000, b_xyz[1]*1000, b_xyz[2]*1000, BASKET_SAG, note))

    # [5][6] 한꺼번에 풀기 — 5는 J1·J6만, 6은 J2·J3·J4만 (J5는 4단계 값 고정)
    a5, a6, e6, used, margin = solve_basket_split(a1, b_xyz, b_ang)
    if e6 > 20:
        print("\n❌ 5·6단계 불가 — 바구니점에 %.0fmm 까지밖에 못 감." % e6)
        print("   바구니를 팔 쪽으로 옮기거나 `pick8.py teach-basket` 을 다시 하세요.")
        return None
    steps.append(("5. J1·J5만 → 바구니방향 (J6고정)", a5))
    print("5·6단계: 바구니 도달 오차 %.1fmm  관절한계 여유 %.0f°  (6단계 관절: %s)"
          % (e6, margin, used or "없음"))
    steps.append(("6. J2·J3·J4만 → 바구니", a6))

    # [7] 그리퍼 풀기
    steps.append(("7. 그리퍼 풀기", "GRIP_OPEN"))

    # [8] J2·J3·J4만 : 6번 하기 전 자세(=5번이 끝난 자세)로 복귀 — 6번의 정확한 역동작.
    #     바구니에서 손을 그대로 빼내는 동작이라 놓은 토마토를 다시 건드리지 않는다.
    steps.append(("8. J2·J3·J4만 → 6번 전 자세", a5.copy()))

    # [8] 관측자세로 복귀
    obs, onote = load_observe()
    print("관측자세: %s  (%s)" % ([round(x,1) for x in obs], onote))
    steps.append(("9. 관측자세 복귀", obs))
    return steps

def check(steps, cur_ang):
    """실물 전 시뮬 검사 — 관절한계·자세간 스윙"""
    print("\n─── 시뮬 검사 ───")
    prev=np.array(cur_ang,float); bad=0
    for label,a in steps:
        if isinstance(a,(str,tuple)): continue
        for i,(lo,hi) in enumerate(JLIM):
            if not (lo-0.1 <= a[i] <= hi+0.1):
                print("   ❌ %s : J%d=%.1f° 관절한계 %s 밖" % (label,i+1,a[i],(lo,hi))); bad+=1
        sw=max(abs(x-y) for x,y in zip(a,prev))
        if sw>MAX_SWING:
            print("   ℹ️ %-26s 스윙 %3.0f° → 실행 시 %d분할 (스톨 방지)"
                  % (label, sw, int(math.ceil(sw/MAX_SWING))))
        prev=a
    print("   %s" % ("❌ 문제 %d건" % bad if bad else "✅ 관절한계 통과"))
    return bad==0

# ── v7 AI 모델 (YOLO 4클래스) 검출 — onnxruntime, torch 불필요 ──────────
MODEL_ONNX  = os.path.join(HERE, "tomato_4cls_v6.onnx")
YOLO_CLASSES = ["ripe", "unripe", "rotten", "disease"]   # 익음/안익음/썩음/병해충
YOLO_CONF   = 0.35
# 딸 대상 클래스.
#   2026-07-24 사용자 사양: **unripe(초록, 안익음)는 따지 않는다** — 아직 안 익어서.
#   ripe(0) · rotten(2) · disease(3) 만 수확. 초록은 검출·표시는 하되 대상에서 제외.
#   투하 바구니는 basket_for_class(): ripe→바구니1, rotten·disease→바구니2.
HARVEST_CLASSES = [0, 2, 3]

def _nms(boxes, scores, iou_thr=0.45):
    idxs = scores.argsort()[::-1]; keep = []
    while len(idxs):
        i = idxs[0]; keep.append(i)
        if len(idxs) == 1: break
        xx1 = np.maximum(boxes[i,0], boxes[idxs[1:],0]); yy1 = np.maximum(boxes[i,1], boxes[idxs[1:],1])
        xx2 = np.minimum(boxes[i,2], boxes[idxs[1:],2]); yy2 = np.minimum(boxes[i,3], boxes[idxs[1:],3])
        w = np.maximum(0, xx2-xx1); h = np.maximum(0, yy2-yy1); inter = w*h
        ai = (boxes[i,2]-boxes[i,0])*(boxes[i,3]-boxes[i,1])
        ao = (boxes[idxs[1:],2]-boxes[idxs[1:],0])*(boxes[idxs[1:],3]-boxes[idxs[1:],1])
        iou = inter/(ai+ao-inter+1e-6)
        idxs = idxs[1:][iou < iou_thr]
    return keep

# ── 3D 프린트 방울토마토 색 보정 (2026-07-24) ──────────────────────────
# 문제: v6 는 박스 위치는 잘 잡는데 색 계열 분류를 틀린다.
#   초록 3D → ripe(익음) 로, 노랑 3D → unripe(안익음) 으로 오분류.
#   (v5 는 3D 자체를 학습 안 했고, v7 은 노랑을 더 심하게 unripe 로 봄 → v6 유지가 맞음)
# 해결: 우리가 쓰는 3D 프린트 토마토는 색이 빨강/초록/노랑 3가지뿐이라, 박스 안의
#   지배적 색상(HSV Hue)으로 클래스를 덮어쓴다. 사용자 정의 매핑:
#       빨강 = ripe(0) / 초록 = unripe(1) / 노랑 = disease(3)
# ⚠️ 이건 '3D 프린트 소품' 전용 규칙이다. 실제 토마토는 익어가는 중간색(주황 등)이
#    있어서 이 규칙이 틀린다 → 실물로 갈 땐 COLOR_OVERRIDE = False 로 끄고 재학습할 것.
COLOR_OVERRIDE = True
COLOR_MIN_RATIO = 0.25     # 박스 안 유효화소 중 이 비율 이상이어야 그 색으로 인정

def classify_by_color(bgr, x1, y1, x2, y2):
    """박스 안 지배 색상 → 클래스 인덱스. 판단 불가면 None(=YOLO 결과 유지)."""
    import cv2
    h0, w0 = bgr.shape[:2]
    # 테두리·배경 섞임을 줄이려 안쪽 60% 만 본다
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    bw, bh = (x2 - x1) * 0.3, (y2 - y1) * 0.3
    a = max(0, int(cx - bw)); b = min(w0, int(cx + bw))
    c = max(0, int(cy - bh)); d = min(h0, int(cy + bh))
    if b - a < 3 or d - c < 3:
        return None
    hsv = cv2.cvtColor(bgr[c:d, a:b], cv2.COLOR_BGR2HSV)
    H, S, V = hsv[:, :, 0].astype(int), hsv[:, :, 1].astype(int), hsv[:, :, 2].astype(int)
    ok = (S >= 90) & (V >= 60)          # 채도·명도 낮은 화소(그림자·배경)는 버림
    n = int(ok.sum())
    if n < 20:
        return None
    Ho = H[ok]
    cnt = {
        0: int(((Ho <= 10) | (Ho >= 170)).sum()),   # 빨강 → ripe
        3: int(((Ho >= 20) & (Ho <= 35)).sum()),    # 노랑 → disease
        1: int(((Ho >= 40) & (Ho <= 90)).sum()),    # 초록 → unripe
    }
    best = max(cnt, key=cnt.get)
    return best if cnt[best] / n >= COLOR_MIN_RATIO else None


# ── 초록(unripe) 보조 검출 (2026-07-24) ────────────────────────────────
# 문제: v6 는 초록 3D 방울토마토를 **아예 못 찾는다**(conf 0.02 까지 낮춰도 박스 0개).
#   박스가 없으니 색 보정(classify_by_color)도 손쓸 수 없다 → 검출 자체를 보태야 한다.
# 방법: HSV 초록 마스크 → 윤곽 → '동그랗고 꽉 찬' 덩어리만 토마토로 인정.
#   배경의 줄기·잎도 초록이므로 모양으로 걸러낸다:
#     · 원형도(4πA/P²)  줄기는 가늘고 길어서 낮다
#     · 볼록성(A/convexHull) 잎은 들쭉날쭉해서 낮다
#     · 가로세로비        줄기는 극단적으로 치우친다
# ⚠️ 이것도 '3D 프린트 소품' 전용이다. 실물 초록토마토·잎은 구분이 더 어렵다.
# 줄기·잎과 어떻게 가르나 — 2026-07-24 두 번의 실측을 거친 결론:
#   1차 시도: 채도만으로 가름(S>=175). 그 장면에선 완벽했으나(토마토246 vs 줄기95)
#             **연한 초록 토마토가 있는 다른 장면에서 실패**했다. 그 장면의 초록화소
#             채도 분위는 25%=76 / 50%=111 / 75%=187 로, 175 는 상위 25% 만 통과시킨다.
#             → 채도 하나로는 조명·개체에 따라 무너진다. 일반화 실패.
#   2차(현재): **깊이로 실제 크기를 잰다.**  r_mm = r_px * 깊이(m) / fx * 1000
#             방울토마토는 실제 반지름 10~20mm, 줄기·지지대는 2~5mm 라 물리적으로 갈린다.
#             이건 조명·색조와 무관해서 채도보다 훨씬 안정적이다.
#             채도는 '초록인지' 만 보게 낮추고(100), 크기 판정을 깊이에 맡긴다.
#   ⚠️ 깊이가 없으면(depth 미전달) 크기 판정을 못 하므로 보수적으로 채도 175 를 쓴다.
# 2026-07-24 재활성. 잎사귀 오검출은 '색조 균일도' 조건 추가로 해결(아래 3조건 AND).
#   검증: 잎사귀+굵은줄기 장면 → 초록 0개 / 연한초록 2개 장면 → 정확히 2개.
GREEN_DETECT      = True
GREEN_SAT_MIN     = 100        # 깊이가 있을 때: '초록인가' 만 판정 (연한 초록도 통과)
# 색조(Hue) 하한 — 잎·줄기를 **마스크 단계에서** 배제하는 가장 강력한 기준.
#   2026-07-24 6개 장면·화소 4만개 실측:
#       3D 초록 토마토 : Hue 중앙 81, 10~90% = 79~82   (청록)
#       잎사귀·줄기     : Hue 중앙 54, 10~90% = 49~69   (연두)
#   Hue>=75 이면 토마토 95.7% 남고 잎은 2.7% 만 남는다.
#   ⚠️ 이 값은 '이 3D 프린트 토마토' 색에 맞춘 것이다. 실물 초록토마토는 연두라
#      이 기준에 안 걸린다 → 실물로 갈 땐 GREEN_DETECT=False 로 끄고 재학습할 것.
GREEN_HUE_MIN     = 75
GREEN_SAT_MIN_NOD = 175        # (미사용) 깊이 없으면 검출 자체를 안 한다 — 위 주석 참조
GREEN_MIN_R       = 10.0       # px 하한 (깊이 판정 전 1차 거르기)
GREEN_FILL        = 0.70       # 찾은 원의 사각영역이 이 비율 이상 초록이어야 인정
GREEN_R_MM        = (8.0, 28.0)  # 실제 반지름 허용 범위(mm)
#   하한 12.0 → 10.0 → 8.0 (2026-07-24). Hue 하한을 75 로 조이면서 마스크가
#   토마토의 '순수 청록' 부분만 남겨 blob 이 작아졌다. 예전 16mm 로 재던 것이
#   같은 토마토인데 9.6~11.6mm 로 측정된다. 하한도 그만큼 낮춰야 한다.
# 가장자리가 중심보다 이만큼 멀어야(볼록) 함. 평면(잎) 배제용.
#   4.0 → 2.0 (2026-07-24): 다른 토마토·잎에 **일부 가려진** 초록토마토는 가장자리 링이
#   가리는 물체 위에 놓여 돌출이 1mm 밖에 안 나온다(전체가 보이면 7~8mm).
#   4.0 → 2.0 → 1.0 (2026-07-24). Hue 로 잎을 이미 배제했으므로 돌출을 느슨하게
#   해도 잎이 안 들어온다. 7개 장면 스윕에서 (크기 8.0, 돌출 1.0) 이 전부 통과.
GREEN_BULGE_MM    = 1.0
# 색조 균일도 — 이게 잎사귀를 거르는 결정타다. 2026-07-24 실측:
#     초록 3D 토마토 : Hue 표준편차 0.49 / 0.65   (단색 프린트라 거의 안 흔들림)
#     잎사귀          : Hue 표준편차 6.05         (잎맥·명암으로 크게 흔들림)
#   임계값 1.5~4.0 어디를 잡아도 결과가 같아 경계에 걸린 게 아니다 → 3.0 채택.
GREEN_HUE_STD_MAX = 3.0
GREEN_SCORE       = 0.50       # 보조검출 신뢰도(YOLO 와 구분되게 고정값)

def detect_green_blobs(bgr, exist, depth_m=None, fx=None):
    """초록 덩어리 안에서 '원'을 찾아 → [(x1,y1,x2,y2,1,score)]. exist 와 겹치면 제외.
    depth_m(미터 단위 깊이배열)·fx 가 오면 **실제 크기**로 줄기를 걸러낸다(권장).

    ⚠️ 윤곽 모양(원형도·볼록성)으로 거르는 방식은 실패했다(2026-07-24 실측):
       초록 토마토가 **줄기·뒷배경 초록과 한 덩어리로 이어져서** 윤곽이 길쭉해지고
       원형도 0.19 / 볼록성 0.65 로 탈락했다. 덩어리를 쪼갤 수도 없다.
    → 그래서 '덩어리의 모양'이 아니라 **거리변환으로 덩어리 안의 원을 직접 찾는다**.
       붙어 있어도 토마토는 국소적으로 가장 두꺼운 원이라 중심이 뚜렷하게 잡힌다.
    """
    import cv2
    use_depth = depth_m is not None and fx
    # ⛔ 깊이가 없으면 아예 검출하지 않는다 (2026-07-24 회귀검증에서 확정).
    #    깊이 없이 채도+반지름(px)만으로는 줄기를 못 거른다 — 초록이 하나도 없는 장면에서
    #    줄기·지지대를 5개나 토마토로 잡았다. 로봇이 줄기를 잡으러 가는 것보다
    #    초록을 놓치는 편이 안전하므로, 깊이 없으면 빈 목록을 돌려준다.
    if not use_depth:
        return []
    smin = GREEN_SAT_MIN
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (GREEN_HUE_MIN, smin, 50), (92, 255, 255))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    # 거리변환: 각 화소가 배경에서 얼마나 떨어졌나 = 그 자리에 들어가는 최대 원의 반지름
    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    out = []
    work = dist.copy()
    for _ in range(10):                    # 후보를 넉넉히 훑고, 크기로 거른다
        _, rmax, _, (cx, cy) = cv2.minMaxLoc(work)
        if rmax < GREEN_MIN_R:
            break
        r = float(rmax)
        # 억제 반경 1.6r → 1.1r : 1.6r 은 바로 옆에 붙은 다른 토마토까지 지워서
        # 두 번째 토마토가 후보에서 사라졌다(2026-07-24 실측).
        cv2.circle(work, (int(cx), int(cy)), int(r * 1.1), 0, -1)   # 다음 원 찾기

        # ── 중심 다시 잡기 (2026-07-24, 영상 IMG_2777 33초 오동작의 원인 수정) ──
        # 거리변환의 최대점은 '초록 덩어리에 들어가는 가장 큰 원'의 중심이다. 토마토가
        # 줄기·잎과 이어져 있으면 그 중심이 토마토 밖으로 끌려간다 → 좌표가 아래로
        # 밀려 팔이 엉뚱한 곳을 잡았다(그때 z=202mm, 실제 토마토는 더 위였다).
        # → 씨앗점 주변에서 **토마토 표면만** 떼어내 그 무게중심을 쓴다:
        #    ① 씨앗점과 깊이가 비슷한 화소만 남긴다(줄기·잎은 앞뒤로 어긋나 빠진다)
        #    ② 원형 커널로 열기 → 가늘게 이어진 줄기 목이 끊긴다
        #    ③ 씨앗점이 속한 덩어리만 골라 최소외접원의 중심·반지름을 쓴다
        if use_depth:
            d_seed = float(depth_m[int(cy), int(cx)])
            if 0.08 < d_seed < 1.2:
                pad = int(r * 2.2)
                wx1, wy1 = max(0, int(cx - pad)), max(0, int(cy - pad))
                wx2 = min(bgr.shape[1], int(cx + pad)); wy2 = min(bgr.shape[0], int(cy + pad))
                loc = (mask[wy1:wy2, wx1:wx2] > 0)
                dloc = depth_m[wy1:wy2, wx1:wx2]
                loc = loc & (np.abs(dloc - d_seed) < 0.025) & (dloc > 0.08)
                kk = max(3, int(r * 0.8) | 1)
                loc = cv2.morphologyEx(loc.astype(np.uint8) * 255, cv2.MORPH_OPEN,
                                       cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kk, kk)))
                nlab, lab = cv2.connectedComponents(loc)
                sy, sx = int(cy) - wy1, int(cx) - wx1
                lid = lab[sy, sx] if (0 <= sy < lab.shape[0] and 0 <= sx < lab.shape[1]) else 0
                if lid == 0 and nlab > 1:                       # 씨앗이 열기로 지워졌으면 최근접
                    ys, xs = np.nonzero(lab)
                    if len(ys):
                        j = np.argmin((ys - sy) ** 2 + (xs - sx) ** 2); lid = lab[ys[j], xs[j]]
                if lid > 0:
                    pts = cv2.findNonZero((lab == lid).astype(np.uint8))
                    if pts is not None and len(pts) >= 12:
                        (ncx, ncy), nr = cv2.minEnclosingCircle(pts)
                        # ⚠️ **중심만** 옮기고 반지름은 거리변환 값을 그대로 쓴다.
                        #    열기 커널(r*0.8)이 토마토 가장자리를 깎아 반지름이 43→30px 로
                        #    줄었고, 그 때문에 멀쩡한 토마토가 크기필터(12~28mm)에서
                        #    탈락했다(2026-07-24 실측). 반지름은 원래 값이 더 정확하다.
                        move = math.hypot(ncx + wx1 - cx, ncy + wy1 - cy)
                        if move <= r * 1.2:       # 너무 멀리 튀면 잘못 잡은 것 → 무시
                            cx, cy = ncx + wx1, ncy + wy1

        x1, y1 = max(0, int(cx - r)), max(0, int(cy - r))
        x2, y2 = min(bgr.shape[1], int(cx + r)), min(bgr.shape[0], int(cy + r))
        if x2 - x1 < 6 or y2 - y1 < 6:
            continue
        sub = mask[y1:y2, x1:x2]
        if sub.size == 0 or (sub > 0).mean() < GREEN_FILL:
            continue
        if any(bx1 <= cx <= bx2 and by1 <= cy <= by2 for bx1, by1, bx2, by2, _, _ in exist):
            continue
        # ── 세 가지를 모두 통과해야 토마토로 인정한다 ──────────────────
        # (2026-07-24 실측. 하나씩만 쓰면 전부 실패했다:
        #   · 채도만  → 연한 초록 토마토를 놓침
        #   · 크기만  → 잎사귀 16.6mm 가 토마토 16.0mm 와 겹쳐 통과
        #   · 돌출만  → 잎사귀 7.0mm 가 토마토 6.0~8.0mm **사이**에 들어와 통과
        #   세 조건을 AND 로 걸어야 잎사귀가 떨어진다.)
        H0, W0 = depth_m.shape
        gy, gx = np.mgrid[0:H0, 0:W0]
        rr = np.sqrt((gx - cx) ** 2 + (gy - cy) ** 2)
        core = (rr <= r * 0.35) & (depth_m > 0.08) & (depth_m < 1.2)
        ring = (rr >= r * 0.80) & (rr <= r * 1.05) & (depth_m > 0.08) & (depth_m < 1.2)
        inner = (rr <= r * 0.7)
        if core.sum() < 20 or ring.sum() < 20 or inner.sum() < 50:
            continue
        dc = float(np.median(depth_m[core]))
        dr_ = float(np.median(depth_m[ring]))

        # ① 실제 크기 — 얇은 줄기 배제
        r_mm = r * dc / float(fx) * 1000.0
        if not (GREEN_R_MM[0] <= r_mm <= GREEN_R_MM[1]):
            continue
        # ② 구형성 — 가장자리가 중심보다 멀어야(볼록) 한다. 평평한 배경 배제
        if (dr_ - dc) * 1000.0 < GREEN_BULGE_MM:
            continue
        # ③ 색조 균일도 — 3D 프린트품은 단색이라 Hue 가 거의 안 흔들린다(0.5~0.7).
        #    잎사귀는 잎맥·명암 때문에 6.0 수준으로 크게 흔들린다. 이게 결정타다.
        if float(hsv[:, :, 0][inner].std()) > GREEN_HUE_STD_MAX:
            continue
        out.append((float(x1), float(y1), float(x2), float(y2), 1, GREEN_SCORE))
    return out


def yolo_detect(sess, bgr, conf_thr=YOLO_CONF, imgsz=640, depth_m=None, fx=None):
    """v7 ONNX 추론 → [(x1,y1,x2,y2,cls,score), ...]. 출력 (1,8,8400)=cx,cy,w,h+4클래스.
    depth_m·fx 를 주면 초록 보조검출이 실제 크기로 줄기를 걸러낸다(정확도 크게 상승)."""
    import cv2
    h0, w0 = bgr.shape[:2]
    r = min(imgsz/h0, imgsz/w0)
    nw, nh = int(round(w0*r)), int(round(h0*r))
    canvas = np.full((imgsz, imgsz, 3), 114, np.uint8)
    left = (imgsz-nw)//2; top = (imgsz-nh)//2
    canvas[top:top+nh, left:left+nw] = cv2.resize(bgr, (nw, nh))
    blob = canvas[:, :, ::-1].transpose(2,0,1)[None].astype(np.float32)/255.0
    out = sess.run(None, {sess.get_inputs()[0].name: blob})[0][0].T   # (8400,8)
    cs = out[:, 4:8]; cls = cs.argmax(1); conf = cs.max(1)
    m = conf > conf_thr; out, cls, conf = out[m], cls[m], conf[m]
    if len(out) == 0: return []
    cx, cy, w, h = out[:,0], out[:,1], out[:,2], out[:,3]
    x1 = (cx-w/2-left)/r; y1 = (cy-h/2-top)/r; x2 = (cx+w/2-left)/r; y2 = (cy+h/2-top)/r
    boxes = np.stack([x1, y1, x2, y2], 1)
    keep = _nms(boxes, conf)
    dets = [(float(boxes[i,0]), float(boxes[i,1]), float(boxes[i,2]), float(boxes[i,3]),
             int(cls[i]), float(conf[i])) for i in keep]
    if COLOR_OVERRIDE:
        fixed = []
        for (bx1, by1, bx2, by2, c, s) in dets:
            cc = classify_by_color(bgr, bx1, by1, bx2, by2)
            fixed.append((bx1, by1, bx2, by2, cc if cc is not None else c, s))
        dets = fixed
    if GREEN_DETECT:                      # v6 가 놓치는 초록(unripe) 을 색+크기로 보탠다
        dets = dets + detect_green_blobs(bgr, dets, depth_m, fx)
    return dets

# ── main ───────────────────────────────────────────────────────────────
def do_measure():
    """카메라로 토마토 검출 → /tmp/tomato_base.txt (TF 불필요, 자체 FK 사용)"""
    import yaml, cv2, pyrealsense2 as rs
    CAL="/home/ane/.ros2/easy_handeye2/calibrations/jetcobot_handeye.calib"
    def quat2R(x,y,z,w):
        return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    cal=yaml.safe_load(open(CAL)); tx=cal['transform']['translation']; rq=cal['transform']['rotation']
    X=np.eye(4); X[:3,:3]=quat2R(rq['x'],rq['y'],rq['z'],rq['w']); X[:3,3]=[tx['x'],tx['y'],tx['z']]

    rclpy.init(); nd=Arm(); ang=nd.wait_state(8.0)
    if ang is None:
        print("❌ /joint_states 없음 — DDS 확인 (source ros_env_automato.sh)"); rclpy.try_shutdown(); return
    print("현재 관절각 %s" % [round(x,1) for x in ang])
    # g_base→joint6 (핸드아이 X는 joint6 기준이므로 마지막 링크 제외)
    T=np.eye(4); a=[math.radians(x) for x in ang]
    for idx,(xyz,rp,i) in enumerate(CHAIN):
        if idx==len(CHAIN)-1: break
        T=T@Hm(xyz,rp,0 if i is None else a[i])
    bj=T

    pipe=rs.pipeline(); cfg=rs.config()
    cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
    cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
    prof=pipe.start(cfg); align=rs.align(rs.stream.color)
    intr=prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    dscale=prof.get_device().first_depth_sensor().get_depth_scale()   # 깊이원값→미터
    import onnxruntime as ort
    sess = ort.InferenceSession(MODEL_ONNX, providers=["CPUExecutionProvider"])
    # 연속수확용 '이미 시도한 위치' 목록 (mm, base 기준). 안 떨어진 토마토를 계속 다시 잡는 것 방지.
    skip = []
    if os.path.exists(SKIP_FILE):
        try: skip = json.load(open(SKIP_FILE))
        except Exception: skip = []
    if skip:
        print("건너뛸 위치 %d곳 (반경 %.0fmm): %s" % (len(skip), SKIP_TOL_MM,
              [[round(x) for x in s] for s in skip]))

    det=None; bgr=None; det_cls=-1; det_sc=0.0; tom=None
    for _ in range(40):
        fr=align.process(pipe.wait_for_frames())
        bgr=np.asanyarray(fr.get_color_frame().get_data()); df=fr.get_depth_frame()
        dmap=np.asanyarray(df.get_data()).astype(np.float32)*dscale   # 미터
        dets = yolo_detect(sess, bgr, depth_m=dmap, fx=intr.fx)
        if not dets: time.sleep(0.05); continue
        # 딸 대상 클래스 우선, 없으면 아무 토마토. 신뢰도 높은 순으로 훑는다.
        pref = [d for d in dets if d[4] in HARVEST_CLASSES] or dets
        pref.sort(key=lambda d: d[5], reverse=True)
        for x1,y1,x2,y2,cls,sc in pref:
            u=int((x1+x2)/2); v=int((y1+y2)/2); r=int(min(x2-x1, y2-y1)/2)
            if r<6: continue
            # 깊이: 박스 안쪽 60%, 하위 25% 분위수(반들거림 대비 뒤배경 오측 방지)
            ri=max(3,int(r*0.6)); st=max(1,ri//6)
            ds=[df.get_distance(min(639,max(0,u+dx)),min(479,max(0,v+dy)))
                for dx in range(-ri,ri+1,st) for dy in range(-ri,ri+1,st) if dx*dx+dy*dy<=ri*ri]
            ds=[d for d in ds if 0.08<d<1.2]
            if len(ds)<5: continue
            dep=float(np.percentile(ds,25))
            # base 좌표를 여기서 바로 구해 skip 판정 (건너뛸 대상이면 다음 후보로)
            rad_m=r*dep/intr.fx
            cam=rs.rs2_deproject_pixel_to_point(intr,[u,v],dep+rad_m)
            cand=(bj@X@np.array([cam[0],cam[1],cam[2],1.0]))[:3]
            if any(np.linalg.norm(cand*1000.0-np.array(s,float))<SKIP_TOL_MM for s in skip):
                continue
            det=(u,v,r,dep); det_cls=cls; det_sc=sc; tom=cand; break
        if det: break
    if not det:
        pipe.stop(); rclpy.try_shutdown()
        print("❌ 토마토 미검출 (v6 모델) — 시야에 토마토가 있는지 확인"); return
    u,v,r,depth=det
    open('/tmp/tomato_base.txt','w').write("%.6f %.6f %.6f\n"%tuple(tom))
    # 이번에 고른 클래스 → run 이 바구니를 정한다 (ripe=1번, 그 외=2번)
    open(CLASS_FILE,'w').write("%d\n" % det_cls)
    # 시도 목록에 추가 (다음 measure 가 같은 자리를 다시 안 고르게)
    skip.append([round(float(x)*1000.0, 1) for x in tom])
    try: json.dump(skip, open(SKIP_FILE,'w'))
    except Exception: pass
    ev=os.path.join(HERE,"evidence/2026-07-22_TCP잡기테스트")
    os.makedirs(ev,exist_ok=True)
    cname=YOLO_CLASSES[det_cls] if 0<=det_cls<4 else "?"
    vis=bgr.copy(); cv2.circle(vis,(u,v),r,(0,255,0),2); cv2.circle(vis,(u,v),3,(0,0,255),-1)
    cv2.putText(vis,"%s %.0f%% base=[%.0f,%.0f,%.0f]mm d=%.0fcm"%(cname,det_sc*100,tom[0]*1000,tom[1]*1000,tom[2]*1000,depth*100),
                (10,28),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,255,255),2)
    cv2.imwrite(os.path.join(ev,"10_pick8_검출.jpg"),vis)
    pipe.stop(); rclpy.try_shutdown()
    print("✅ 검출(v6 AI): %s 신뢰도 %.0f%%  화면(%d,%d) r=%dpx 깊이 %.1fcm"
          % (YOLO_CLASSES[det_cls] if 0<=det_cls<4 else "?", det_sc*100, u,v,r,depth*100))
    print("   토마토 g_base = [%.0f, %.0f, %.0f] mm  → /tmp/tomato_base.txt" % tuple(tom*1000))
    print("   확인이미지: %s/10_pick8_검출.jpg" % ev)

def stem_side_yaw(bgr, depth_m, box, bj, X, intr, tom_base):
    """토마토가 줄기 기준 어느 쪽으로 치우쳤는지 → 접근 yaw(도).

    2026-07-24. 전역 '세로 줄기 찾기' 는 실패했다(줄기가 클립·잎으로 끊겨 검출이
    프레임마다 3개→1개로 흔들리고, 토마토 자신을 줄기로 오인했다).
    → **토마토 박스 바로 좌·우만** 보고, 그중 **토마토와 같은 깊이(±3.5cm)** 인
      초록 화소만 센다. 배경 초록이 안 섞여서 3프레임 연속 판정이 일치했다.

    반환 yaw: 줄기 반대쪽에서 진입하도록 부호를 정한다.
      hin 을 +yaw 로 돌리면 출발점이 base -y 쪽으로 밀린다 = 로봇 오른쪽에서 진입.
      따라서 '줄기가 base +y(왼쪽)' 이면 +yaw, '줄기가 -y' 면 -yaw.
    """
    import cv2
    x1,y1,x2,y2 = [int(v) for v in box[:4]]
    H,W = depth_m.shape
    bw, bh = x2-x1, y2-y1
    if bw < 6 or bh < 6:
        return 0.0, "박스작음"
    cx, cy = (x1+x2)//2, (y1+y2)//2
    core = depth_m[max(0,cy-bh//6):cy+bh//6, max(0,cx-bw//6):cx+bw//6]
    cvd = core[(core>0.08)&(core<1.2)]
    if cvd.size < 10:
        return 0.0, "깊이없음"
    d0 = float(np.median(cvd))
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    green = cv2.inRange(hsv,(35,60,40),(92,255,255)) > 0
    near  = (np.abs(depth_m-d0) < 0.035) & (depth_m > 0.08)
    by1,by2 = max(0,y1-bh//3), min(H,y2+bh//3)
    wsel = max(8, int(bw*0.8))
    Ls, Rs = slice(max(0,x1-wsel), x1), slice(x2, min(W,x2+wsel))
    nl = int((green&near)[by1:by2, Ls].sum())
    nr = int((green&near)[by1:by2, Rs].sum())
    if nl+nr < 40:
        return 0.0, "줄기없음(정면)"
    ratio = (nr-nl)/float(nl+nr)
    if abs(ratio) < 0.35:
        return 0.0, "가운데(정면)"
    # 화면 좌/우가 base 의 어느 쪽인지는 자세마다 다르다 → 실제로 역투영해 base y 로 판단
    sx = (x2 + wsel//2) if ratio > 0 else (x1 - wsel//2)
    sx = max(0, min(W-1, int(sx)))
    import pyrealsense2 as rs
    cam = rs.rs2_deproject_pixel_to_point(intr, [sx, cy], d0)
    stem_base = (bj @ X @ np.array([cam[0],cam[1],cam[2],1.0]))[:3]
    stem_left = stem_base[1] > tom_base[1]          # 줄기가 base +y(왼쪽)에 있나
    yaw = APPROACH_YAW_MAX if stem_left else -APPROACH_YAW_MAX
    return yaw, ("줄기 왼쪽→오른쪽에서 진입" if stem_left else "줄기 오른쪽→왼쪽에서 진입")


def do_harvest_all(limit=20, dry_run=False):
    """dry_run=True 면 촬영·계산만 하고 **로봇에 명령을 한 번도 보내지 않는다**.
    (2026-07-24: ESTOP 파일로 막으려 했으나 아래 'ESTOP 흔적 제거' 코드가 그 파일을
     지워버려 실제로 로봇이 움직였다. 그래서 파일이 아니라 코드 경로로 분리한다.)"""
    """연속 수확 — **한 번만 촬영해서 전부 계산**하고, 그 뒤엔 촬영·계산 없이 움직이기만.

    2026-07-24 사용자 사양. 기존 GUI 방식(measure→run 을 개당 새 프로세스로 반복)은
    개당 8~17초가 준비시간(import·모델로드 3초, 카메라 자동노출 4초, ROS 노드 2회 생성)
    으로 날아갔다. 여기서는 모델·카메라·ROS 노드를 **한 번만** 열고 재사용한다.

    ⚠️ 트레이드오프: 처음 좌표를 한 번만 따므로, 앞의 토마토를 따다가 줄기가 흔들리면
       뒤 토마토의 좌표가 실제와 어긋날 수 있다. (사용자가 이 방식을 명시적으로 선택)
    """
    import yaml, cv2, pyrealsense2 as rs, onnxruntime as ort
    CAL="/home/ane/.ros2/easy_handeye2/calibrations/jetcobot_handeye.calib"
    def quat2R(x,y,z,w):
        return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    cal=yaml.safe_load(open(CAL)); tx=cal['transform']['translation']; rq=cal['transform']['rotation']
    X=np.eye(4); X[:3,:3]=quat2R(rq['x'],rq['y'],rq['z'],rq['w']); X[:3,3]=[tx['x'],tx['y'],tx['z']]

    t0=time.time()
    rclpy.init(); nd=Arm(); cur=nd.wait_state(8.0)
    if cur is None:
        print("❌ /joint_states 없음 — DDS 확인"); rclpy.try_shutdown(); return 2
    print("현재 관절각 %s" % [round(x,1) for x in cur])

    T=np.eye(4); a=[math.radians(x) for x in cur]
    for idx,(xyz,rp,i) in enumerate(CHAIN):
        if idx==len(CHAIN)-1: break
        T=T@Hm(xyz,rp,0 if i is None else a[i])
    bj=T

    # ── 촬영 1회 ────────────────────────────────────────────────
    pipe=rs.pipeline(); cfg=rs.config()
    cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
    cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
    prof=pipe.start(cfg); align=rs.align(rs.stream.color)
    intr=prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    dscale=prof.get_device().first_depth_sensor().get_depth_scale()
    sess=ort.InferenceSession(MODEL_ONNX, providers=["CPUExecutionProvider"])
    bgr=None; dmap=None; dets=[]
    for _ in range(40):
        fr=align.process(pipe.wait_for_frames())
        bgr=np.asanyarray(fr.get_color_frame().get_data())
        df=fr.get_depth_frame()
        dmap=np.asanyarray(df.get_data()).astype(np.float32)*dscale
        dets=yolo_detect(sess,bgr,depth_m=dmap,fx=intr.fx)
        if dets: break
        time.sleep(0.05)
    if not dets:
        pipe.stop(); rclpy.try_shutdown()
        print("❌ 토마토 미검출"); return 3
    bgr=bgr.copy(); dmap=dmap.copy()

    # ── 검출 전부를 base 좌표 + 바구니 + 접근각으로 변환 ──────────────
    targets=[]
    for (x1,y1,x2,y2,cls,sc) in dets:
        if cls not in HARVEST_CLASSES: continue
        u=int((x1+x2)/2); v=int((y1+y2)/2); r=int(min(x2-x1,y2-y1)/2)
        if r<6: continue
        ri=max(3,int(r*0.6)); st=max(1,ri//6)
        ds=[df.get_distance(min(639,max(0,u+dx)),min(479,max(0,v+dy)))
            for dx in range(-ri,ri+1,st) for dy in range(-ri,ri+1,st) if dx*dx+dy*dy<=ri*ri]
        ds=[d for d in ds if 0.08<d<1.2]
        if len(ds)<5: continue
        dep=float(np.percentile(ds,25))
        cam=rs.rs2_deproject_pixel_to_point(intr,[u,v],dep+r*dep/intr.fx)
        tom=(bj@X@np.array([cam[0],cam[1],cam[2],1.0]))[:3]
        yaw,why = stem_side_yaw(bgr,dmap,(x1,y1,x2,y2),bj,X,intr,tom)
        targets.append(dict(tom=tom, cls=cls, sc=sc, yaw=yaw, why=why, u=u, v=v))
    pipe.stop()
    print("\n촬영 1회 완료 (%.1f초) — 수확 대상 %d개" % (time.time()-t0, len(targets)))
    if not targets:
        rclpy.try_shutdown(); print("❌ 수확 대상 없음"); return 3

    # 가까운 것부터 (팔이 덜 움직이게)
    targets.sort(key=lambda t: np.linalg.norm(t["tom"]))

    # ── 전부 미리 계산 ──────────────────────────────────────────
    obs = json.load(open(OBSERVE_FILE))["angles"] if os.path.exists(OBSERVE_FILE) else cur
    print("\n" + "="*62)
    print("  좌표를 딴 지금, %d개의 9단계를 통째로 계산합니다 (이후 계산 없음)" % len(targets))
    print("="*62)
    plans=[]
    for i,t in enumerate(targets,1):
        bno=basket_for_class(t["cls"])
        print("\n[%d/%d] %s %.0f%%  base=[%.0f,%.0f,%.0f]mm  →바구니%d  %s"
              % (i,len(targets),YOLO_CLASSES[t["cls"]],t["sc"]*100,
                 t["tom"][0]*1000,t["tom"][1]*1000,t["tom"][2]*1000,bno,t["why"]))
        steps=plan(obs, basket=bno, tom_meas=t["tom"], yaw_deg=t["yaw"])
        if steps is None:
            print("   ⚠️ 계획 실패 — 건너뜀"); continue
        if not check(steps, obs):
            print("   ⚠️ 시뮬 검사 실패 — 건너뜀"); continue
        plans.append((t,steps))
    if not plans:
        rclpy.try_shutdown(); print("\n❌ 실행 가능한 계획 없음"); return 4

    # ── 계산만 모드: 여기서 끝. 로봇에 아무 명령도 안 보낸다 ──────────
    if dry_run:
        print("\n" + "="*62)
        print("  🧮 계산 전용 모드 — 로봇에 명령을 보내지 않고 종료합니다.")
        print("     계획 %d개 준비됨. 실제 실행은 `python3 pick8.py harvestall`" % len(plans))
        print("="*62)
        rclpy.try_shutdown()
        return 0

    # ── 실행 (촬영·계산 없음) ────────────────────────────────────
    print("\n" + "="*62)
    print("  계산 끝 — %d개 연속 수확 시작. 지금부터 촬영·계산 안 함." % len(plans))
    print("  🛑 멈추려면 조그창의 빨간 [비상정지]")
    print("="*62)
    if os.path.exists(ESTOP_FILE): os.remove(ESTOP_FILE)
    ok_n=0; t_run=time.time()
    for i,(t,steps) in enumerate(plans,1):
        if os.path.exists(ESTOP_FILE):
            nd.send("stop"); print("\n🛑 비상정지 — 중단"); break
        print("\n───── %d/%d 번째 (%s → 바구니%d) ─────"
              % (i,len(plans),YOLO_CLASSES[t["cls"]],basket_for_class(t["cls"])))
        t1=time.time(); failed=False
        for label,ang in steps:
            if os.path.exists(ESTOP_FILE):
                nd.send("stop"); print("🛑 비상정지 — 중단"); failed=True; break
            if isinstance(ang,str):
                nd.grip(0 if ang=="GRIP_CLOSE" else 100, label); continue
            okk,got,err = nd.goto(ang,label)
            if not okk and label.startswith(("2.","3.","4.")):
                print("   ❌ 파지 구간 도달 실패 — 이 개체 중단"); failed=True; break
        if os.path.exists(ESTOP_FILE): break
        if not failed:
            ok_n+=1; print("   ✅ 완료 (%.1f초)" % (time.time()-t1))
    print("\n" + "="*62)
    print("  연속 수확 종료 — %d/%d 성공, 총 %.1f초 (평균 %.1f초/개)"
          % (ok_n,len(plans),time.time()-t_run, (time.time()-t_run)/max(1,ok_n)))
    print("="*62)
    rclpy.try_shutdown()
    return 0 if ok_n else 6


def main():
    mode = sys.argv[1] if len(sys.argv)>1 else 'plan'

    if mode=='measure':
        do_measure(); return 0

    if mode=='harvestall':      # 한 번 촬영 → 전부 계산 → 연속 실행 (촬영·계산 반복 없음)
        return do_harvest_all()
    if mode=='harvestall-plan': # 위와 같되 **계산까지만** — 로봇 명령 0회
        return do_harvest_all(dry_run=True)

    rclpy.init(); nd=Arm()
    cur=nd.wait_state(8.0)
    if cur is None:
        print("❌ /joint_states 없음 — DDS 확인:  source ros_env_automato.sh && ros2 node list")
        rclpy.try_shutdown(); return 2
    print("현재 관절각 %s\n" % [round(x,1) for x in cur])

    if mode=='teach-observe':
        json.dump({"angles":[round(x,1) for x in cur],
                   "note":"pick8.py teach-observe 로 기록 (%s)"%time.strftime('%F %T'),
                   "tcp_mm":[round(float(x)*1000,1) for x in grasp_pt(cur)]},
                  open(OBSERVE_FILE,'w'), ensure_ascii=False, indent=1)
        print("✅ 관측(안전)자세 저장 → %s" % OBSERVE_FILE)
        print("   관절각 %s\n   손끝 %s mm" %
              ([round(x,1) for x in cur],[round(float(x)*1000) for x in grasp_pt(cur)]))
        rclpy.try_shutdown(); return 0

    if mode=='teach-basket':
        json.dump({"angles":[round(x,1) for x in cur],
                   "note":"pick8.py teach-basket 로 기록 (%s)"%time.strftime('%F %T'),
                   "tcp_mm":[round(float(x)*1000,1) for x in grasp_pt(cur)]},
                  open(BASKET_FILE,'w'), ensure_ascii=False, indent=1)
        print("✅ 바구니 투하 자세 저장 → %s" % BASKET_FILE)
        print("   관절각 %s\n   손끝 %s mm" %
              ([round(x,1) for x in cur],[round(float(x)*1000) for x in grasp_pt(cur)]))
        rclpy.try_shutdown(); return 0

    # 바구니 선택: measure 가 남긴 클래스로 결정 (ripe=1번 / unripe·rotten·disease=2번).
    # 파일이 없으면(단독 plan 실행 등) 1번.
    det_cls = 0
    if os.path.exists(CLASS_FILE):
        try: det_cls = int(open(CLASS_FILE).read().strip())
        except Exception: det_cls = 0
    bno = basket_for_class(det_cls)
    print("검출 클래스 %s → **바구니%d** 로 투하"
          % (YOLO_CLASSES[det_cls] if 0 <= det_cls < 4 else "?", bno))

    steps = plan(cur, basket=bno)
    if steps is None:
        rclpy.try_shutdown(); return 3

    print("\n─── %d분할 계획 ───" % N_STAGES)
    for label,a in steps:
        show = a if isinstance(a,str) else ("(실측 보정)" if isinstance(a,tuple) else [round(x,1) for x in a])
        print("   %-28s %s" % (label, show))
    ok = check(steps, cur)

    if mode=='plan':
        print("\n(plan 모드 — 로봇 안 움직임. 실행은 `python3 pick8.py run`)")
        rclpy.try_shutdown(); return 0 if ok else 4
    if not ok:
        print("\n❌ 시뮬 검사 실패 — 실행 중단"); rclpy.try_shutdown(); return 4

    # 9분할 계산은 여기서 이미 전부 끝났다. 아래는 '계산 없이 이동만' 한다.
    nmove = sum(1 for _,a in steps if not isinstance(a,(str,tuple)))
    print("\n" + "="*62)
    print("  계산 끝. 지금부터는 움직이기만 합니다.")
    print("  ─ 미리 계산해둔 이동 지점 : %d 개" % nmove)
    print("  ─ 움직이는 중에는 계산 안 함 (좌표를 딸 때 9단계를 통째로 다 계산했음)")
    print("  ─ 속도 %d / 그리퍼속도 %d / 지점마다 도착확인 후 %.1f초 쉼 (최대 %d초 대기)"
          % (SPEED, GRIPPER_SPEED, SETTLE, TIMEOUT))
    print("  ─ 🛑 멈추려면 : 화면의 빨간 [비상정지] 버튼을 누르세요")
    print("="*62)
    if os.path.exists(ESTOP_FILE): os.remove(ESTOP_FILE)
    t_run=time.time()
    hist=[]
    for label,a in steps:
        if os.path.exists(ESTOP_FILE):
            nd.send("stop"); print("\n🛑 비상정지 감지 — 즉시 중단 (그리퍼는 건드리지 않음)")
            rclpy.try_shutdown(); return 9
        # ⚠️ a 는 numpy 배열일 수 있다. `a=="문자열"` 은 배열 비교라 ValueError.
        if isinstance(a, str):
            nd.grip(0 if a=="GRIP_CLOSE" else 100, label); continue
        okk,got,err = nd.goto(a,label)
        hist.append(a)
        if not okk and label.startswith(("2.","3.","4.")):
            print("   ❌ 파지 구간에서 도달 실패 — 중단(그리퍼는 건드리지 않음)")
            rclpy.try_shutdown(); return 5

    if RETURN_BY_REVERSE:
        print("\n─── 역순 복귀 ───")
        for a in reversed(hist): nd.goto(a,"역순")

    n=nd.n_cmd
    print("\n" + "="*62)
    print("  총 소요 %.1f초" % (time.time()-t_run))
    if n==N_STAGES:
        print("  ✅ %d분할 완료 — 발행한 명령 정확히 %d회" % (N_STAGES, N_STAGES))
    else:
        print("  ⚠️ %d분할 완료 — 그런데 명령을 %d회 발행함 (%d여야 정상)" % (N_STAGES, n, N_STAGES))
    print("="*62)
    rclpy.try_shutdown(); return 0

if __name__=="__main__":
    sys.exit(main())
