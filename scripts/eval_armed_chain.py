#!/usr/bin/env python3
"""armed pose를 **사슬로** 재는 스크립트 — 타겟별 armed pose 재검토용.

왜 새로 만드나 (기존 측정이 전부 놓친 것)
-----------------------------------------
`docs/ARMED_POSE_HANDOFF.md` 1.4절은 타겟별 armed pose를 "이동량 중앙 −20.6%"로
재고도 복잡도를 이유로 기각했다. 2026-08-02에 다시 재니 −6.3%였다. **그런데 둘 다
`armed_i -> 목표_i` 편도만 잰 값이다.**

타겟별 armed pose의 이득은 편도에 있지 않다. 지금 사이클은

    look -> armed -> [t1] -> armed -> [t2] -> armed -> ... -> armed -> look

로 **매 수확마다 같은 자리로 돌아온다**. 사람이라면 토마토 하나 따고 집에 갔다
다시 나오지 않는다. 타겟별로 두면

    look -> armed_1 -> [t1] -> armed_2 -> [t2] -> ... -> look

가 되어 `목표_i -> armed_{i+1}` 전이가 생기는데, **그 전이를 잰 측정이 없었다.**
그래서 기각 결정도 재검토도 근거 수치 없이 이뤄졌다. 이 스크립트가 그 수치를 만든다.

무엇을 비교하나
---------------
접근·파지·후퇴는 어느 방식이나 동일하므로 **전이(transit)만** 잰다.

    A   현재      look -> armed -> [t_i] -> armed -> ... -> armed -> look
    B1  타겟별    look -> armed_1 -> [t_i] -> armed_{i+1} -> ... -> look
                  armed_i = look pose 기하 + J1을 목표 방위각에 맞춘 것(규칙)
    B2  타겟별    같은 구조인데 armed_i를 전이 합이 최소가 되게 후보에서 고름
    C   없음      look -> [t_1] -> [t_2] -> ... -> look  (정렬에서 정렬로 직행)

**A'(현재 + 복귀 삭제)는 따로 재지 않는다.** 단일 armed pose에서는 복귀를 없애도
다음 목표로 가려면 `[0/5]`가 결국 같은 armed pose를 경유하므로
`정렬_i -> armed -> 정렬_{i+1}`이 되어 **A와 경로가 같아진다.** "복귀 삭제"는
armed pose가 목표마다 다를 때만 내용이 생긴다 — 이것 자체가 결과의 일부다.

같이 재는 것 셋
---------------
1. **벽이 armed 족을 어디서 자르는가.** J1 격자 각각에 대해 자세 자체의 유효성.
   1.4절은 "J1 0~60도에 흩어진 9개 자세"를 복잡도 근거로 들었는데, 그 사이 벽이
   생겼다. 상한이 잘렸다면 그 전제부터 성립하지 않는다.
2. **각 armed_i의 그리퍼 끝단 여유.** 단일 armed pose는 33.3cm였다(1.3절).
   타겟별은 목표 쪽으로 돌린 자세라 줄어드는 것이 당연하고, 얼마나 줄어드는지가
   채택 여부를 가른다.
3. **순서 의존성.** 현재 수확 순서는 카메라 깊이 z 오름차순인데
   (`harvest_sequence_node.py:149`) 이는 J1·방위각과 무관하다. 단일 armed pose에서는
   목표 간 비용이 대칭이라 순서가 공짜였지만, 타겟별로 가면 순서가 곧 비용이 된다.

사용법
------
    # 스택은 런북(문서 8절 실행 순서) 1)번 그대로
    python3 -u scripts/eval_armed_chain.py --repeat 3 \\
        --octomap bags/bed_look_octomap_wall.bin --octomap-acm \\
        --csv bags/armed_chain.csv
"""

import argparse
import csv
import functools
import json
import math
import os
import statistics as st
import sys

print = functools.partial(print, flush=True)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from moveit_msgs.msg import PlanningScene, RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

import octomap_io
import scene_objects
import sweep_targets_planned as S
from sweep_targets_planned import PlanProbe, select_approach
from mycobot_280_pick import coord_to_goal_node as N

LOOK = list(N.LOOK_POSE_JOINT_POSITIONS)
ARMED = list(N.ARMED_POSE_JOINT_POSITIONS)


def armed_family(j1_deg):
    """look pose 기하를 유지한 채 J1만 바꾼 자세. 1.2절의 결론(안전한 대기
    자세는 뻗은 자세가 아니라 접힌 자세)을 그대로 따른다."""
    q = list(LOOK)
    q[0] = math.radians(j1_deg)
    return q


def state_valid(node, joints):
    if not hasattr(node, '_sv'):
        node._sv = node.create_client(GetStateValidity, '/check_state_validity')
        node._sv.wait_for_service(timeout_sec=10)
    req = GetStateValidity.Request()
    rs = RobotState()
    rs.joint_state = JointState()
    rs.joint_state.name = list(N.JOINT_NAMES)
    rs.joint_state.position = list(joints)
    rs.is_diff = False
    req.robot_state = rs
    req.group_name = N.GROUP_NAME
    fut = node._sv.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    r = fut.result()
    hits = sorted({c.contact_body_2 if c.contact_body_1 == '<octomap>'
                   else c.contact_body_1 for c in r.contacts})
    return r.valid, hits


# 수확통 놓는 지점 — docs/ARMED_POSE_HANDOFF.md 4절 확정값.
# 통 y=+0.15, 림 z=0.100, 놓는 지점(그리퍼 끝단) z=0.130. flange는 그보다
# 그리퍼 길이만큼 위. 목표 제약은 위치 6x6cm 상자 + yaw 자유(같은 절).
BIN_FLANGE = (0.0, 0.15, 0.130 + N.GRIPPER_LENGTH_OFFSET_M)
BIN_BOX = [0.06, 0.06, 0.04]


def to_bin(node, frm, repeat):
    """관절 자세 -> 수확통 놓는 자세. (비용, 도착 관절값)을 돌려준다."""
    node.start_pose = list(frm)
    quat = N._compute_look_at_quat_xyzw((0.0, 0.0, -1.0))   # 그리퍼가 아래를 봄
    best = None
    for _ in range(repeat):
        r = node.plan_to(BIN_FLANGE, quat, box_xyz=BIN_BOX, yaw_free=True)
        if r['ok'] and (best is None or r['travel_deg'] < best['travel_deg']):
            best = r
    if best is None:
        return None, None
    return best['travel_deg'], [best['final'][n] for n in N.JOINT_NAMES]


def transit(node, frm, to, repeat):
    """관절 자세 -> 관절 자세 전이 비용. 실패하면 None."""
    best = None
    for _ in range(repeat):
        r = node.plan_to_config(list(to), from_joints=list(frm))
        if r['ok'] and (best is None or r['travel_deg'] < best):
            best = r['travel_deg']
    return best


def main():
    p = argparse.ArgumentParser(description='armed pose 사슬 비용 비교')
    p.add_argument('--targets', default='bags/lab_bed_detections.json')
    p.add_argument('--octomap', default='bags/bed_look_octomap_wall.bin')
    p.add_argument('--octomap-acm', action='store_true',
                   help='그리퍼/손목이 octomap voxel과 충돌해도 되게 한다')
    p.add_argument('--repeat', type=int, default=3)
    p.add_argument('--orientation-tolerance', type=float, default=0.2)
    p.add_argument('--j1-grid', nargs='+', type=float,
                   default=[-20, -10, 0, 10, 20, 30, 40, 50, 60, 70])
    p.add_argument('--b2-candidates', nargs='+', type=float,
                   default=[-20, 0, 20, 40])
    # [2026-08-02] 수확통을 사이클에 넣으면 순위가 바뀌는가.
    # 통은 J1 +105도라 armed(+40)와도 정렬(+10~+62)과도 멀다 — 경유 구조에
    # 미치는 영향이 자명하지 않아 별도로 잰다.
    p.add_argument('--with-bin', action='store_true',
                   help='목표마다 파지 직후 수확통에 놓는 구간을 사슬에 넣는다')
    # [2026-08-02] 노드가 실제로 쓸 **고정 통 자세**로 재기 위한 옵션.
    # --with-bin의 통 구간은 pose goal(상자+yaw 자유)이라 플래너가 매번 다른
    # IK 해를 고른다 — 그건 "통에 놓을 수 있는가"의 하한이지 노드가 하는 일이
    # 아니다. 노드는 move_to_configuration으로 **하나의 관절벡터**에 간다.
    # 이 파일(scripts/find_bin_pose.py 결과)을 주면 그 자세로 재고, 사슬 D
    # (통 자세 = 대기 자세)가 비교 대상에 추가된다.
    p.add_argument('--bin-pose', default=None,
                   help='고정 통 자세 JSON(bags/bin_pose.json). 주면 통 구간을 '
                        '이 관절벡터로 재고 사슬 D를 추가한다')
    p.add_argument('--csv', default=None)
    args = p.parse_args()

    S.ORIENTATION_TOLERANCE_RAD = args.orientation_tolerance
    with open(args.targets, encoding='utf-8') as fh:
        dets = json.load(fh)['detections']

    rclpy.init()
    node = PlanProbe(2.0, 10, ARMED)
    if not node.wait_ready():
        raise SystemExit('move_group 없음')

    # ---- 씬 (함정 8: 시작 상태를 가정하지 않는다) ----
    scene_objects.publish_tomatoes(node, dets, remove=True)
    scene_objects.allow_gripper_octomap_collisions(node, False, quiet=True)
    octomap_io.clear_octomap(node, rclpy)
    scene_objects.publish_tomatoes(node, dets)
    scene_objects.allow_gripper_tomato_collisions(node, dets, quiet=True)
    owp = octomap_io.load_octomap_file(args.octomap)
    octomap_io.inject_octomap(node, rclpy, PlanningScene, owp)
    scene_objects.allow_gripper_octomap_collisions(node, args.octomap_acm,
                                                   quiet=True)
    print(f'씬: 열매 {len(dets)}개 + octomap {len(octomap_io.decode_msg(owp.octomap))} voxel'
          f' + octomap ACM {"완화" if args.octomap_acm else "완화 없음"}')

    BINQ = None
    if args.bin_pose:
        with open(args.bin_pose, encoding='utf-8') as fh:
            BINQ = list(json.load(fh)['joints_rad'])
        ok, hits = state_valid(node, BINQ)
        print(f'고정 통 자세: J1 {math.degrees(BINQ[0]):+.1f}도, '
              f'이 씬에서 {"유효" if ok else f"충돌 {hits}"}')
        if not ok:
            raise SystemExit('통 자세가 이 씬에서 충돌한다 — 재도출할 것')
    print()

    # ---- 1) 벽이 armed 족을 어디서 자르는가 ----
    print('[1] armed pose 족의 유효성 (자세 자체의 충돌 검사)')
    print(f'{"J1(도)":>8}{"유효":>7}{"끝단 여유":>11}   충돌 링크')
    valid_j1 = []
    for j1 in args.j1_grid:
        q = armed_family(j1)
        ok, hits = state_valid(node, q)
        pos = None
        try:
            from find_armed_pose_cartesian import fk_flange
            pos = fk_flange(node, q)
        except Exception:
            pass
        clr = (scene_objects.tip_clearance(pos[:3], pos[3:], dets)
               if pos else float('nan'))
        if ok:
            valid_j1.append(j1)
        print(f'{j1:+8.0f}{"OK" if ok else "충돌":>7}{clr*100:10.1f}cm   {hits}')
    print(f'\n유효한 J1: {valid_j1}')
    if not valid_j1:
        raise SystemExit('유효한 armed pose가 없다 — 씬을 확인할 것')
    lo, hi = min(valid_j1), max(valid_j1)

    # ---- 2) 목표별 정렬 자세 ----
    print(f'\n[2] 목표별 정렬 관절해 수집 (반복 {args.repeat}회)')
    goals = []
    for i, d in enumerate(dets, 1):
        b, why = select_approach(d['base_x'], d['base_y'], d['base_z'])
        if b is None:
            print(f'  #{i} 기하 게이트 탈락({why})')
            continue
        node.start_pose = ARMED
        best = None
        for _ in range(args.repeat + 2):
            r = node.plan_to(b['align'], b['quat'])
            if r['ok'] and (best is None or r['travel_deg'] < best['travel_deg']):
                best = r
        if best is None:
            print(f'  #{i} z={d["base_z"]:.3f} 정렬 플래닝 실패 — 사슬에서 제외')
            continue
        az = math.degrees(math.atan2(d['base_y'], d['base_x']))
        goals.append({
            'idx': i, 'det': d, 'best': b, 'az_deg': az,
            'joints': [best['final'][n] for n in N.JOINT_NAMES],
            'align_j1_deg': best['j1_deg'],
            'depth': d.get('camera_z', d['base_x']),
        })
        print(f'  #{i} z={d["base_z"]:.3f} 방위 {az:+5.1f}도  정렬 J1 {best["j1_deg"]:+6.1f}도')
    n = len(goals)
    print(f'\n사슬에 들어간 목표 {n}/{len(dets)}개')

    # ---- 3) armed_i 배정 ----
    # B1 규칙: 정렬 자세의 J1을 그대로 따라가되 벽이 허용하는 범위로 클램프.
    #          "미리 목표 쪽으로 돌려둔다"를 가장 단순하게 구현한 것.
    for g in goals:
        g['b1_j1'] = max(lo, min(hi, g['align_j1_deg']))
        g['b1'] = armed_family(g['b1_j1'])

    # --with-bin이면 각 목표의 파지 직후 통에 놓고 그 자리에서 다음 구간을
    # 시작한다. 통 경유 지점이 다음 전이의 출발점이 되므로 사슬 구조가 바뀐다.
    def after_grasp(frm):
        """파지 직후 위치 -> (다음 구간 출발 자세, 통까지 비용).
        통을 안 쓰면 그 자리 그대로."""
        if not args.with_bin:
            return frm, 0.0
        if BINQ is not None:
            # 고정 통 자세 — 노드와 같은 관절 목표로 간다.
            c = transit(node, frm, BINQ, args.repeat)
            if c is None:
                return None, None
            return list(BINQ), c
        c, j = to_bin(node, frm, args.repeat)
        if c is None:
            return None, None
        return j, c

    def chain_A():
        cost = transit(node, LOOK, ARMED, args.repeat)
        if cost is None:
            return None, []
        legs = [('look->armed', cost)]
        for g in goals:
            a = transit(node, ARMED, g['joints'], args.repeat)
            if a is None:
                return None, legs
            after, bin_cost = after_grasp(g['joints'])
            if after is None:
                return None, legs
            b = transit(node, after, ARMED, args.repeat)
            if b is None:
                return None, legs
            cost += a + bin_cost + b
            legs.append((f'armed->#{g["idx"]}->[통]->armed', a + bin_cost + b))
        back = transit(node, ARMED, LOOK, args.repeat)
        if back is None:
            return None, legs
        return cost + back, legs

    def chain_B(key):
        first = transit(node, LOOK, goals[0][key], args.repeat)
        if first is None:
            return None, []
        cost = first
        legs = [('look->armed_1', first)]
        last = None
        for i, g in enumerate(goals):
            a = transit(node, g[key], g['joints'], args.repeat)
            if a is None:
                return None, legs
            after, bin_cost = after_grasp(g['joints'])
            if after is None:
                return None, legs
            cost += a + bin_cost
            legs.append((f'armed_{i+1}->#{g["idx"]}->[통]', a + bin_cost))
            last = after
            if i + 1 < n:
                nxt = goals[i + 1]
                b = transit(node, after, nxt[key], args.repeat)
                if b is None:
                    return None, legs
                cost += b
                legs.append((f'[통]->armed_{i+2}', b))
        back = transit(node, last, LOOK, args.repeat)
        if back is None:
            return None, legs
        return cost + back, legs

    def chain_C():
        first = transit(node, LOOK, goals[0]['joints'], args.repeat)
        if first is None:
            return None, []
        cost = first
        legs = [('look->#1', first)]
        last = None
        for i in range(n):
            after, bin_cost = after_grasp(goals[i]['joints'])
            if after is None:
                return None, legs
            cost += bin_cost
            last = after
            if i + 1 < n:
                b = transit(node, after, goals[i + 1]['joints'], args.repeat)
                if b is None:
                    return None, legs
                cost += b
                legs.append((f'#{goals[i]["idx"]}->[통]->#{goals[i+1]["idx"]}',
                             bin_cost + b))
        back = transit(node, last, LOOK, args.repeat)
        if back is None:
            return None, legs
        return cost + back, legs

    def chain_D():
        """통 자세 = 대기 자세. look -> 통 -> [t_i] -> 통 -> ... -> look.

        A와 달리 armed pose를 경유하지 않는다. 목표 사이 비용이 **목표마다
        독립**이라(모두 같은 자세를 경유) C와 달리 **수확 순서에 무관**하다 —
        1.5절이 C의 채택 조건으로 걸었던 "실제 순서로 재확인"이 D에는 붙지
        않는 이유다."""
        cost = transit(node, LOOK, BINQ, args.repeat)
        if cost is None:
            return None, []
        legs = [('look->통', cost)]
        for g in goals:
            a = transit(node, BINQ, g['joints'], args.repeat)      # 통 -> 정렬
            b = transit(node, g['joints'], BINQ, args.repeat)      # 파지 후 -> 통
            if a is None or b is None:
                return None, legs
            cost += a + b
            legs.append((f'통->#{g["idx"]}->통', a + b))
        back = transit(node, BINQ, LOOK, args.repeat)
        if back is None:
            return None, legs
        return cost + back, legs

    print('\n[3] 사슬 비용 (전이만, 접근·파지·후퇴 제외)')
    results = {}
    chains = [('A  현재(단일 armed)', chain_A),
              ('B1 타겟별(규칙)', lambda: chain_B('b1')),
              ('C  armed 없음', chain_C)]
    if BINQ is not None:
        chains.append(('D  통 자세=대기 자세', chain_D))
    for label, fn in chains:
        total, legs = fn()
        results[label] = total
        print(f'  {label:<22} '
              + (f'{total:7.0f}도' if total else f'실패(구간 {len(legs)}개까지)'))

    # ---- B2: 후보 중 전이 합 최소 (그리디) ----
    cands = [c for c in args.b2_candidates if lo <= c <= hi]
    if cands:
        print(f'\n[4] B2 — 후보 {cands}에서 그리디 선택')
        prev = LOOK
        cost = 0.0
        chosen = []
        okall = True
        for i, g in enumerate(goals):
            best = None
            for c in cands:
                q = armed_family(c)
                a = transit(node, prev, q, args.repeat)
                b = transit(node, q, g['joints'], args.repeat)
                if a is None or b is None:
                    continue
                if best is None or a + b < best[0]:
                    best = (a + b, c, q)
            if best is None:
                okall = False
                break
            cost += best[0]
            chosen.append(best[1])
            after, bin_cost = after_grasp(g['joints'])
            if after is None:
                okall = False
                break
            cost += bin_cost
            prev = after
        if okall:
            back = transit(node, prev, LOOK, args.repeat)
            if back is not None:
                cost += back
                results['B2 타겟별(그리디)'] = cost
                print(f'  B2 타겟별(그리디)      {cost:7.0f}도')
                print(f'  선택된 J1: {chosen}')
                print(f'  서로 다른 자세 {len(set(chosen))}개')

    # ---- 요약 ----
    base = results.get('A  현재(단일 armed)')
    print('\n=== 요약 (전이 비용, 목표 %d개) ===' % n)
    for k, v in results.items():
        if v is None:
            print(f'  {k:<22}   실패')
        elif base:
            print(f'  {k:<22} {v:7.0f}도  ({100*(v-base)/base:+6.1f}%)')
        else:
            print(f'  {k:<22} {v:7.0f}도')

    # ---- 5) B1 자세들의 끝단 여유 ----
    print('\n[5] B1 armed_i의 그리퍼 끝단 여유 (단일 armed pose는 33.3cm)')
    try:
        from find_armed_pose_cartesian import fk_flange
        clrs = []
        for g in goals:
            pos = fk_flange(node, g['b1'])
            c = scene_objects.tip_clearance(pos[:3], pos[3:], dets)
            clrs.append(c)
            print(f'  #{g["idx"]:2d} J1 {g["b1_j1"]:+6.1f}도  여유 {c*100:5.1f}cm'
                  + ('  **8cm 미만**' if c < 0.08 else ''))
        print(f'  최소 {min(clrs)*100:.1f}cm, 중앙 {st.median(clrs)*100:.1f}cm')
    except Exception as exc:
        print(f'  FK 실패: {exc}')

    if args.csv:
        with open(args.csv, 'w', newline='', encoding='utf-8') as fh:
            w = csv.writer(fh)
            w.writerow(['method', 'transit_deg', 'n_targets', 'octomap_acm'])
            for k, v in results.items():
                w.writerow([k, '' if v is None else round(v, 1), n,
                            args.octomap_acm])
        print(f'\nCSV 저장: {args.csv}')

    scene_objects.allow_gripper_octomap_collisions(node, False, quiet=True)
    scene_objects.publish_tomatoes(node, dets, remove=True)
    octomap_io.clear_octomap(node, rclpy)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
