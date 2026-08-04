#!/usr/bin/env python3
"""**같은 스택 한 세션 안에서** 씬 구성 x 시작 자세를 교차 비교한다.

무엇을 답하는 스크립트인가
--------------------------
`docs/ARMED_POSE_HANDOFF.md` 5절(octomap 재검증) — 그전까지의 모든 수치는 씬에 **열매(구)만**
넣고 잰 것이다. 줄기·지지대·잎은 형상을 몰라 빠져 있어 수치가 낙관적이고,
그게 실물과의 가장 큰 차이일 가능성이 높다고 적혀 있었다.

그 형상은 `bags/bed_look_slim`을 재생하면 octomap으로 만들어진다. 여기서는
그 octomap을 `octomap_io.py`로 굳혀 두었다가 주입하고, **열매만 있을 때와
줄기·지지대까지 있을 때가 얼마나 다른지**를 잰다.

왜 한 프로세스에서 교차로 도는가 (이 스크립트의 존재 이유)
----------------------------------------------------------
같은 문서 2절 함정 3의 교훈이 "씬이 다르면 수치가 다르다"인데, 그 함정은
**측정을 따로 돌렸기 때문에** 뒤늦게 드러났다. 조건을 바꿀 때마다 스택을
새로 띄우면 octomap도 ACM도 미세하게 달라져서, 차이가 조건 때문인지 환경
때문인지 구분할 수 없다.

그래서 여기서는 스택 하나 · 주입 octomap 하나 · 목표 목록 하나를 고정해 두고
**씬 구성과 시작 자세만 갈아 끼운다.** 이렇게 해야 "-20%"가 조건의 효과라고
말할 수 있다.

씬 구성 4가지
-------------
    empty    아무것도 없음                — 오염 없는 기준선(문서 4절 622도)
    tomato   열매 15개(구) + ACM          — 현재 유효한 값(문서 4절 502도)
    octomap  녹화 장면 octomap만          — 줄기·지지대만의 효과를 분리
    full     열매 + ACM + octomap         — **실물에 가장 가까운 씬**

`octomap`을 따로 두는 이유는 효과를 쪼개기 위해서다. full만 재면 "나빠졌다"는
알지만 열매 때문인지 줄기 때문인지 모른다.

**octomap에는 ACM 완화를 걸지 않는다.** octomap은 열매를 지운 클라우드에서
나온 것이라(마스킹) 줄기·지지대·잎만 들어 있고, 그건 그리퍼가 스쳐도 되는
대상이 아니다.

사용법
------
    # 터미널 A — 가벼운 스택(카메라·필터 노드 없이. 클라우드 발행자가 있으면
    #            주입한 octomap이 덮어써진다)
    ros2 launch mycobot_280_moveit2 demo_octomap.launch.py \
        enable_octomap:=false enable_camera:=false \
        enable_pointcloud_filter:=false use_rviz:=false

    # 터미널 B
    python3 -u scripts/eval_bed_scene.py --octomap bags/bed_look_octomap.bin \
        --repeat 3 --csv bags/bed_scene_eval.csv

측정 전 반드시 확인 (문서 2절)
------------------------------
    pgrep -xc move_group            # 1이어야 한다 (함정 1)
    --orientation-tolerance 0.2     # 노드와 같은 값 (함정 5). 기본값으로 넣어 뒀다
"""

import argparse
import csv
import functools
import json
import math
import os
import statistics as st
import sys
import time

print = functools.partial(print, flush=True)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from moveit_msgs.msg import PlanningScene

import octomap_io
import scene_objects
import sweep_targets_planned as S
from sweep_targets_planned import PlanProbe, select_approach
from mycobot_280_pick import coord_to_goal_node as N

SCENES = ('empty', 'tomato', 'octomap', 'full')

# 시작 자세는 노드가 실제로 쓰는 상수를 그대로 가져온다. 여기에 값을 베껴
# 두면 노드가 바뀌었을 때 조용히 옛 값을 재게 된다.
START_POSES = {
    'look': ('look pose', list(N.LOOK_POSE_JOINT_POSITIONS)),
    'armed': ('armed pose', list(N.ARMED_POSE_JOINT_POSITIONS)),
}

# A/B 분기 판별. docs/ROSBAG_HANDOFF.md 4절 — A는 30~50도, B는 120~140도이고
# 그 사이 70~120도가 통째로 비어 있어 80도 어디를 잘라도 결과가 같다.
# **베드 전용 기준이다** — 수확통(방위각 +90도)에는 쓰면 안 된다.
BRANCH_A_MAX_J1_DEG = 80.0


def apply_scene(node, dets, scene, owp, current):
    """씬 구성을 목표 상태로 만든다. 이미 맞으면 아무것도 하지 않는다.

    octomap을 넣고 빼는 것보다 토마토를 넣고 빼는 것이 싸므로, 호출 순서를
    정렬해 두면 재구성 횟수가 줄어든다(main에서 octomap 유무로 묶어 돈다).
    """
    want_tomato = scene in ('tomato', 'full')
    want_octomap = scene in ('octomap', 'full')
    has_tomato, has_octomap = current

    if has_tomato and not want_tomato:
        scene_objects.publish_tomatoes(node, dets, remove=True)
    elif want_tomato and not has_tomato:
        scene_objects.publish_tomatoes(node, dets)
        scene_objects.allow_gripper_tomato_collisions(node, dets, quiet=True)

    if want_octomap != has_octomap:
        if want_octomap:
            octomap_io.inject_octomap(node, rclpy, PlanningScene, owp)
        else:
            # 빈 octomap을 diff로 발행하는 것으로는 안 지워진다 —
            # octomap_io.clear_octomap()의 주석 참고.
            octomap_io.clear_octomap(node, rclpy)

    return (want_tomato, want_octomap)


def verify_scene(node, dets, expect_tomato, expect_octomap):
    """씬이 정말 의도한 상태인지 되읽어 확인한다.

    함정 3(장애물 없이 재면 뚫고 가는 경로가 성공으로 잡힌다)은 결국 "씬이
    비어 있는 줄 몰랐다"는 사고다. 조건이 바뀔 때마다 확인하지 않으면 같은
    실수를 반복하게 되므로, 매 조합 앞에서 한 번씩 검증한다.
    """
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene
    if not hasattr(node, '_verify_client'):
        node._verify_client = node.create_client(GetPlanningScene,
                                                 '/get_planning_scene')
        node._verify_client.wait_for_service(timeout_sec=10)
    req = GetPlanningScene.Request()
    req.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                                 | PlanningSceneComponents.OCTOMAP)
    fut = node._verify_client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    world = fut.result().scene.world
    n_tomato = sum(1 for o in world.collision_objects
                   if o.id.startswith(scene_objects.TOMATO_OBJECT_PREFIX))
    n_voxel = (len(octomap_io.decode_msg(world.octomap.octomap))
               if world.octomap.octomap.data else 0)
    ok = ((n_tomato == len(dets)) == expect_tomato) and ((n_voxel > 0) == expect_octomap)
    print(f'  씬 확인: 토마토 {n_tomato}개, octomap voxel {n_voxel}개 '
          + ('OK' if ok else '**의도와 다름**'))
    if not ok:
        raise SystemExit('씬이 의도한 상태가 아니다 — 이 상태로 잰 수치는 무의미하다')
    return n_tomato, n_voxel


def eval_combo(node, goals, repeat):
    """한 (씬, 시작 자세) 조합에서 모든 목표를 repeat회씩 플래닝한다."""
    per_target = []
    for d, best in goals:
        vals, j1s = [], []
        for _ in range(repeat):
            r = node.plan_to(best['align'], best['quat'])
            if r['ok']:
                vals.append(r['travel_deg'])
                j1s.append(r['j1_deg'])
        per_target.append({
            'class_name': d.get('class_name', '?'),
            'base_x': d['base_x'], 'base_y': d['base_y'], 'base_z': d['base_z'],
            'trials': repeat,
            'success': len(vals),
            # min: Phase 3(N개 계획 중 최소 선택)이 적용된 상태를 가정한 값.
            #      find_armed_pose_cartesian.py가 502도를 낸 것과 같은 집계라
            #      그 수치와 직접 비교할 수 있다.
            # median: Phase 3이 없는 현재 노드에 가까운 값.
            'travel_min': min(vals) if vals else None,
            'travel_median': st.median(vals) if vals else None,
            'j1_min': min(j1s) if j1s else None,
            'j1_median': st.median(j1s) if j1s else None,
        })
    return per_target


def summarize_combo(rows, repeat):
    trials = sum(r['trials'] for r in rows)
    ok = sum(r['success'] for r in rows)
    got = [r for r in rows if r['travel_min'] is not None]
    dead = [r for r in rows if r['success'] == 0]
    mins = [r['travel_min'] for r in got]
    meds = [r['travel_median'] for r in got]
    j1 = [r['j1_median'] for r in got]
    return {
        'success_pct': 100.0 * ok / trials if trials else 0.0,
        'reachable': len(got),
        'dead': len(dead),
        'dead_rows': dead,
        'median_of_min': st.median(mins) if mins else float('nan'),
        'worst_of_min': max(mins) if mins else float('nan'),
        'median_of_median': st.median(meds) if meds else float('nan'),
        'branch_a': sum(1 for v in j1 if v < BRANCH_A_MAX_J1_DEG),
        'branch_b': sum(1 for v in j1 if v >= BRANCH_A_MAX_J1_DEG),
    }


def check_start_pose_reachable(node, label, joints, from_joints):
    """시작 자세 자체가 이 씬에서 유효한가 — 이걸 먼저 봐야 한다.

    armed pose는 "대기 자세"라 사이클마다 반드시 거쳐 간다. 그 자세가 줄기
    voxel과 충돌하면 15개 목표의 이동량을 아무리 잘 재도 소용이 없다.
    문서 1.1절에서 1차 armed pose가 베드에 7.3cm까지 파고든 것을 FK로 뒤늦게
    발견했는데, 그때 없던 검사가 이것이다.
    """
    node.start_pose = list(from_joints)
    r = node.plan_to_config(list(joints))
    mark = 'OK' if r['ok'] else f"**실패: {r.get('error')}**"
    print(f'  {label} 도달 가능성: {mark}'
          + (f" (이동량 {r['travel_deg']:.0f}도)" if r['ok'] else ''))
    return r['ok']


def main():
    p = argparse.ArgumentParser(
        description='씬 구성 x 시작 자세 교차 비교 (octomap 재검증)')
    p.add_argument('--targets', default='bags/lab_bed_detections.json')
    p.add_argument('--octomap', default='bags/bed_look_octomap.bin',
                   help='주입할 octomap 파일(octomap_io.py capture 결과)')
    p.add_argument('--scenes', nargs='+', default=list(SCENES), choices=SCENES)
    p.add_argument('--poses', nargs='+', default=list(START_POSES),
                   choices=list(START_POSES))
    p.add_argument('--repeat', type=int, default=3,
                   help='목표당 반복. 3이면 find_armed_pose_cartesian.py가 '
                        '502도를 낸 것과 같은 집계가 되어 직접 비교된다')
    p.add_argument('--planning-time', type=float, default=2.0)
    p.add_argument('--attempts', type=int, default=10)
    # 함정 5 — 스윕 기본값 0.05는 노드(0.2)와 다르다. 여기서는 노드 값을 기본으로.
    p.add_argument('--orientation-tolerance', type=float, default=0.2)
    p.add_argument('--csv', default=None)
    args = p.parse_args()

    S.ORIENTATION_TOLERANCE_RAD = args.orientation_tolerance
    with open(args.targets, encoding='utf-8') as fh:
        dets = json.load(fh)['detections']

    need_octomap = any(s in ('octomap', 'full') for s in args.scenes)
    owp = octomap_io.load_octomap_file(args.octomap) if need_octomap else None
    if owp is not None:
        leaves = octomap_io.decode_msg(owp.octomap)
        print(f'\noctomap 파일: {args.octomap}')
        octomap_io.summarize(leaves, owp.octomap.resolution, label='  ')
        octomap_io.near_targets_report(leaves, dets)

    rclpy.init()
    node = PlanProbe(args.planning_time, args.attempts)
    if not node.wait_ready():
        raise SystemExit('move_group 없음')

    goals = []
    for d in dets:
        best, reason = select_approach(d['base_x'], d['base_y'], d['base_z'])
        if best is not None:
            goals.append((d, best))
    print(f'\n목표 {len(goals)}/{len(dets)}개가 기하 게이트 통과')
    print(f'조합 {len(args.scenes)} x {len(args.poses)} = '
          f'{len(args.scenes)*len(args.poses)}개, 각 목표 {args.repeat}회 반복')

    # 시작 상태를 **가정하지 않는다.** 앞선 세션이 남긴 토마토나 주입해 둔
    # octomap이 그대로 있을 수 있는데(실제로 밟았다), 그걸 empty로 착각하면
    # 함정 3 그 자체가 된다. 명시적으로 비우고 시작한다.
    print('\n씬 초기화(앞 세션 잔여물 제거)')
    scene_objects.publish_tomatoes(node, dets, remove=True)
    octomap_io.clear_octomap(node, rclpy)

    # octomap 재구성이 비싸므로 octomap 유무로 묶어 돈다.
    ordered = sorted(args.scenes, key=lambda s: s in ('octomap', 'full'))
    state = (False, False)          # (토마토 있음, octomap 있음)
    all_rows, summary = [], []
    t0 = time.time()

    for scene in ordered:
        state = apply_scene(node, dets, scene, owp, state)
        print(f'\n===== 씬: {scene} =====')
        verify_scene(node, dets, state[0], state[1])

        # armed pose가 이 씬에서 갈 수 있는 자세인지 먼저 본다.
        if 'armed' in args.poses:
            check_start_pose_reachable(node, 'look -> armed pose',
                                       START_POSES['armed'][1],
                                       START_POSES['look'][1])

        for pose_key in args.poses:
            pose_label, joints = START_POSES[pose_key]
            node.start_pose = list(joints)
            rows = eval_combo(node, goals, args.repeat)
            s = summarize_combo(rows, args.repeat)
            print(f'  [{pose_label:<10}] 성공 {s["success_pct"]:5.1f}%  '
                  f'도달 {s["reachable"]}/{len(goals)}  '
                  f'이동량 중앙 {s["median_of_min"]:.0f}도 최악 {s["worst_of_min"]:.0f}도  '
                  f'A분기 {s["branch_a"]} / B분기 {s["branch_b"]}')
            if s['dead']:
                for r in s['dead_rows']:
                    print(f'      전회 실패: {r["class_name"]} '
                          f'({r["base_x"]:+.3f}, {r["base_y"]:+.3f}, {r["base_z"]:+.3f})')
            for r in rows:
                all_rows.append({'scene': scene, 'pose': pose_key, **r})
            summary.append({'scene': scene, 'pose': pose_key,
                            **{k: v for k, v in s.items() if k != 'dead_rows'}})

    print(f'\n(총 {time.time()-t0:.0f}초)')

    # --- 요약표 ---
    print('\n=== 요약 ===')
    print(f'{"씬":<10}{"시작자세":<10}{"성공률":>8}{"도달":>7}'
          f'{"중앙(min)":>11}{"최악(min)":>11}{"중앙(median)":>13}{"A/B":>9}')
    for s in summary:
        print(f'{s["scene"]:<10}{s["pose"]:<10}{s["success_pct"]:7.1f}%'
              f'{s["reachable"]:5d}/{len(goals):<2d}'
              f'{s["median_of_min"]:11.0f}{s["worst_of_min"]:11.0f}'
              f'{s["median_of_median"]:13.0f}'
              f'{s["branch_a"]:5d}/{s["branch_b"]:<3d}')

    # 씬이 얼마나 낙관을 걷어냈는가 — 같은 시작 자세끼리 비교한다.
    by = {(s['scene'], s['pose']): s for s in summary}
    print('\n=== 씬이 수치를 얼마나 바꿨나 (같은 시작 자세끼리) ===')
    for pose_key in args.poses:
        base = by.get(('tomato', pose_key))
        full = by.get(('full', pose_key))
        if not base or not full:
            continue
        d_med = 100.0 * (full['median_of_min'] - base['median_of_min']) / base['median_of_min']
        print(f'  {START_POSES[pose_key][0]:<10} tomato -> full: '
              f'이동량 중앙 {base["median_of_min"]:.0f} -> {full["median_of_min"]:.0f}도 '
              f'({d_med:+.1f}%), '
              f'성공률 {base["success_pct"]:.1f}% -> {full["success_pct"]:.1f}%, '
              f'도달 {base["reachable"]} -> {full["reachable"]}개')

    if args.csv:
        with open(args.csv, 'w', newline='', encoding='utf-8') as fh:
            w = csv.DictWriter(fh, fieldnames=list(all_rows[0]))
            w.writeheader()
            w.writerows(all_rows)
        print(f'\nCSV 저장: {args.csv} ({len(all_rows)}행)')
        spath = args.csv.replace('.csv', '_summary.csv')
        with open(spath, 'w', newline='', encoding='utf-8') as fh:
            w = csv.DictWriter(fh, fieldnames=list(summary[0]))
            w.writeheader()
            w.writerows(summary)
        print(f'CSV 저장: {spath} ({len(summary)}행)')

    # 씬을 원래대로 되돌린다 — 다음 측정이 남은 토마토를 모르고 재면 함정 3이다.
    apply_scene(node, dets, 'empty', owp, state)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
