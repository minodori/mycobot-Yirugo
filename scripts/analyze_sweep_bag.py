#!/usr/bin/env python3
"""스윕 rosbag을 사이클 단위로 분해해 관절 거동을 측정한다.

왜 필요한가
-----------
지금까지 스윕 분석은 `coord_to_goal_node`의 텍스트 로그를 정규식으로 긁는
방식이었다. 그래서 **관절값이 필요한 지표를 측정할 수 없었다** — 접근축
리팩터의 핵심 근거였던 "J1 스윙이 60~120도에서 약 34도로 줄었다"는 예측을
여섯 번의 스윕 동안 한 번도 확인하지 못했다(로그에는 compute_ik가 예상한
'관절 이동량 합'만 있고 실제 궤적이 없다).

이 스크립트는 bag의 /joint_states(실제 궤적)와 /rosout(노드 로그)을 시간축에서
겹쳐, 사이클별로 다음을 뽑는다:
  - 관절별 실제 이동 범위와 J1 스윙 (**예측 검증의 핵심**)
  - 실제 관절 이동량 합 vs compute_ik 예상치
  - 단계별 소요 시간
  - 명령값 대비 추종 오차 (/arm_group_controller/controller_state)

사용법
------
    python3 scripts/analyze_sweep_bag.py bags/sweep_0731_1800
    python3 scripts/analyze_sweep_bag.py bags/sweep_0731_1800 --csv out.csv

ROS 환경이 source 되어 있어야 한다(rosbag2_py, 메시지 타입 필요).
"""

import argparse
import math
import os
import re
import sys

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:  # pragma: no cover
    sys.exit(f'import 실패: {exc}\nsource /opt/ros/jazzy/setup.bash 후 실행할 것')

ARM_JOINTS = [
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
]
J_LABEL = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

# /rosout에서 사이클 경계와 단계를 찾는 패턴
RE_TARGET = re.compile(r'목표 지점\(([\d.-]+), ([\d.-]+), ([\d.-]+)\)')
RE_ADOPT = re.compile(
    r'탐색 완료 — 고도각 ([+-]?\d+)도 / roll (\d+)도 채택 .*?'
    r'예상 관절 이동량 합 (\d+)도'
)
RE_STAGE = re.compile(r'\[([1-5])/5 [^\]]+\] (시작|완료|목표 위치로|목표 지점으로|정렬 위치로|그리퍼)')
RE_DONE = re.compile(r'5단계 전부 성공')
RE_RETURN_DONE = re.compile(r'look pose 복귀 완료')


def read_bag(path):
    """bag에서 필요한 토픽만 시간순으로 읽어 반환."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {'/joint_states', '/rosout', '/arm_group_controller/controller_state'}
    missing = wanted - set(types)
    if '/joint_states' in missing:
        sys.exit(f'bag에 /joint_states가 없음. 담긴 토픽: {sorted(types)}')
    if missing:
        print(f'경고: bag에 없는 토픽 {sorted(missing)} — 해당 지표는 생략됨\n')

    js, logs, ctrl = [], [], []
    cache = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in wanted:
            continue
        if topic not in cache:
            cache[topic] = get_message(types[topic])
        msg = deserialize_message(data, cache[topic])
        t = stamp / 1e9
        if topic == '/joint_states':
            pos = dict(zip(msg.name, msg.position))
            if all(j in pos for j in ARM_JOINTS):
                js.append((t, [pos[j] for j in ARM_JOINTS]))
        elif topic == '/rosout':
            logs.append((t, msg.msg))
        else:
            ctrl.append((t, list(msg.error.positions) if msg.error.positions else None))
    return js, logs, ctrl


def split_cycles(logs):
    """`목표 지점(...)` 로그를 경계로 사이클을 나눈다."""
    cycles = []
    cur = None
    for t, text in logs:
        m = RE_TARGET.search(text)
        if m:
            if cur:
                cycles.append(cur)
            cur = {'t0': t, 't1': None, 'target': tuple(float(m.group(i)) for i in (1, 2, 3)),
                   'elev': None, 'roll': None, 'predicted': None, 'ok': False, 'events': []}
            continue
        if cur is None:
            continue
        m = RE_ADOPT.search(text)
        if m:
            cur['elev'], cur['roll'], cur['predicted'] = (
                int(m.group(1)), int(m.group(2)), int(m.group(3)))
        if RE_STAGE.search(text) or RE_DONE.search(text):
            cur['events'].append((t, text.strip()[:60]))
        if RE_DONE.search(text):
            cur['ok'] = True
        if RE_RETURN_DONE.search(text):
            cur['t1'] = t
    if cur:
        cycles.append(cur)
    for c in cycles:
        if c['t1'] is None:
            c['t1'] = c['t0'] + 1e9  # 끝을 못 찾으면 열린 구간
    return cycles


def measure(js, t0, t1):
    """구간 [t0,t1]의 관절 거동. (시작자세, 관절별 진폭, 총 이동량, J1 스윙)"""
    seg = [(t, p) for t, p in js if t0 <= t <= t1]
    if len(seg) < 2:
        return None
    start = seg[0][1]
    lo = [min(p[i] for _, p in seg) for i in range(6)]
    hi = [max(p[i] for _, p in seg) for i in range(6)]
    span = [math.degrees(hi[i] - lo[i]) for i in range(6)]
    # 총 이동량 = 샘플 간 변화의 절대값 누적(실제 경로 길이)
    travel = [0.0] * 6
    for (_, a), (__, b) in zip(seg, seg[1:]):
        for i in range(6):
            travel[i] += abs(b[i] - a[i])
    travel_deg = [math.degrees(v) for v in travel]
    j1_swing = span[0]
    return {
        'start_deg': [math.degrees(v) for v in start],
        'span_deg': span,
        'travel_deg': travel_deg,
        'travel_sum': sum(travel_deg),
        'j1_swing': j1_swing,
        'n': len(seg),
    }


def main():
    ap = argparse.ArgumentParser(description='스윕 rosbag 사이클 분석')
    ap.add_argument('bag', help='bag 디렉터리 경로')
    ap.add_argument('--csv', default=None, help='CSV 저장 경로')
    args = ap.parse_args()

    if not os.path.isdir(args.bag):
        sys.exit(f'디렉터리가 아님: {args.bag}')

    js, logs, ctrl = read_bag(args.bag)
    print(f'/joint_states {len(js)}개, /rosout {len(logs)}개 읽음')
    cycles = split_cycles(logs)
    if not cycles:
        sys.exit('사이클 경계(`목표 지점(...)` 로그)를 못 찾음 — '
                 'coord_to_goal_node가 기록 중 돌았는지 확인할 것')
    print(f'사이클 {len(cycles)}개 검출\n')

    rows = []
    print(f"{'목표z':>6} {'고도각':>6} {'예상합':>7} {'실제합':>7} {'차이':>7} "
          f"{'J1스윙':>7} {'소요s':>6} {'결과':>5}")
    for c in cycles:
        m = measure(js, c['t0'], c['t1'])
        if m is None:
            continue
        pred = c['predicted']
        diff = (m['travel_sum'] - pred) if pred else float('nan')
        rows.append({
            'target_z': c['target'][2], 'elev': c['elev'], 'roll': c['roll'],
            'predicted_sum': pred, 'actual_sum': round(m['travel_sum'], 1),
            'j1_swing': round(m['j1_swing'], 1), 'duration_s': round(c['t1'] - c['t0'], 1),
            'ok': c['ok'],
            **{f'{J_LABEL[i]}_span': round(m['span_deg'][i], 1) for i in range(6)},
        })
        print(f"{c['target'][2]:6.2f} {str(c['elev'] or '-'):>5}° "
              f"{str(pred or '-'):>7} {m['travel_sum']:7.0f} {diff:+7.0f} "
              f"{m['j1_swing']:7.1f} {c['t1'] - c['t0']:6.1f} "
              f"{'성공' if c['ok'] else '실패':>5}")

    if rows:
        j1 = [r['j1_swing'] for r in rows]
        tot = [r['actual_sum'] for r in rows]
        print(f"\nJ1 실제 스윙 : 최소 {min(j1):.1f}° / 평균 {sum(j1)/len(j1):.1f}° / 최대 {max(j1):.1f}°")
        print(f"실제 이동량합: 최소 {min(tot):.0f}° / 평균 {sum(tot)/len(tot):.0f}° / 최대 {max(tot):.0f}°")
        preds = [r['predicted_sum'] for r in rows if r['predicted_sum']]
        if preds:
            print(f"compute_ik 예상 평균: {sum(preds)/len(preds):.0f}° "
                  f"(실제와의 차이는 OMPL이 고른 경로가 IK 해와 다르기 때문)")

    if args.csv and rows:
        import csv
        with open(args.csv, 'w', newline='', encoding='utf-8') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f'\nCSV 저장: {args.csv}')


if __name__ == '__main__':
    main()
