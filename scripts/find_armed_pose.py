#!/usr/bin/env python3
"""수집된 정렬 관절해로부터 armed pos 후보를 도출한다.

docs/new_concept.md §10 — "각 관절의 이동이 최소이거나 관절이 많이 꼬이는 경로를
배제한 최적 경로를 선택한다. **그 경로의 시작점이 armed pos로 정의한다**".

입력: scripts/sweep_targets_planned.py --collect-solutions 의 출력 JSON
      (목표별 정렬 관절해, A분기 우선)

왜 격자 스윕이 아닌가
---------------------
"J1을 -10~+50도 훑는다" 같은 접근은 look pose의 J1(-7.5도)을 기준점처럼 취급하는
것인데, 그 값은 기구학적 의미가 없다 — docs/look_pose.md:4가 "실물 팔이 릴리즈
상태에서 **손으로 맞춘 뒤** get_angles()로 읽어 확정"이라고 명시한다. 선정 기준은
카메라 시야와 충돌 회피뿐이었다. J2~J6도 같은 이유로 임의값이므로, "J1만 훑고
나머지는 look pose 고정"은 임의값 위에 임의값을 얹는 것이다.

대신 **실제로 가야 할 자세들의 관절공간 중심**을 구한다. 6차원 문제이지 1차원
스윕이 아니다.

두 가지 중심
------------
  centroid (평균)  — 총 이동량을 줄인다. 이상치에 끌려간다
  minimax          — **최악의 목표**까지의 이동량을 최소화한다. 사이클 시간의
                     하한이 아니라 상한을 잡아주므로 이쪽이 실무적으로 낫다

관절별 가중치가 필요하다. J1(베이스)은 팔 전체를 돌리므로 같은 1도라도 J5/J6보다
훨씬 비싸다 — 실물에서 "몸을 틀면서 이동한다"로 관측된 것이 J1 스윙이다
(docs/VISUALIZATION_HANDOFF.md:240). 기본 가중치는 그 관측을 반영한다.

사용법
------
    python3 scripts/find_armed_pose.py bags/align_solutions.json
    python3 scripts/find_armed_pose.py bags/align_solutions.json --bin-y 0.25 --bin-z 0.10
"""

import argparse
import json
import math

# 관절 이동 비용 가중치. J1은 팔 전체 관성을 돌리므로 가장 비싸고, 손목으로 갈수록
# 싸다. 정확한 물리값이 아니라 "베이스 회전을 우선 줄인다"는 정책의 표현이다.
DEFAULT_WEIGHTS = [3.0, 2.0, 1.5, 1.0, 1.0, 0.5]

# URDF mycobot_280_m5 관절 한계(rad). armed pos 후보가 한계에 붙지 않는지 확인용.
JOINT_LIMITS_RAD = [
    (-2.9321, 2.9321),   # joint2_to_joint1  (J1, ±168도)
    (-2.3562, 2.3562),   # joint3_to_joint2
    (-2.4435, 2.4435),   # joint4_to_joint3
    (-2.9321, 2.9321),   # joint5_to_joint4
    (-2.9321, 2.9321),   # joint6_to_joint5
    (-3.0543, 3.0543),   # joint6output_to_joint6
]

# J5가 0 근처면 J4/J6가 수십~백 도 튄다(coord_to_goal_node WRIST_SINGULARITY_MARGIN_RAD).
# armed pos 자체가 그 근처면 거기서 출발하는 모든 경로가 불안정해진다.
WRIST_SINGULARITY_MARGIN_RAD = math.radians(8.0)
WRIST_PITCH_INDEX = 4


def weighted_travel(a, b, weights):
    """두 관절 구성 사이의 가중 이동량(도). 모든 축이 limited revolute이므로
    wrap 없이 단순 차이를 쓴다 — J1 한계가 ±168도라 180도를 넘는 실제 이동이
    불가능하고, wrap을 쓰면 오히려 도달 불가능한 경로를 '가깝다'고 오판한다."""
    return sum(w * abs(x - y) for x, y, w in zip(a, b, weights)) * 180.0 / math.pi


def evaluate(candidate, solutions, weights):
    d = [weighted_travel(candidate, s['joints'], weights) for s in solutions]
    return {'max': max(d), 'sum': sum(d), 'mean': sum(d) / len(d), 'each': d}


def refine_minimax(solutions, weights, seed, iterations=2000, step0=0.20):
    """좌표 하강으로 최악 이동량을 줄인다. 6차원에 표본 15개뿐이라 정교한
    최적화가 필요 없고, 재현 가능해야 하므로 난수를 쓰지 않는다."""
    best = list(seed)
    best_score = evaluate(best, solutions, weights)['max']
    step = step0
    for _ in range(iterations):
        improved = False
        for axis in range(6):
            for delta in (step, -step):
                trial = list(best)
                trial[axis] += delta
                lo, hi = JOINT_LIMITS_RAD[axis]
                if not (lo < trial[axis] < hi):
                    continue
                score = evaluate(trial, solutions, weights)['max']
                if score < best_score - 1e-9:
                    best, best_score, improved = trial, score, True
        if not improved:
            step *= 0.5
            if step < 1e-4:
                break
    return best, best_score


def describe(name, cand, solutions, weights, look_pose):
    ev = evaluate(cand, solutions, weights)
    deg = [math.degrees(v) for v in cand]
    print(f'\n[{name}]')
    print('  관절(도)  ' + '  '.join(f'{v:+7.2f}' for v in deg))
    print(f'  가중 이동량   최대 {ev["max"]:7.1f}   평균 {ev["mean"]:7.1f}   합 {ev["sum"]:8.1f}')
    j5 = cand[WRIST_PITCH_INDEX]
    if abs(j5) < WRIST_SINGULARITY_MARGIN_RAD:
        print(f'  ⚠ J5 = {math.degrees(j5):+.1f}° — 손목 특이점 여유 '
              f'{math.degrees(WRIST_SINGULARITY_MARGIN_RAD):.0f}° 안쪽')
    tight = [(i, math.degrees(min(cand[i] - JOINT_LIMITS_RAD[i][0],
                                  JOINT_LIMITS_RAD[i][1] - cand[i])))
             for i in range(6)]
    worst = min(tight, key=lambda t: t[1])
    print(f'  관절 한계 여유 최소  J{worst[0]+1} {worst[1]:.0f}°')
    if look_pose:
        print(f'  look pose 대비 개선  최대 '
              f'{100*(1 - ev["max"]/evaluate(look_pose, solutions, weights)["max"]):+.0f}%')
    return ev


def main():
    p = argparse.ArgumentParser(description='정렬 관절해로부터 armed pos 도출')
    p.add_argument('solutions', help='--collect-solutions 출력 JSON')
    p.add_argument('--weights', nargs=6, type=float, default=DEFAULT_WEIGHTS)
    p.add_argument('--json-out', default=None, help='선정 결과 저장 경로')
    args = p.parse_args()

    with open(args.solutions, encoding='utf-8') as fh:
        payload = json.load(fh)
    sols = payload['solutions']
    names = payload['joint_names']
    look = payload.get('start_pose')
    w = args.weights

    print(f'입력 {len(sols)}개 정렬 관절해 ({args.solutions})')
    print(f'관절 순서: {", ".join(names)}')
    print(f'가중치    : {w}')
    branches = {}
    for s in sols:
        branches[s['branch']] = branches.get(s['branch'], 0) + 1
    print(f'분기      : ' + ', '.join(f'{k} {v}개' for k, v in sorted(branches.items())))

    j1 = sorted(math.degrees(s['joints'][0]) for s in sols)
    print(f'정렬 J1 분포: {j1[0]:+.1f}° ~ {j1[-1]:+.1f}°  (중앙값 {j1[len(j1)//2]:+.1f}°)')

    if look:
        describe('현재 look pose (기준선)', look, sols, w, None)

    centroid = [sum(s['joints'][i] for s in sols) / len(sols) for i in range(6)]
    describe('centroid (관절별 평균)', centroid, sols, w, look)

    # medoid — 실제 정렬 자세 중 하나. 반드시 도달 가능하다는 장점이 있다.
    medoid = min(sols, key=lambda s: evaluate(s['joints'], sols, w)['max'])
    describe(f'medoid (실제 정렬 자세 중 최적, z={medoid["base_z"]:.3f})',
             medoid['joints'], sols, w, look)

    best, _ = refine_minimax(sols, w, centroid)
    ev = describe('minimax (좌표하강 최적화) ← 권장', best, sols, w, look)

    print('\n목표별 가중 이동량 (minimax 기준):')
    for s, d in sorted(zip(sols, ev['each']), key=lambda t: -t[1]):
        print(f'  {s["class_name"]:<8} z={s["base_z"]:.3f}  '
              f'J1 {math.degrees(s["joints"][0]):+6.1f}°  이동량 {d:6.1f}  [{s["branch"]}]')

    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            json.dump({
                'joint_names': names,
                'armed_pose_rad': [round(v, 6) for v in best],
                'armed_pose_deg': [round(math.degrees(v), 3) for v in best],
                'weights': w,
                'max_weighted_travel_deg': round(ev['max'], 1),
                'source': args.solutions,
                'n_solutions': len(sols),
            }, fh, indent=2, ensure_ascii=False)
        print(f'\n저장: {args.json_out}')


if __name__ == '__main__':
    main()
