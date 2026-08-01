#!/usr/bin/env python3
"""토마토 베드 전체를 격자로 훑어 접근 가능성 지도를 만드는 오프라인 도구.

왜 오프라인인가
---------------
RViz FakeSystem으로 목표를 하나씩 발행하는 방식은 사이클당 약 30초가 걸린다.
그런데 로그로 측정해보면 그 30초 중 **판단 로직(접근축 탐색)은 0.3초(1%)**뿐이고
나머지 99%는 궤적 실행과 고정 대기다:

    복귀 12.6s(42%) / 정렬 6.1s(20%) / 그리퍼 개폐 5.2s(18%) /
    파지 2.6s / 접근 2.2s / 후퇴 1.9s / octomap 대기 1.5s / 접근축 탐색 0.3s

FakeSystem(mock_components/GenericSystem)이 궤적을 **실시간으로** 실행하기
때문이며, RViz를 꺼도 벽시계 시간은 줄지 않는다. 즉 대량 데이터가 필요하면
"시각화를 포기"하는 게 아니라 **"실행을 포기"**해야 한다.

이 스크립트는 coord_to_goal_node가 목표를 받았을 때 수행하는 판단 중 **기하
부분만** 그대로 재현한다 — 게이트 통과 여부, 접근축 후보 생성, 채택될 고도각,
정렬 위치 반지름, 예상 J1 스윙, 수직 우회량. ROS를 띄우지 않으므로 점당
1ms 미만이고, 2000점 격자가 1초에 끝난다.

무엇을 재현하고 무엇을 재현하지 않는가
--------------------------------------
재현함 (노드와 **같은 상수·같은 함수**를 import해서 씀 — 값이 어긋날 수 없음):
  - 게이트 1·2: 목표 반지름/방위각
  - 게이트 3: 정렬 반지름 >= MIN_ALIGN_RADIUS_M
  - 게이트 4·5: flange/정렬 위치의 어깨 기준 도달 한계
  - 접근축 후보 생성과 정렬 순서(이상 고도각에 가까운 순)
  - 예상 최소 J1 스윙 = asin(손목 측면 오프셋 / 정렬 반지름) — 400만 샘플로
    검증된 관계식(docs/VISUALIZATION_HANDOFF.md 1절)

재현하지 않음 (이것들이 필요하면 실제 스택이 필요함):
  - compute_ik 성공 여부 (KDL 수치해석 IK, 확률적)
  - OMPL 플래닝 성공 여부
  - Cartesian 경로 fraction
  - 충돌 검사 / Octomap

그래서 이 스크립트의 결과는 **"기하적으로 가능한가"**의 상한이다. 여기서
탈락한 점은 실제로도 확실히 실패하고, 여기서 통과한 점은 "IK가 풀리면 성공"
후보다. 실측 스윕(z=0.40~0.14, 27점)에서 이 기준을 통과한 점은 27/27 성공했다.

사용법
------
    python3 scripts/sweep_bed_offline.py                    # 기본 격자
    python3 scripts/sweep_bed_offline.py --step 0.01        # 1cm 간격
    python3 scripts/sweep_bed_offline.py --csv bed.csv      # CSV 저장
    python3 scripts/sweep_bed_offline.py --x 0.20 0.30 --y -0.10 0.10 --z 0.15 0.35

ROS 환경이 source 되어 있어야 한다(geometry_msgs import 때문). move_group이나
RViz는 필요 없다.
"""

import argparse
import math
import os
import sys

# 노드 모듈을 그대로 import해서 상수/헬퍼를 공유한다. 값을 복사해오면 언젠가
# 어긋나므로, 반드시 import로 묶어둘 것.
_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
for _p in ('src/mycobot_280_pick', 'src/pymoveit2'):
    sys.path.insert(0, os.path.join(_ROOT, _p))
try:
    from geometry_msgs.msg import Point
    from mycobot_280_pick import coord_to_goal_node as N
except ImportError as exc:  # pragma: no cover
    sys.exit(
        f'import 실패: {exc}\n'
        'ROS 환경을 source 했는지 확인할 것:\n'
        '  source /opt/ros/jazzy/setup.bash\n'
        '(pymoveit2는 이 저장소의 src/pymoveit2를 쓰므로 별도 설치 불필요)'
    )

WRIST_LATERAL_OFFSET_M = 0.0732  # URDF joint6_to_joint5 origin (0, -0.07318, 0)
SHOULDER = [0.0, 0.0, N.SHOULDER_HEIGHT_M]

# 탈락 사유 코드 — 격자 지도에 한 글자로 찍는다.
REASONS = {
    'ok': '.',
    'radius': 'R',      # 게이트 1: 목표 반지름 밖
    'azimuth': 'A',     # 게이트 2: 방위각 밖
    'nocand': 'X',      # 게이트 3~5: 어떤 접근축도 통과 못 함
}


def evaluate(x, y, z):
    """목표 하나를 노드와 같은 기준으로 평가. dict 반환."""
    r = math.hypot(x, y)
    az_deg = math.degrees(math.atan2(y, x))

    if not (N.MIN_TARGET_RADIUS_M <= r <= N.MAX_TARGET_RADIUS_M):
        return {'reason': 'radius', 'radius': r, 'azimuth_deg': az_deg}
    if abs(az_deg) > N.MAX_TARGET_AZIMUTH_DEG:
        return {'reason': 'azimuth', 'radius': r, 'azimuth_deg': az_deg}

    target = Point()
    target.x, target.y, target.z = x, y, z
    azimuth_rad = math.atan2(y, x)

    # 이상 고도각 = 고정 기준점(look pose flange) -> 목표 방향
    d = [t - p for t, p in zip((x, y, z), N.APPROACH_REFERENCE_POINT)]
    ideal = math.atan2(d[2], math.hypot(d[0], d[1]))

    cands = []
    for elev in N.APPROACH_ELEVATION_CANDIDATES_RAD:
        fwd = N._forward_unit_vector(azimuth_rad, elev)
        flange, align = N._waypoints_along_forward(target, fwd)
        align_r = math.hypot(align[0], align[1])
        if align_r < N.MIN_ALIGN_RADIUS_M:
            continue
        if math.dist(flange, SHOULDER) > N.MAX_SHOULDER_DISTANCE_M:
            continue
        align_d = math.dist(align, SHOULDER)
        if align_d > N.MAX_ALIGN_SHOULDER_DISTANCE_M:
            continue
        cands.append({
            'elev_rad': elev,
            'err': abs(elev - ideal),
            'align_radius': align_r,
            'align_shoulder': align_d,
            'detour_m': abs(align[2] - z),
        })

    if not cands:
        return {'reason': 'nocand', 'radius': r, 'azimuth_deg': az_deg,
                'ideal_deg': math.degrees(ideal)}

    cands.sort(key=lambda c: c['err'])
    best = cands[0]
    # 예상 최소 J1 스윙 — 손목 측면 오프셋 때문에 강제되는 베이스 회전
    ratio = min(1.0, WRIST_LATERAL_OFFSET_M / best['align_radius'])
    return {
        'reason': 'ok',
        'radius': r,
        'azimuth_deg': az_deg,
        'ideal_deg': math.degrees(ideal),
        'elev_deg': math.degrees(best['elev_rad']),
        'n_candidates': len(cands),
        'align_radius': best['align_radius'],
        'align_shoulder': best['align_shoulder'],
        'detour_mm': best['detour_m'] * 1000.0,
        'min_j1_deg': math.degrees(math.asin(ratio)),
    }


def frange(lo, hi, step):
    n = int(round((hi - lo) / step)) + 1
    return [round(lo + i * step, 4) for i in range(n)]


def main():
    p = argparse.ArgumentParser(description='토마토 베드 접근 가능성 오프라인 스윕')
    p.add_argument('--x', nargs=2, type=float, default=[0.15, 0.32], metavar=('MIN', 'MAX'))
    p.add_argument('--y', nargs=2, type=float, default=[-0.16, 0.16], metavar=('MIN', 'MAX'))
    p.add_argument('--z', nargs=2, type=float, default=[0.14, 0.40], metavar=('MIN', 'MAX'))
    p.add_argument('--step', type=float, default=0.02, help='격자 간격(m), 기본 0.02')
    p.add_argument('--csv', type=str, default=None, help='CSV 저장 경로')
    p.add_argument('--slice-z', type=float, default=None,
                   help='이 높이의 xy 단면 지도를 출력(기본: 중간 높이)')
    args = p.parse_args()

    xs, ys, zs = (frange(*args.x, args.step), frange(*args.y, args.step),
                  frange(*args.z, args.step))
    print(f'격자: x {len(xs)} × y {len(ys)} × z {len(zs)} = {len(xs)*len(ys)*len(zs)}점 '
          f'(간격 {args.step*100:.0f}cm)')
    print(f'게이트: 반지름 {N.MIN_TARGET_RADIUS_M}~{N.MAX_TARGET_RADIUS_M}m, '
          f'방위각 ±{N.MAX_TARGET_AZIMUTH_DEG}°, 정렬반지름 ≥{N.MIN_ALIGN_RADIUS_M}m, '
          f'flange ≤{N.MAX_SHOULDER_DISTANCE_M}m, 정렬 ≤{N.MAX_ALIGN_SHOULDER_DISTANCE_M}m\n')

    results = []
    for z in zs:
        for y in ys:
            for x in xs:
                res = evaluate(x, y, z)
                res.update(x=x, y=y, z=z)
                results.append(res)

    ok = [r for r in results if r['reason'] == 'ok']
    print(f'통과 {len(ok)} / {len(results)} ({100*len(ok)/len(results):.1f}%)')
    from collections import Counter
    for reason, cnt in Counter(r['reason'] for r in results).most_common():
        if reason == 'ok':
            continue
        label = {'radius': '목표 반지름 밖', 'azimuth': '방위각 밖',
                 'nocand': '접근축 없음(사각지대/도달한계)'}[reason]
        print(f'  탈락 {REASONS[reason]} {label}: {cnt}개')

    if ok:
        def stat(key, unit='', f='{:.1f}'):
            v = [r[key] for r in ok]
            return (f'{f.format(min(v))}~{f.format(max(v))}{unit} '
                    f'(평균 {f.format(sum(v)/len(v))}{unit})')
        print('\n통과 점들의 특성:')
        print(f'  채택 고도각      {stat("elev_deg", "°")}')
        print(f'  수직 우회량      {stat("detour_mm", "mm", "{:.0f}")}')
        print(f'  정렬 반지름      {stat("align_radius", "m", "{:.3f}")}')
        print(f'  예상 최소 J1 스윙 {stat("min_j1_deg", "°")}')
        print(f'  후보 개수        {stat("n_candidates", "개", "{:.0f}")}')

    # xy 단면 지도
    slice_z = args.slice_z if args.slice_z is not None else zs[len(zs) // 2]
    slice_z = min(zs, key=lambda v: abs(v - slice_z))
    print(f'\nz = {slice_z:.2f} m 단면 (행=y, 열=x, 로봇은 원점에서 +x를 봄)')
    print('  기호: . 통과   R 반지름밖   A 방위각밖   X 접근축없음\n')
    header = '        ' + ''.join(f'{x*100:5.0f}' for x in xs)
    print(header)
    print('        ' + '  x(cm)→'.rjust(len(header) - 8))
    for y in reversed(ys):
        row = ''.join(
            f'{REASONS[next(r["reason"] for r in results if r["x"]==x and r["y"]==y and r["z"]==slice_z)]:>5}'
            for x in xs
        )
        print(f'y={y*100:+5.0f}cm{row}')

    if args.csv:
        import csv
        keys = ['x', 'y', 'z', 'reason', 'radius', 'azimuth_deg', 'ideal_deg', 'elev_deg',
                'n_candidates', 'align_radius', 'align_shoulder', 'detour_mm', 'min_j1_deg']
        with open(args.csv, 'w', newline='', encoding='utf-8') as fh:
            w = csv.DictWriter(fh, fieldnames=keys, extrasaction='ignore')
            w.writeheader()
            w.writerows(results)
        print(f'\nCSV 저장: {args.csv} ({len(results)}행)')


if __name__ == '__main__':
    main()
