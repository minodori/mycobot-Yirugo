#!/usr/bin/env python3
"""수확통에 놓는 동작을 평가한다 — 통 넓이와 yaw 자유도가 얼마나 이득인가.

배경
----
`docs/new_concept.md`는 사이클을 `armed pos -> 타겟 -> armed pos -> 수확통 ->
armed pos`로 재구성한다. 그런데 수확통은 토마토와 성격이 다르다.

  토마토: 특정 열매 한 개를 정확히 물어야 한다 — 위치도 방향도 빡빡하다.
  수확통: 통(가로 8cm x 세로 8cm) **안 어디든** 떨어뜨리면 되고, 그리퍼가
          아래를 보기만 하면 **수직축 둘레 회전(yaw)은 완전히 자유**다.

두 번째가 특히 크다. 방향 제약 3축 중 하나를 통째로 푸는 것이라, 목표가
"자세 하나"에서 "자세의 1차원 다양체"가 된다. 플래너 입장에서는 목표 샘플링
성공률과 해의 품질이 모두 올라간다.

이 스크립트는 그 이득을 실측한다. 네 조건을 같은 armed pos에서 비교한다:
  point       위치 5mm 구 + 방향 3축 모두 0.2rad  (토마토와 같은 빡빡함)
  box         위치 8x8cm 상자 + 방향 3축 0.2rad
  yaw         위치 5mm 구 + yaw 자유
  box+yaw     위치 8x8cm 상자 + yaw 자유          <- 실제로 쓸 조건

주의
----
통 좌표(y, z)는 아직 확정 전이다(사용자가 계산으로 정하기로 함). 여기서는
후보를 여러 개 훑어 "어디에 놓는 게 좋은가"까지 같이 본다.

사용법
------
    python3 scripts/eval_bin_release.py --repeat 12
    python3 scripts/eval_bin_release.py --bin-y 0.20 0.25 0.28 --bin-z 0.05 0.10
"""

import argparse
import math
import statistics as st
import sys

sys.path.insert(0, __file__.rsplit('/', 1)[0])

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from sweep_targets_planned import PlanProbe
import sweep_targets_planned as S
from mycobot_280_pick import coord_to_goal_node as N

BIN_OBJECT_ID = 'harvest_bin'
BIN_WALL_M = 0.005

# 수확통 실측 (2026-08-01, 사용자 제공). g_base의 z=0이 곧 지면이다
# (world -> g_base가 identity임을 TF로 확인).
#   전체 높이   150 mm  -> 림(입구)이 z = 0.150
#   내부 깊이    50 mm  -> 안쪽 바닥이 z = 0.100
#   안쪽 넓이  80x80 mm
BIN_RIM_Z_M = 0.150
BIN_FLOOR_Z_M = 0.100
BIN_INNER_M = 0.08

# 가장자리에 떨어뜨리면 튕겨 나갈 수 있으므로 사방 1cm를 남긴다.
BIN_MARGIN_M = 0.01
# 상자의 z 두께 = 놓아도 되는 높이 범위. 림보다 위이기만 하면 되고 너무 높으면
# 토마토가 튄다 — 림 위 1~5cm를 허용 구간으로 본다.
BIN_DROP_Z_SPAN_M = 0.04
BIN_BOX_XYZ = [BIN_INNER_M - 2 * BIN_MARGIN_M,
               BIN_INNER_M - 2 * BIN_MARGIN_M,
               BIN_DROP_Z_SPAN_M]

# armed pos (scripts/find_armed_pose.py 산출, bags/armed_pose.json)
ARMED_POSE_DEG = [27.08, -2.39, -63.53, 1.70, -80.44, 64.83]

# 그리퍼 길이 — flange는 놓는 지점보다 이만큼 위에 있어야 한다.
GRIPPER_LEN_M = N.GRIPPER_LENGTH_OFFSET_M


def publish_bin_collision(node, y, remove=False):
    """수확통을 planning scene에 실제 형상으로 넣는다.

    이게 없으면 팔이 통을 관통하는 경로도 '성공'으로 집계된다 — 첫 측정의
    한계였다. 아래 5개 프리미티브로 근사한다:
      받침   바닥(z=0)부터 안쪽 바닥(z=0.100)까지 꽉 찬 상자
      벽 4개 안쪽 바닥부터 림(z=0.150)까지, 두께 5mm

    ACM(allowed collision matrix)은 **건드리지 않는다.** PlanningScene diff의
    ACM은 병합이 아니라 통째 대체라, 부분 발행하면 SRDF self-collision-disable이
    전부 소실되는 사고가 있었다(docs/obstacle_avoidance_manual_test.md).
    world.collision_objects만 넣는 것은 가산이라 안전하다.
    """
    obj = CollisionObject()
    obj.header.frame_id = N.BASE_LINK_NAME
    obj.id = BIN_OBJECT_ID
    obj.operation = CollisionObject.REMOVE if remove else CollisionObject.ADD

    if not remove:
        outer = BIN_INNER_M + 2 * BIN_WALL_M
        half = BIN_INNER_M / 2.0 + BIN_WALL_M / 2.0
        wall_h = BIN_RIM_Z_M - BIN_FLOOR_Z_M
        specs = [
            # (dx, dy, dz, x, y, z) — 모두 g_base 기준
            (outer, outer, BIN_FLOOR_Z_M, 0.0, y, BIN_FLOOR_Z_M / 2.0),
            (outer, BIN_WALL_M, wall_h, 0.0, y + half, BIN_FLOOR_Z_M + wall_h / 2.0),
            (outer, BIN_WALL_M, wall_h, 0.0, y - half, BIN_FLOOR_Z_M + wall_h / 2.0),
            (BIN_WALL_M, BIN_INNER_M, wall_h, +half, y, BIN_FLOOR_Z_M + wall_h / 2.0),
            (BIN_WALL_M, BIN_INNER_M, wall_h, -half, y, BIN_FLOOR_Z_M + wall_h / 2.0),
        ]
        for dx, dy, dz, px, py, pz in specs:
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [dx, dy, dz]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = px, py, pz
            pose.orientation.w = 1.0
            obj.primitives.append(prim)
            obj.primitive_poses.append(pose)

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [obj]
    node._scene_pub.publish(scene)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)


def run_rim_sweep(node, args, quat):
    """통 림 높이를 훑어 "낮추면 충돌이 주는가"를 답한다."""
    global BIN_RIM_Z_M, BIN_FLOOR_Z_M
    y = args.bin_y[0]
    depth = 0.05
    print(f'통 림 높이 스윕 — y={y:.2f}, 내부 깊이 {depth*100:.0f}cm 고정, '
          f'놓는 지점 = 림 + {args.drop_above*100:.0f}cm, 반복 {args.repeat}회')
    print(f'통 형상 {"포함" if args.with_bin_collision else "미포함"}\n')
    print(f'{"림 z":>7}{"바닥 z":>8}{"놓는 z":>8}'
          f'{"point 성공":>12}{"point 이동":>11}'
          f'{"box+yaw 성공":>14}{"box+yaw 이동":>13}')
    for rim in args.bin_rim:
        BIN_RIM_Z_M, BIN_FLOOR_Z_M = rim, rim - depth
        if args.with_bin_collision:
            publish_bin_collision(node, y)
        flange = (0.0, y, rim + args.drop_above + GRIPPER_LEN_M)
        out = []
        for box, yaw in ((None, False), (BIN_BOX_XYZ, True)):
            res = [node.plan_to(flange, quat, box_xyz=box, yaw_free=yaw)
                   for _ in range(args.repeat)]
            ok = [r for r in res if r['ok']]
            out.append((len(ok),
                        st.fmean([r['travel_deg'] for r in ok]) if ok else float('nan')))
        print(f'{rim:7.3f}{rim-depth:8.3f}{rim+args.drop_above:8.3f}'
              f'{out[0][0]:9d}/{args.repeat:<2d}{out[0][1]:11.0f}'
              f'{out[1][0]:11d}/{args.repeat:<2d}{out[1][1]:13.0f}')
    if args.with_bin_collision:
        publish_bin_collision(node, y, remove=True)


def downward_quat():
    """그리퍼 정면(로컬 +Z)이 아래(-Z)를 향하는 orientation.

    coord_to_goal_node의 look-at 헬퍼를 그대로 쓴다 — 같은 축 규약을 공유해야
    한다(그 파일 _compute_look_at_quat_xyzw 주석 참고).
    """
    return N._compute_look_at_quat_xyzw([0.0, 0.0, -1.0])


def main():
    p = argparse.ArgumentParser(description='수확통 놓기 동작 평가')
    p.add_argument('--repeat', type=int, default=12)
    p.add_argument('--bin-y', nargs='+', type=float, default=[0.15, 0.20, 0.25])
    # 놓는 지점(그리퍼 끝단)의 z. 림(0.150)보다 **위**여야 한다 — 그 아래면
    # 그리퍼가 통 벽에 부딪힌다. 림 위 1/3/5cm를 기본 후보로 둔다.
    p.add_argument('--bin-z', nargs='+', type=float,
                   default=[BIN_RIM_Z_M + 0.01,
                            BIN_RIM_Z_M + 0.03,
                            BIN_RIM_Z_M + 0.05])
    p.add_argument('--planning-time', type=float, default=2.0)
    p.add_argument('--attempts', type=int, default=10)
    # [2026-08-01] 통 높이를 낮추면 충돌이 주는가? 두 효과가 반대로 작용한다 —
    # 벽이 낮아져 걸릴 것이 줄지만, 놓는 지점이 낮아져 팔이 더 아래로 뻗어야
    # 한다. 내부 깊이(50mm)는 유지하고 받침 높이만 바꿔 림 위치를 훑는다.
    p.add_argument('--bin-rim', nargs='+', type=float, default=None,
                   metavar='Z',
                   help='통 림(입구) 높이를 여러 개 훑는다(m). 안쪽 바닥은 '
                        '항상 림-50mm. 놓는 지점은 림+--drop-above')
    p.add_argument('--drop-above', type=float, default=0.03,
                   help='놓는 지점을 림보다 이만큼 위에 둔다(m, 기본 3cm)')
    p.add_argument('--with-bin-collision', action='store_true',
                   help='수확통을 planning scene에 실제 형상으로 넣고 평가.\n'
                        '없으면 팔이 통을 관통하는 경로도 성공으로 집계된다')
    args = p.parse_args()

    S.ORIENTATION_TOLERANCE_RAD = 0.2
    quat = downward_quat()
    start = [math.radians(v) for v in ARMED_POSE_DEG]

    rclpy.init()
    node = PlanProbe(args.planning_time, args.attempts, start)
    node._scene_pub = node.create_publisher(PlanningScene, '/planning_scene', 10)
    if not node.wait_ready():
        raise SystemExit('move_group 없음')

    print(f'\narmed pos에서 수확통으로 — 반복 {args.repeat}회')
    print(f'통: 림 z={BIN_RIM_Z_M*100:.0f}cm, 바닥 z={BIN_FLOOR_Z_M*100:.0f}cm, '
          f'안쪽 {BIN_INNER_M*100:.0f}x{BIN_INNER_M*100:.0f}cm (g_base z=0이 지면)')
    print(f'사용 상자 {BIN_BOX_XYZ[0]*100:.0f}x{BIN_BOX_XYZ[1]*100:.0f}'
          f'x{BIN_BOX_XYZ[2]*100:.0f}cm (사방 {BIN_MARGIN_M*100:.0f}cm 여유)')
    print(f'그리퍼 정면이 아래를 향함 (끝단->flange {GRIPPER_LEN_M*100:.0f}cm 보정)')
    print(f'표의 z는 **그리퍼 끝단** 높이 — 림({BIN_RIM_Z_M*100:.0f}cm) 위여야 한다\n')

    conditions = [
        ('point',   None,        False),
        ('box',     BIN_BOX_XYZ, False),
        ('yaw',     None,        True),
        ('box+yaw', BIN_BOX_XYZ, True),
    ]

    if args.bin_rim:
        run_rim_sweep(node, args, quat)
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f'{"통 위치":>14}{"조건":>10}{"성공":>8}{"이동량":>9}{"SD":>7}{"J1 도달":>9}')
    rows = []
    for y in args.bin_y:
        if args.with_bin_collision:
            publish_bin_collision(node, y)
        for z in args.bin_z:
            flange = (0.0, y, z + GRIPPER_LEN_M)
            for name, box, yaw in conditions:
                res = [node.plan_to(flange, quat, box_xyz=box, yaw_free=yaw)
                       for _ in range(args.repeat)]
                ok = [r for r in res if r['ok']]
                if ok:
                    tr = [r['travel_deg'] for r in ok]
                    j1 = [r['j1_deg'] for r in ok]
                    print(f'{f"y={y:.2f} z={z:.2f}":>14}{name:>10}'
                          f'{len(ok):5d}/{args.repeat:<2d}{st.fmean(tr):9.0f}'
                          f'{st.pstdev(tr):7.0f}{st.fmean(j1):8.0f}°')
                    rows.append((y, z, name, len(ok), st.fmean(tr)))
                else:
                    print(f'{f"y={y:.2f} z={z:.2f}":>14}{name:>10}'
                          f'{0:5d}/{args.repeat:<2d}{"—":>9}{"—":>7}{"—":>9}')
                    rows.append((y, z, name, 0, float('nan')))
            print()

    print('조건별 종합 (모든 통 위치 합산):')
    for name, _, _ in conditions:
        sub = [r for r in rows if r[2] == name]
        tot = sum(r[3] for r in sub)
        n = len(sub) * args.repeat
        valid = [r[4] for r in sub if r[4] == r[4]]
        print(f'  {name:<9} 성공 {tot:3d}/{n:<3d} ({100*tot/n:5.1f}%)   '
              f'이동량 평균 {st.fmean(valid) if valid else float("nan"):.0f}°')

    if args.with_bin_collision:
        publish_bin_collision(node, args.bin_y[-1], remove=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
