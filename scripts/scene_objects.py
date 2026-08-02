#!/usr/bin/env python3
"""planning scene에 토마토를 넣고 ACM을 고치는 공용 헬퍼.

원래 `find_armed_pose_cartesian.py` 안에 있던 것을 빼냈다. octomap 재검증
(`eval_bed_scene.py`)도 **똑같은 씬**을 만들어야 하는데, 두 벌로 관리하면
"토마토 여유 1cm"나 "ACM 허용 링크 10개" 같은 값이 갈라져서 두 측정이 비교
불가능해진다. 씬을 만드는 코드가 하나여야 수치가 비교된다.

여기 담긴 판단 둘은 실측에서 나온 것이라 함부로 바꾸면 안 된다
(`docs/ARMED_POSE_HANDOFF.md` 2절 함정 4):

  * 토마토를 씬에 넣으면 **ACM이 필수**다. 안 넣으면 접근할 토마토 자신과
    충돌해 플래닝 성공률이 2~4%로 떨어진다.
  * **ACM diff는 병합이 아니라 통째 대체다.** 부분 발행하면 SRDF의
    self-collision-disable이 전부 소실된다 — 반드시 조회 → 보존 → 추가 → 재발행.
"""

import functools
import math
import os
import sys
import time

print = functools.partial(print, flush=True)

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
for _p in ('src/mycobot_280_pick', 'src/pymoveit2'):
    sys.path.insert(0, os.path.join(_ROOT, _p))

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive

from mycobot_280_pick import coord_to_goal_node as N

TOMATO_OBJECT_PREFIX = 'tomato_'
# 검출 반지름에 더할 여유(m). YOLO 추정 반지름은 13~18mm인데, depth 오차와
# 열매가 흔들릴 여지를 감안해 키운다. 너무 키우면 접근 자체가 막히므로
# 정렬 standoff(2cm)보다는 작게 둔다.
TOMATO_MARGIN_M = 0.010


def scene_publisher(node, wait_sec=5.0):
    """PlanningScene 발행자를 노드에 한 번만 만든다.

    DDS 디스커버리가 붙기 전에 쏘면 **조용히 사라진다.** 예전엔 만든 뒤 1초
    돌고 말았는데, 갓 만든 노드에서 바로 쓰면 그 사이에 move_group이 못 붙어
    발행이 통째로 증발했다(2026-08-02, 장애물 collision object를 넣었는데
    씬에 안 남는 것으로 드러남). 그래서 **구독자가 붙을 때까지** 기다린다.
    """
    if not hasattr(node, '_scene_pub'):
        node._scene_pub = node.create_publisher(PlanningScene, '/planning_scene', 10)
        deadline = time.time() + wait_sec
        while (node._scene_pub.get_subscription_count() == 0
               and time.time() < deadline):
            rclpy.spin_once(node, timeout_sec=0.05)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
    return node._scene_pub


OBSTACLE_OBJECT_ID = 'demo_obstacle'


def publish_obstacle(node, xyz_min, xyz_max, remove=False):
    """[2026-08-02] 데모 장애물을 **collision object**로 넣고 뺀다.

    octomap voxel로 넣는 길(octomap_io.py obstacle)과 목적은 같은데 **실물에서는
    이쪽만 살아남는다.** octomap 쪽이 실물에서 무효인 이유가 둘이다:

      1. D435 클라우드가 occupancy_map_monitor를 통해 octomap을 계속 갱신한다 —
         주입한 voxel을 덮어쓴다.
      2. coord_to_goal_node가 목표마다 `/clear_octomap`을 부른다(문서 5.6b) —
         지워진다.

    collision object는 둘 중 어느 것에도 안 지워진다. 대신 화면에서는 octomap
    voxel이 아니라 초록 상자로 보인다(PlanningScene의 Scene Geometry).

    ACM은 건드리지 않는다 — **모든 링크가 이 장애물을 피해야** 데모가 성립한다.
    (`--octomap-acm`이 그리퍼·손목을 통과시키는 것과 대비된다.)
    """
    pub = scene_publisher(node)
    scene = PlanningScene()
    scene.is_diff = True
    obj = CollisionObject()
    obj.header.frame_id = N.BASE_LINK_NAME
    obj.id = OBSTACLE_OBJECT_ID
    obj.operation = CollisionObject.REMOVE if remove else CollisionObject.ADD
    if not remove:
        size = [hi - lo for lo, hi in zip(xyz_min, xyz_max)]
        center = [(lo + hi) / 2.0 for lo, hi in zip(xyz_min, xyz_max)]
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(v) for v in size]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        pose.orientation.w = 1.0
        obj.primitives = [prim]
        obj.primitive_poses = [pose]
    scene.world.collision_objects.append(obj)
    pub.publish(scene)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)


def publish_tomatoes(node, dets, remove=False):
    """토마토를 구 collision object로 씬에 넣는다. ACM은 건드리지 않는다."""
    pub = scene_publisher(node)
    scene = PlanningScene()
    scene.is_diff = True
    for i, d in enumerate(dets):
        obj = CollisionObject()
        obj.header.frame_id = N.BASE_LINK_NAME
        obj.id = f'{TOMATO_OBJECT_PREFIX}{i}'
        obj.operation = CollisionObject.REMOVE if remove else CollisionObject.ADD
        if not remove:
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.SPHERE
            prim.dimensions = [float(d.get('radius_m', 0.017)) + TOMATO_MARGIN_M]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (
                d['base_x'], d['base_y'], d['base_z'])
            pose.orientation.w = 1.0
            obj.primitives = [prim]
            obj.primitive_poses = [pose]
        scene.world.collision_objects.append(obj)
    pub.publish(scene)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)


def allow_gripper_tomato_collisions(node, dets, only=None, quiet=False):
    """그리퍼·손목·flange가 토마토와 충돌해도 되게 ACM을 고친다.

    이걸 안 하면 접근할 토마토 자신과 충돌해 플래닝이 거의 전부 실패한다 —
    실측 성공률 2~4%였다. coord_to_goal_node가 목표 지점의 target_object에
    대해 똑같은 처리를 한다(_allow_gripper_target_object_collision, L1352):
    "그 자리에 등록된 구가 flange 자신과 충돌로 잡혀서 Unable to sample any
    valid states for goal tree로 매번 실패"했다는 실측 기록이 있고, joint5/
    joint6도 같은 이유로 허용 목록에 있다.

    **`only`가 이 함수의 핵심 인자다** (2026-08-02 추가).

      only=None  열매 **전부**를 완화한다. 그러면 그리퍼가 **옆 토마토를
                 통과하는 경로도 성공으로 잡힌다** — 문서의 열매 포함 수치가
                 전부 이 조건이고, 그만큼 낙관적이다(함정 17).
      only=i     i번 열매만 완화하고 나머지는 장애물로 남긴다. "접근할 열매
                 자신과의 충돌"만 풀어주는, 원래 의도에 맞는 조건이다.

    노드는 이웃 열매를 씬에 **등록하지 않으므로** only=None 쪽에 가깝다.
    즉 only=i는 "노드가 지금 하는 일"이 아니라 **"이웃을 피하려면 무엇을
    치러야 하는가"**를 재는 조건이다.

    주의: 어느 쪽이든 팔뚝(joint2~4)은 여전히 충돌로 잡히므로, 팔이 베드를
    관통하는 경로는 그대로 걸러진다.

    **octomap에는 이 허용이 적용되지 않는다.** octomap은 열매를 지운
    클라우드에서 나온 것이라 줄기·지지대·잎만 들어 있고, 그건 그리퍼가
    스쳐도 되는 대상이 아니다 — 잡히면 잡히는 게 맞다.
    """
    acm = fetch_acm(node)

    allowed = (list(N.GRIPPER_LINK_NAMES) + list(N.WRIST_LINK_NAMES)
               + [N.END_EFFECTOR_NAME])
    names = list(acm.entry_names)
    rows = [list(e.enabled) for e in acm.entry_values]

    # 없는 토마토 항목만 추가한다(전부 False로). 값 자체는 아래에서 세운다 —
    # **매번 다시 세우는 것이 중요하다.** 예전 판은 이미 있는 항목을 건너뛰어서
    # only를 바꿔 다시 불러도 앞의 완화가 그대로 남았다(함정 15와 같은 종류).
    for i in range(len(dets)):
        oid = f'{TOMATO_OBJECT_PREFIX}{i}'
        if oid not in names:
            for row in rows:
                row.append(False)
            names.append(oid)
            rows.append([False] * len(names))

    index = {nm: k for k, nm in enumerate(names)}
    for i in range(len(dets)):
        j = index[f'{TOMATO_OBJECT_PREFIX}{i}']
        allow = (only is None or only == i)
        for link in allowed:
            k = index.get(link)
            if k is None:
                continue
            rows[j][k] = allow
            rows[k][j] = allow

    updated = AllowedCollisionMatrix()
    updated.entry_names = names
    updated.default_entry_names = list(acm.default_entry_names)
    updated.default_entry_values = list(acm.default_entry_values)
    updated.entry_values = [AllowedCollisionEntry(enabled=r) for r in rows]

    scene = PlanningScene()
    scene.is_diff = True
    scene.allowed_collision_matrix = updated
    scene_publisher(node).publish(scene)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    if not quiet:
        what = ('토마토 전부' if only is None
                else f'토마토 #{only + 1}만 (나머지 {len(dets) - 1}개는 장애물)')
        print(f'ACM 갱신: 그리퍼/손목/flange {len(allowed)}개 링크 x {what} '
              '충돌 허용 (팔뚝은 그대로 충돌로 잡힘)')


OCTOMAP_ACM_NAME = '<octomap>'


def allow_gripper_octomap_collisions(node, allow=True, quiet=False):
    """그리퍼·손목·flange가 **octomap voxel**과 충돌해도 되게 ACM을 고친다.

    토마토(구)에 거는 완화(allow_gripper_tomato_collisions)와 같은 형태지만
    대상이 다르다. octomap은 열매를 지운 클라우드에서 나온 것이라 줄기·지지대·
    잎이 들어 있고, 원래는 "그리퍼가 스쳐도 되는 대상이 아니다"라고 보고
    일부러 완화하지 않았다.

    그런데 실측(6.3절)에서 [3/5] 직진 접근의 Cartesian fraction이 열매만 있을
    때 14/15가 100%인데 octomap을 넣으면 4/13으로 떨어졌다. 즉 **줄기 voxel이
    마지막 접근 구간을 막고 있다.** 손가락이 잎을 스치는 것까지 막을 필요가
    있는지는 판단의 문제이므로, 그 판단을 수치로 하기 위한 스위치다.

    **팔뚝(joint2~4)은 여전히 완화하지 않는다** — 팔이 베드나 지지대를
    관통하는 경로는 그대로 걸러야 한다. 완화 대상은 토마토 쪽과 같은 10개
    링크(그리퍼 6 + 손목 2 + flange 1 + gripper_base)다.

    ACM은 이름이 `<octomap>`인 가상 객체로 잡힌다.
    """
    acm = fetch_acm(node)
    allowed = (list(N.GRIPPER_LINK_NAMES) + list(N.WRIST_LINK_NAMES)
               + [N.END_EFFECTOR_NAME])
    names = list(acm.entry_names)
    rows = [list(e.enabled) for e in acm.entry_values]

    if OCTOMAP_ACM_NAME not in names:
        for row in rows:
            row.append(False)
        names.append(OCTOMAP_ACM_NAME)
        rows.append([False] * len(names))
    oi = names.index(OCTOMAP_ACM_NAME)
    for j, nm in enumerate(names):
        if nm in allowed:
            rows[j][oi] = allow
            rows[oi][j] = allow

    updated = AllowedCollisionMatrix()
    updated.entry_names = names
    updated.default_entry_names = list(acm.default_entry_names)
    updated.default_entry_values = list(acm.default_entry_values)
    updated.entry_values = [AllowedCollisionEntry(enabled=r) for r in rows]

    scene = PlanningScene()
    scene.is_diff = True
    scene.allowed_collision_matrix = updated
    scene_publisher(node).publish(scene)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    if not quiet:
        verb = '허용' if allow else '금지'
        print(f'ACM: 그리퍼/손목/flange {len(allowed)}개 링크 x octomap 충돌 {verb} '
              f'(팔뚝은 그대로 충돌로 잡힘)')


def fetch_acm(node):
    if not hasattr(node, '_scene_query'):
        node._scene_query = node.create_client(GetPlanningScene,
                                               '/get_planning_scene')
        node._scene_query.wait_for_service(timeout_sec=10)
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    fut = node._scene_query.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    return fut.result().scene.allowed_collision_matrix


def tip_clearance(pos, quat, dets):
    """그리퍼 끝단에서 가장 가까운 토마토까지 거리(m).

    끝단 = flange에서 그리퍼 정면(로컬 +Z) 방향으로 GRIPPER_LENGTH_OFFSET_M.
    쿼터니언에서 정면 축을 뽑아 쓴다(회전행렬 3번째 열).
    """
    x, y, z, w = quat
    fz = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))
    tip = tuple(p + N.GRIPPER_LENGTH_OFFSET_M * f for p, f in zip(pos, fz))
    return min(math.dist(tip, (d['base_x'], d['base_y'], d['base_z'])) for d in dets)
