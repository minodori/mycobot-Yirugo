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


def scene_publisher(node):
    """PlanningScene 발행자를 노드에 한 번만 만든다.

    DDS 디스커버리가 붙기 전에 쏘면 조용히 사라지므로 만든 직후 잠깐 돈다.
    """
    if not hasattr(node, '_scene_pub'):
        node._scene_pub = node.create_publisher(PlanningScene, '/planning_scene', 10)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
    return node._scene_pub


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


def allow_gripper_tomato_collisions(node, dets, quiet=False):
    """그리퍼·손목·flange가 **모든 토마토**와 충돌해도 되게 ACM을 고친다.

    이걸 안 하면 접근할 토마토 자신과 충돌해 플래닝이 거의 전부 실패한다 —
    실측 성공률 2~4%였다. coord_to_goal_node가 목표 지점의 target_object에
    대해 똑같은 처리를 한다(_allow_gripper_target_object_collision, L1352):
    "그 자리에 등록된 구가 flange 자신과 충돌로 잡혀서 Unable to sample any
    valid states for goal tree로 매번 실패"했다는 실측 기록이 있고, joint5/
    joint6도 같은 이유로 허용 목록에 있다.

    주의: 이 처리는 "그리퍼가 열매를 스쳐도 된다"는 뜻이다. 팔뚝(joint2~4)은
    여전히 충돌로 잡히므로, 팔이 베드를 관통하는 경로는 그대로 걸러진다.

    **octomap에는 이 허용이 적용되지 않는다.** octomap은 열매를 지운
    클라우드에서 나온 것이라 줄기·지지대·잎만 들어 있고, 그건 그리퍼가
    스쳐도 되는 대상이 아니다 — 잡히면 잡히는 게 맞다.
    """
    acm = fetch_acm(node)

    allowed = (list(N.GRIPPER_LINK_NAMES) + list(N.WRIST_LINK_NAMES)
               + [N.END_EFFECTOR_NAME])
    names = list(acm.entry_names)
    rows = [list(e.enabled) for e in acm.entry_values]

    for i in range(len(dets)):
        oid = f'{TOMATO_OBJECT_PREFIX}{i}'
        if oid in names:
            continue
        for row in rows:
            row.append(False)
        for j, nm in enumerate(names):
            rows[j][-1] = nm in allowed
        new_row = [nm in allowed for nm in names]
        new_row.append(False)
        names.append(oid)
        rows.append(new_row)

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
        print(f'ACM 갱신: 그리퍼/손목/flange {len(allowed)}개 링크 x 토마토 '
              f'{len(dets)}개 충돌 허용 (팔뚝은 그대로 충돌로 잡힘)')


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
