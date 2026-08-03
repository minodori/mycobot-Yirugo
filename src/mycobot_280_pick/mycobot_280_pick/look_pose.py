"""look pose 상수와 판정 — **무거운 import가 없는** 공용 모듈.

왜 따로 두나
------------
"팔이 지금 look pose에 있는가"를 세 노드가 각자 판정한다:

  yolo_d435_detector_node      판단(candidates) 게이팅
  pointcloud_tomato_filter_node 클라우드 발행 게이팅(2026-08-03 추가)
  coord_to_goal_node           복귀·검증

그런데 `coord_to_goal_node`를 import하면 pymoveit2·MoveIt 메시지가 전부 딸려
와서, 가벼운 노드들이 그 무게를 지게 된다. 그래서 yolo 노드가 상수를 **복제**해
두고 "두 값이 갈리면 게이팅이 어긋나므로 둘 다 갱신할 것"이라고 적어 뒀는데,
세 번째 복제가 생기려는 순간 그 방식이 한계에 왔다.

이 모듈은 표준 라이브러리만 쓴다 — 어느 노드에서 import해도 부담이 없다.

**원본은 `coord_to_goal_node.LOOK_POSE_JOINT_POSITIONS`다**(그 파일에 이 값이
무엇과 결합돼 있는지 — 카메라 프레이밍, APPROACH_REFERENCE_POINT, WORLD_UP 등 —
가 길게 적혀 있다). 여기 값은 그 사본이고, **look pose를 바꾸면 두 곳을 같이
고쳐야 한다.** 어긋났는지는 `assert_matches()`로 확인할 수 있다.
"""

JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]

LOOK_POSE_JOINT_POSITIONS = [
    -0.130376,
    1.816190,
    -0.989253,
    -0.872665,
    0.279078,
    0.024435,
]

# 팔이 look pose에 있다고 볼 허용 오차(rad). 실물 정지 오차와 컨트롤러
# tolerance를 감안한 값이고, coord_to_goal_node의 대기 자세 판정과 같은 기준이다.
LOOK_POSE_TOLERANCE_RAD = 0.08


def is_near_look_pose(positions_by_name, tolerance=LOOK_POSE_TOLERANCE_RAD):
    """`{관절이름: 각도}`가 look pose 근처인가.

    6축이 다 안 들어왔으면 **False**를 돌려준다 — "모르면 아니다"로 두어야
    게이트가 열린 채로 지나가지 않는다.
    """
    try:
        return all(
            abs(positions_by_name[name] - target) <= tolerance
            for name, target in zip(JOINT_NAMES, LOOK_POSE_JOINT_POSITIONS)
        )
    except KeyError:
        return False


def assert_matches(joint_names, joint_positions, source='(unknown)'):
    """다른 파일의 사본과 이 값이 같은지 확인한다. 다르면 AssertionError.

    look pose를 한쪽에서만 고치는 사고를 잡으려는 것이다 — 어긋나도 에러가
    안 나고 **게이트만 조용히 안 열리는** 종류의 버그라 눈으로는 못 찾는다.
    """
    assert list(joint_names) == JOINT_NAMES, (
        f'{source}의 JOINT_NAMES가 look_pose.py와 다르다')
    assert all(abs(a - b) < 1e-9
               for a, b in zip(joint_positions, LOOK_POSE_JOINT_POSITIONS)), (
        f'{source}의 LOOK_POSE_JOINT_POSITIONS가 look_pose.py와 다르다 — '
        'look pose를 한쪽에서만 고쳤는지 확인할 것')
