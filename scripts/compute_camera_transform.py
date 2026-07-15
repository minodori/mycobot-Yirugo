#!/usr/bin/env python3
"""
관절 각도 6개(pymycobot의 get_angles() 순서/단위와 동일: J1~J6, 도 단위)로부터
g_base -> 카메라 순기구학(FK)을 계산해서, demo_octomap.launch.py의
camera_x/y/z/roll/pitch/yaw 값을 뽑아주는 스크립트.

전제: 카메라는 joint6_flange(그리퍼가 붙는, J6에 따라 회전하는 부분)가 아니라
**joint6 링크 자체**(J5까지만 적용된, J6 회전에는 영향받지 않는 고정 부분)에
마운트되어 있음 — 실측으로 확인함 (J6를 돌리면 그리퍼만 돌아가고 카메라는
그대로임). 그래서 FK 계산에 J6는 사용하지 않음.

카메라의 실제 정면 방향도 joint6의 로컬 X축이 아니라 **로컬 Y축**임을 실측
으로 확인함 (마운트 방식 때문). 카메라 좌표축(X=정면,Y=좌,Z=위)은 joint6
좌표축 기준으로 X=joint6_Y, Y=-joint6_X, Z=joint6_Z에 대응함 (CAMERA_FROM_JOINT6
행렬로 아래에 고정 반영됨 — 결과적으로 joint6 기준 Z축으로 90도 회전).
이 매핑이 실제 마운트와 다르면(예: 다른 로봇/브라켓) 다시 확인해야 함.

카메라 원점과 joint6 원점 사이의 위치 오차는 무시할 만큼 작다고 가정
(필요하면 --camera-offset-x/y/z로 보정, 카메라 로컬 좌표계 기준).

조인트 원점 값은 mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf
에서 그대로 가져옴 (g_base -> joint1 -> ... -> joint6 체인, J1~J5만 사용).

사용법:
  1. RPi에서 서보 릴리즈 후 손으로 카메라가 원하는 뷰를 보도록 자세를 잡음
  2. python3 -c "from pymycobot import MyCobot280; mc = MyCobot280('/dev/ttyJETCOBOT', 1000000); print(mc.get_angles())"
     로 관절 각도 6개(도 단위) 확인
  3. python3 compute_camera_transform.py <J1> <J2> <J3> <J4> <J5> <J6>
     예) python3 compute_camera_transform.py 0 45 -30 0 90 0
     (J6는 카메라 계산에 실제로는 안 쓰이지만, get_angles() 그대로 넣기
     편하도록 인자로는 받음)
"""

import argparse

import numpy as np
from scipy.spatial.transform import Rotation as R

# ---- mycobot_280_m5.urdf 조인트 원점 (g_base -> joint6 체인, J1~J5) ----
JOINT_ORIGINS = [
    # (xyz, rpy) — 각 조인트의 "각도 0일 때" 고정 원점 변환
    ([0.0, 0.0, 0.13156], [0.0, 0.0, 0.0]),          # joint1 -> joint2
    ([0.0, 0.0, 0.0], [0.0, 1.5708, -1.5708]),        # joint2 -> joint3
    ([-0.1104, 0.0, 0.0], [0.0, 0.0, 0.0]),           # joint3 -> joint4
    ([-0.096, 0.0, 0.06462], [0.0, 0.0, -1.5708]),    # joint4 -> joint5
    ([0.0, -0.07318, 0.0], [1.5708, -1.5708, 0.0]),   # joint5 -> joint6
]


# 카메라 좌표축(X=정면,Y=좌,Z=위)을 joint6 좌표축 기준으로 표현한 고정 회전
# (joint6 기준 Z축으로 90도 회전과 동일).
# 열 벡터: camera_X -> joint6_Y, camera_Y -> -joint6_X, camera_Z -> joint6_Z
CAMERA_FROM_JOINT6 = np.array([
    [0.0, -1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])


def homogeneous(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = R.from_euler("xyz", rpy).as_matrix()
    m[:3, 3] = xyz
    return m


def joint_rotation_z(angle_rad):
    m = np.eye(4)
    m[:3, :3] = R.from_euler("z", angle_rad).as_matrix()
    return m


def forward_kinematics(joint_angles_deg):
    """g_base -> joint6 변환 행렬(4x4)을 반환 (J1~J5만 사용, J6은 카메라와 무관)."""
    transform = np.eye(4)  # g_base -> joint1 (fixed, identity)
    for (xyz, rpy), angle_deg in zip(JOINT_ORIGINS, joint_angles_deg[:5]):
        angle_rad = np.radians(angle_deg)
        transform = transform @ homogeneous(xyz, rpy) @ joint_rotation_z(angle_rad)
    return transform


def main():
    parser = argparse.ArgumentParser(
        description="관절 각도(도)로부터 카메라 static transform 값 계산"
    )
    parser.add_argument(
        "angles",
        type=float,
        nargs=6,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="get_angles()로 읽은 관절 각도 6개 (도 단위, J6은 참고용으로만 받음)",
    )
    parser.add_argument("--camera-offset-x", type=float, default=0.0,
                         help="카메라 로컬 좌표계(정면=x) 기준 오프셋 (m)")
    parser.add_argument("--camera-offset-y", type=float, default=0.0,
                         help="카메라 로컬 좌표계(좌=y) 기준 오프셋 (m)")
    parser.add_argument("--camera-offset-z", type=float, default=0.0,
                         help="카메라 로컬 좌표계(위=z) 기준 오프셋 (m)")
    args = parser.parse_args()

    joint6_transform = forward_kinematics(args.angles)

    # 오프셋은 카메라 로컬 좌표계 기준으로 받으므로, joint6 로컬 좌표계로
    # 옮기려면 CAMERA_FROM_JOINT6 회전을 먼저 곱해야 함(homogeneous transform
    # 합성 시 평행이동은 앞쪽 회전의 영향을 받음).
    camera_offset_local = np.array(
        [args.camera_offset_x, args.camera_offset_y, args.camera_offset_z]
    )
    camera_from_joint6 = np.eye(4)
    camera_from_joint6[:3, :3] = CAMERA_FROM_JOINT6
    camera_from_joint6[:3, 3] = CAMERA_FROM_JOINT6 @ camera_offset_local
    camera_transform = joint6_transform @ camera_from_joint6

    position = camera_transform[:3, 3]
    rotation = R.from_matrix(camera_transform[:3, :3])
    roll, pitch, yaw = rotation.as_euler("xyz")

    print(f"입력 관절 각도(도): {args.angles} (J6은 카메라 계산에 미사용)")
    print()
    print(f"camera_x     = {position[0]:.4f}")
    print(f"camera_y     = {position[1]:.4f}")
    print(f"camera_z     = {position[2]:.4f}")
    print(f"camera_roll  = {roll:.4f}")
    print(f"camera_pitch = {pitch:.4f}")
    print(f"camera_yaw   = {yaw:.4f}")
    print()
    print("바로 실행할 명령:")
    print(
        "ros2 launch mycobot_280_moveit2 demo_octomap.launch.py "
        f"camera_x:={position[0]:.4f} camera_y:={position[1]:.4f} "
        f"camera_z:={position[2]:.4f} camera_roll:={roll:.4f} "
        f"camera_pitch:={pitch:.4f} camera_yaw:={yaw:.4f}"
    )


if __name__ == "__main__":
    main()
