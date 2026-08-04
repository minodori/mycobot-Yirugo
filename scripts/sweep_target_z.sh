#!/usr/bin/env bash
# 타겟 높이(z)를 내려가며 /target_point를 한 번씩 발행하는 스윕 스크립트.
#
# 2026-07-30 접근축 리팩터(coord_to_goal_node의 고도각 탐색 + 접근축 위
# 웨이포인트 계산) 전후를 같은 조건으로 비교하기 위해 만듦. 그 전에는
# 손으로 z를 바꿔가며 ros2 topic pub을 반복했는데, 사이클마다 대기 시간이
# 달라져서 로그를 나란히 놓고 보기 어려웠음.
#
# 사용법:
#   터미널 A: stdbuf -oL -eL ros2 run mycobot_280_pick coord_to_goal_node 2>&1 | tee coord_to_node.log
#   터미널 B: ./scripts/sweep_target_z.sh 2>&1 | tee topic_pub.log
#
# 한 사이클(정렬->그리퍼 열기->접근->파지->후퇴->look pose 복귀)이 끝나기 전에
# 다음 좌표를 보내면 coord_to_goal_node가 "이전 목표 실행 중이라 무시함"으로
# 버리므로, DWELL_SEC은 한 사이클이 끝날 만큼 넉넉히 잡아야 함.

set -euo pipefail

TARGET_X="${TARGET_X:-0.24}"
TARGET_Y="${TARGET_Y:--0.02}"
Z_START="${Z_START:-0.40}"
Z_END="${Z_END:-0.14}"
Z_STEP="${Z_STEP:-0.01}"
DWELL_SEC="${DWELL_SEC:-45}"

echo "스윕 시작: x=${TARGET_X} y=${TARGET_Y} z=${Z_START} -> ${Z_END} (step ${Z_STEP}), 사이클 간격 ${DWELL_SEC}s"

z="${Z_START}"
while (( $(echo "$z >= $Z_END" | bc -l) )); do
    echo "=== target z=${z} ==="
    ros2 topic pub --once /target_point geometry_msgs/msg/PointStamped \
        "{header: {frame_id: 'g_base'}, point: {x: ${TARGET_X}, y: ${TARGET_Y}, z: ${z}}}"
    sleep "${DWELL_SEC}"
    z=$(echo "$z - $Z_STEP" | bc -l)
    # bc가 ".39" 같은 선행 0 없는 형식을 내므로 0을 붙여 정규화
    z=$(printf '%.2f' "$z")
done

echo "스윕 완료."
