#!/usr/bin/env bash
# 스윕 전 과정을 rosbag(mcap)으로 기록한다.
#
# 왜 필요한가
# -----------
# 지금까지 분석은 `coord_to_goal_node`의 텍스트 로그(tee)를 정규식으로 긁는
# 방식이었다. 그래서 **관절값이 필요한 지표를 아예 측정할 수 없었다** —
# 대표적으로 "J1 실제 스윙"(예상 34도)을 여섯 번의 스윕 동안 한 번도 확인하지
# 못했다. rosbag으로 /joint_states를 남기면 그게 바로 계산된다.
#
# 무엇을 담고 무엇을 빼는가
# -------------------------
# `--all-topics`로 전부 담되 카메라만 제외한다. 이유가 둘이다:
#   1) 카메라(색/depth/pointcloud 22개 토픽)는 초당 수십 MB라 용량이 폭증한다.
#      나머지 핵심 토픽은 실측 3.5MB/분이라 27점 스윕(약 13.5분)이 ~48MB다.
#   2) `--all-topics`를 쓰면 기록 시작 시점에 **아직 없는 토픽**(coord_to_goal_node를
#      나중에 띄우면 생기는 /target_point, /plan_result, /grasp_step 등)도
#      자동으로 잡힌다. 토픽을 일일이 나열하면 이것들을 놓치기 쉽다.
#
# 특히 함께 담기는 것들:
#   /joint_states                        실제 관절 궤적(100Hz) — J1 스윙의 출처
#   /arm_group_controller/joint_trajectory  명령된 궤적 — 실제값과 비교하면 추종오차
#   /rosout                              노드 로그 전체. 텍스트 tee와 달리 타임스탬프·
#                                        레벨·노드명이 필드로 분리돼 파싱이 쉽다
#   /tf, /tf_static                      flange 실제 위치 추적, RViz 재생에 필수
#   /robot_description(_semantic)        transient_local QoS. 빼면 재생 시 RViz에
#                                        로봇 모델이 안 뜬다
#
# 사용법
# ------
#   터미널 A: ros2 launch mycobot_280_moveit2 demo_octomap.launch.py enable_octomap:=false
#   터미널 B: ros2 run mycobot_280_pick coord_to_goal_node
#   터미널 C: ./scripts/record_sweep.sh              <- 이 스크립트
#   터미널 D: ./scripts/sweep_target_z.sh
#
# 스윕이 끝나면 터미널 C에서 Ctrl+C. 결과는 bags/<이름>/ 에 남는다.
#
# 재생 시 주의
# ------------
# 실제 스택이나 sync_plan이 떠 있는 상태에서 `ros2 bag play`를 하면
# /joint_states 발행자가 둘이 되어 서로 다른 값이 교대로 나간다(2026-07-31에
# 실제로 겪은 사고 — 스택이 두 벌 돌면서 실물이 엉뚱하게 움직였다).
# 재생 전에 반드시 스택과 sync_plan을 모두 끌 것.

set -euo pipefail

OUT_DIR="${OUT_DIR:-bags}"
NAME="${1:-sweep_$(date +%m%d_%H%M)}"
DEST="${OUT_DIR}/${NAME}"

# 제외 패턴. 10초 시범 기록의 토픽별 용량을 실측해서 정했다(총 12.8MB = 71MB/분):
#   /controller_manager/statistics/*       32%  순수 진단용, 분석에 안 씀
#   /filtered_cloud                        26%  메시지 1개가 3.4MB(포인트클라우드)
#   /controller_manager/introspection_data 15%  진단용
#   /dynamic_joint_states                   4%  /joint_states와 중복
# 이 넷을 빼면 71 -> 약 10MB/분이 되어 27점 스윕이 ~140MB로 떨어진다.
#
# [2026-07-31 정정] `/monitored_planning_scene`(6%)도 처음엔 제외했다가 되돌렸다.
# 그게 **octomap 상태가 담기는 유일한 토픽**이기 때문이다 — 실제로 한 스윕이
# octomap 충돌로 전멸했을 때(enable_octomap 기본값이 true라 카메라 장면이
# 장애물로 들어갔음) 이 토픽이 없어서 "planning scene에 octomap 0개"라는
# 잘못된 결론을 내렸고, /rosout의 충돌 메시지를 뒤져서야 원인을 찾았다.
# 6%는 그 진단 가치에 비하면 싸다.
#
# 남는 것 중 큰 것은 /arm_group_controller/controller_state인데, 이건 **명령값과
# 실제값을 같이 담고 있어** 추종 오차 분석에 필요하므로 유지한다.
#
# octomap/포인트클라우드까지 봐야 하면 EXCLUDE를 '^/camera/'로만 줄여서 실행할 것.
EXCLUDE="${EXCLUDE:-^/camera/|^/controller_manager/(statistics|introspection_data)|^/filtered_cloud$|^/dynamic_joint_states$}"

mkdir -p "${OUT_DIR}"

if [ -e "${DEST}" ]; then
    echo "이미 존재함: ${DEST}" >&2
    echo "다른 이름을 주거나 지운 뒤 다시 실행할 것: $0 <이름>" >&2
    exit 1
fi

echo "기록 시작 -> ${DEST}"
echo "  제외 패턴: ${EXCLUDE}"
echo "  중지: Ctrl+C"
echo

exec ros2 bag record \
    --all-topics \
    --exclude-regex "${EXCLUDE}" \
    --storage mcap \
    -o "${DEST}"
