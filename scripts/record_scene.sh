#!/usr/bin/env bash
# 토마토 베드 "장면"을 rosbag(mcap)으로 기록한다 — 트랙 A.
#
# record_sweep.sh와 무엇이 다른가
# -------------------------------
# record_sweep.sh는 **팔의 움직임**을 남긴다(카메라 전체 제외). 이 스크립트는
# 반대로 **카메라가 본 장면**을 남긴다: 컬러/depth, YOLO 검출 bbox, 필터링된
# 포인트클라우드, octomap 상태.
#
# 핵심 설계 — 클라우드가 아니라 그 "입력"을 담는다
# ------------------------------------------------
# points_filtered는 rgb 포함 시 포인트당 16바이트 x 307200점 = 4.9MB/장이다.
# 이걸 그대로 담는 대신 원재료(aligned depth + color + camera_info + bbox)를
# 담아두면, 재생할 때 pointcloud_tomato_filter_node를 다시 돌려서 클라우드를
# 재생성할 수 있다. 용량이 줄 뿐 아니라 **bbox_padding_ratio 같은 파라미터를
# 오프라인에서 다시 튜닝**할 수 있다 — 녹화된 클라우드로는 불가능한 일이다.
# (그래서 PROFILE=slim이 기본이다. 아래 참고)
#
# eye-in-hand 제약
# ----------------
# 카메라는 joint6에 얹혀 있다(핸드-아이 캘리브레이션). 따라서 녹화 중 팔이
# 움직이면 장면이 깨진다. **팔을 look pose에 세워둔 채로 기록할 것.**
# coord_to_goal_node가 떠 있으면 YOLO의 target_point를 받아 팔이 자동으로
# 움직이므로, 녹화용으로 YOLO를 띄울 땐 출력을 죽은 토픽으로 리맵할 것:
#   ros2 run mycobot_280_pick yolo_d435_detector_node \
#       --ros-args -r target_point:=target_point_dryrun
#
# 사용법
# ------
#   ./scripts/record_scene.sh                 # slim 프로파일, 무기한(Ctrl+C로 종료)
#   ./scripts/record_scene.sh bed_0731 20     # 이름 지정 + 20초 후 자동 종료
#   PROFILE=full ./scripts/record_scene.sh    # raw/compressed/클라우드 전부(용량 비교용)
#
# 재생 방법 (2026-07-31 실측으로 확정한 순서)
# -------------------------------------------
#   1) realsense 드라이버를 끈다. 안 끄면 라이브 카메라와 재생본이 같은 토픽에
#      동시에 발행된다.
#        pkill -INT -f realsense2_camera_node
#   2) 압축본을 raw로 되돌린다(필터/YOLO 노드는 raw를 구독한다).
#      주의: Jazzy에서 in_transport는 위치 인자가 아니라 **파라미터**다.
#      `republish compressed`처럼 주면 조용히 raw로 잡히고 아무것도 안 나온다.
#        ros2 run image_transport republish --ros-args \
#            -p in_transport:=compressed -p out_transport:=raw \
#            -r in/compressed:=/camera/camera/color/image_raw/compressed \
#            -r out:=/camera/camera/color/image_raw
#        ros2 run image_transport republish --ros-args \
#            -p in_transport:=compressedDepth -p out_transport:=raw \
#            -r in/compressedDepth:=/camera/camera/aligned_depth_to_color/image_raw/compressedDepth \
#            -r out:=/camera/camera/aligned_depth_to_color/image_raw
#   3) 재생성할 노드를 띄운다. YOLO는 target_point를 반드시 리맵할 것(팔 움직임 방지).
#        ros2 run mycobot_280_pick pointcloud_tomato_filter_node
#        ros2 run mycobot_280_pick yolo_d435_detector_node \
#            --ros-args -r target_point:=target_point_dryrun
#   4) 재생. **/tf_static을 반드시 포함할 것** — camera_link -> camera_*_optical_frame
#      static TF는 realsense 드라이버가 발행하는 것이라, 드라이버를 끄면 사라진다.
#      빼먹으면 RViz의 PointCloud2가 "frame이 없다"며 빨간 오류를 낸다(실제로 겪음).
#      /tomato_boxes는 **빼야** 한다 — 재생본과 살아있는 YOLO가 같은 토픽에 겹친다.
#        ros2 bag play bags/<이름> --loop --topics \
#            /camera/camera/color/image_raw/compressed \
#            /camera/camera/aligned_depth_to_color/image_raw/compressedDepth \
#            /camera/camera/color/camera_info \
#            /camera/camera/aligned_depth_to_color/camera_info \
#            /tf_static
#   5) 보기:
#        rviz2 -d src/mycobot_280_pick/config/scene_replay.rviz
#
# /joint_states와 /tf는 일부러 재생하지 않는다. 살아있는 스택(robot_state_publisher)이
# 이미 발행 중이라 발행자가 둘이 되고, sync_plan이 떠 있으면 실물이 엉뚱하게 움직인다
# (2026-07-31 실제 사고). 어차피 녹화 중 팔은 look pose에 정지해 있었으므로 재생할
# 이유도 없다 — 라이브 TF가 녹화 당시와 같은 자세를 가리킨다.

set -euo pipefail

OUT_DIR="${OUT_DIR:-bags}"
NAME="${1:-scene_$(date +%m%d_%H%M)}"
DURATION="${2:-0}"          # 0이면 Ctrl+C까지 무기한
PROFILE="${PROFILE:-slim}"
DEST="${OUT_DIR}/${NAME}"

# 어느 프로파일에서나 필요한 것 — 좌표계와 로봇 모델.
# /robot_description(_semantic)은 transient_local QoS라, 빼면 재생 시 RViz에
# 로봇 모델 자체가 안 뜬다.
COMMON=(
    /tf /tf_static /joint_states
    /robot_description /robot_description_semantic
    /camera/camera/color/camera_info
    /camera/camera/aligned_depth_to_color/camera_info
    /camera/camera/extrinsics/depth_to_color
    /tomato_boxes                 # 마스킹 근거(수십 바이트). 재생 시 필터 노드 입력
    /tomato_candidates            # 클래스+3D좌표+반지름+관측횟수. 전부 합쳐 수백 바이트
    /target_radius_m
    /monitored_planning_scene     # octomap 상태가 담기는 유일한 토픽
)

# slim: 압축본만 담고, 클라우드/검출영상은 재생 때 재생성한다.
#
# 20초 시범 기록(PROFILE=full)의 토픽별 실측 바이트로 정했다 — 합계 87.4 MB/s:
#   color/image_raw            482 MB (28.8%)  921.7 KB/장
#   points_filtered            428 MB (25.6%) 4915.4 KB/장  <- 재생성 가능
#   tomato_detections_image    352 MB (21.1%)  921.7 KB/장  <- 재생성 가능
#   aligned_depth/image_raw    320 MB (19.2%)  614.5 KB/장
#   color .../compressed        50 MB ( 3.0%)   97.9 KB/장  <- raw의 1/9.4
#   aligned .../compressedDepth  37 MB ( 2.2%)   72.0 KB/장  <- raw의 1/8.5
#
# -> slim은 4.7 MB/s로, full 대비 약 18배 작다.
#
# compressedDepth는 16UC1을 PNG로 담아 **무손실**이라 depth 값이 그대로
# 보존된다(포인트클라우드 좌표가 정확히 재현됨). 컬러는 JPEG이라 손실이 있지만
# YOLO 입력으로는 충분하다 — 실제 카메라들도 JPEG로 스트리밍한다.
#
# 재생 시에는 raw로 되돌려야 필터/YOLO 노드가 구독할 수 있다:
#   ros2 run image_transport republish compressed --ros-args \
#       -r in/compressed:=/camera/camera/color/image_raw/compressed \
#       -r out:=/camera/camera/color/image_raw
SLIM=(
    /camera/camera/color/image_raw/compressed
    /camera/camera/aligned_depth_to_color/image_raw/compressedDepth
)

# full: raw와 결과물까지 전부. 용량 비교나 "노드 없이 그냥 보기"용.
FULL=(
    "${SLIM[@]}"
    /camera/camera/color/image_raw
    /camera/camera/aligned_depth_to_color/image_raw
    /camera/camera/depth/color/points_filtered
    /tomato_detections_image
)

case "${PROFILE}" in
    slim) TOPICS=("${COMMON[@]}" "${SLIM[@]}") ;;
    full) TOPICS=("${COMMON[@]}" "${FULL[@]}") ;;
    *) echo "알 수 없는 PROFILE: ${PROFILE} (slim|full)" >&2; exit 1 ;;
esac

mkdir -p "${OUT_DIR}"
if [ -e "${DEST}" ]; then
    echo "이미 존재함: ${DEST}" >&2
    echo "다른 이름을 주거나 지운 뒤 다시 실행할 것: $0 <이름> [초]" >&2
    exit 1
fi

echo "기록 시작 -> ${DEST}"
echo "  프로파일: ${PROFILE} (${#TOPICS[@]}개 토픽)"
if [ "${DURATION}" -gt 0 ]; then
    echo "  길이: ${DURATION}초 후 자동 종료"
else
    echo "  길이: 무기한 — 중지는 Ctrl+C"
fi
echo

# 길이 제한은 SIGINT로 건다. `ros2 bag record`의 -d/--max-bag-duration은
# "이 주기로 파일을 분할"이지 "이만큼 녹화하고 멈춤"이 아니라서 쓸 수 없고
# (--max-duration은 Jazzy에 아예 없음), SIGKILL로 끊으면 metadata.yaml이
# 안 써져서 bag이 열리지 않는다. SIGINT여야 정상 마감된다.
if [ "${DURATION}" -gt 0 ]; then
    exec timeout -s INT "${DURATION}" ros2 bag record --storage mcap -o "${DEST}" "${TOPICS[@]}"
fi

exec ros2 bag record --storage mcap -o "${DEST}" "${TOPICS[@]}"
