#!/usr/bin/env bash
# src/mycobot_ros2(vendored 중첩 저장소)의 프로젝트 고유 변경을 복원한다.
#
# 왜 필요한가
# -----------
# `.gitignore`가 `src/mycobot_ros2/`를 제외한다. 그 안에 자체 `.git`을 가진
# 중첩 저장소가 있어서(원격: elephantrobotics/mycobot_ros2, 브랜치 humble)
# 부모 저장소가 그 안으로 내려가지 않기 때문이다 — gitlink(서브모듈)로만
# 기록하려 한다. 통째 편입도 불가하다: 1.4 GB다.
#
# 그런데 그 안에 **이 프로젝트가 동작하는 데 필수인 변경**이 들어 있다:
#   - firefighter.urdf.xacro : 그리퍼 콜리전 mesh scale="0.001 ..." (없으면
#     콜리전 형상이 1000배로 잡혀 30cm 거리에서도 충돌로 판정된다)
#   - firefighter.urdf.xacro : 그리퍼 마운트 rpy="1.562593 0 3.141593"
#     (실물 TF를 직접 비교해 확정한 값)
#   - firefighter.srdf       : look_pose group_state, self-collision 목록
#   - initial_positions.yaml : FakeSystem 시작 자세 = look pose
#   - sensors_3d.yaml        : occupancy_map_monitor 설정(padding_scale 0.92 등)
#   - ompl_planning.yaml     : Ruckig jerk 제한 스무딩(실물 떨림 완화)
#   - kinematics.yaml, ros2_controllers.yaml, moveit_controllers.yaml
#   - launch/demo_octomap.launch.py : enable_octomap/enable_camera/
#     enable_pointcloud_filter/use_rviz 인자, 핸드아이 TF, 필터 노드
#
# **다른 장비에서 clone하면 이것들이 전부 없어서 아무것도 동작하지 않는다.**
# 그래서 패치로 떠서 부모 저장소에 넣어 둔다(전체 62KB + 커밋 3개 31KB).
#
# 무엇이 담겼나
# -------------
#   0001~0003-*.patch        상류(origin/humble)에 없는 **로컬 커밋 3개**.
#                            format-patch라 커밋 메시지와 작성자가 보존된다.
#   9999-worktree-*.patch    커밋되지 않은 작업트리 변경(30개 파일).
#
# 기준 상류 커밋: 3999e2cd (2026-02-09, "Merge pull request #55 ...")
# 그보다 새 상류에 적용하면 충돌할 수 있다 — 그때는 수동 병합해야 한다.
#
# 사용법
# ------
#   # 재클론 직후 (src/mycobot_ros2가 깨끗한 상태여야 한다)
#   ./scripts/apply_vendor_patch.sh
#
#   # 지금 상태에서 패치를 다시 뜨려면 (설정을 고칠 때마다 갱신할 것)
#   ./scripts/apply_vendor_patch.sh --regenerate
#
# 적용 후 반드시 빌드할 것:
#   colcon build --symlink-install --packages-select mycobot_280_moveit2
# `ros2 param set`으로는 반영되지 않는 설정들이라 완전 재시작도 필요하다.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR="${ROOT}/src/mycobot_ros2"
PATCHES="${ROOT}/vendor/mycobot_ros2"
BASE_COMMIT="3999e2cda7460d61f4fd2ffaa31049f000eae7a8"

if [ ! -d "${VENDOR}/.git" ]; then
    echo "중첩 저장소가 없다: ${VENDOR}" >&2
    echo "먼저 클론할 것:" >&2
    echo "  git clone -b humble https://github.com/elephantrobotics/mycobot_ros2.git ${VENDOR}" >&2
    exit 1
fi

if [ "${1:-}" = "--regenerate" ]; then
    echo "현재 상태에서 패치를 다시 뜬다 -> ${PATCHES}"
    rm -f "${PATCHES}"/*.patch
    git -C "${VENDOR}" format-patch --no-signature -o "${PATCHES}" origin/humble..HEAD
    git -C "${VENDOR}" diff > "${PATCHES}/9999-worktree-uncommitted.patch"
    echo
    ls -1 "${PATCHES}"
    echo
    echo "커밋 $(git -C "${VENDOR}" rev-list --count origin/humble..HEAD)개 + 작업트리 변경"
    echo "부모 저장소에 커밋하는 것을 잊지 말 것."
    exit 0
fi

# --- 적용 ---
CURRENT="$(git -C "${VENDOR}" rev-parse HEAD)"
if [ "${CURRENT}" != "${BASE_COMMIT}" ]; then
    echo "경고: 중첩 저장소 HEAD가 기준 커밋과 다르다." >&2
    echo "  현재  ${CURRENT}" >&2
    echo "  기준  ${BASE_COMMIT}" >&2
    echo "이미 패치가 적용돼 있거나 상류가 갱신된 상태일 수 있다." >&2
    read -r -p "그래도 진행하려면 yes: " answer
    [ "${answer}" = "yes" ] || exit 1
fi

echo "로컬 커밋 3개 적용..."
git -C "${VENDOR}" am "${PATCHES}"/000*.patch

echo "작업트리 변경 적용..."
git -C "${VENDOR}" apply "${PATCHES}/9999-worktree-uncommitted.patch"

echo
echo "완료. 다음을 실행할 것:"
echo "  colcon build --symlink-install --packages-select mycobot_280_moveit2"
