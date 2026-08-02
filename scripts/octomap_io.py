#!/usr/bin/env python3
"""planning scene의 octomap을 **파일로 굳히고 되돌린다**.

왜 필요한가
-----------
`docs/ARMED_POSE_HANDOFF.md` 5절(octomap 재검증) — 그전까지의 모든 수치는 씬에 **열매(구)만**
넣고 잰 것이다. 줄기·지지대·잎은 형상을 몰라 빠져 있고, 그게 실물과의 가장 큰
차이일 가능성이 높다. 그 형상은 이미 `bags/bed_look_slim`에 들어 있다 — 재생하면
occupancy_map_monitor가 octomap으로 만들어 준다.

문제는 **비용**이다. octomap을 한 번 만들려면 스택 + republish 2개 + 필터 2개 +
YOLO + bag 재생(런치 7액션)을 띄우고 워밍업까지 기다려야 한다. 측정은 그 뒤
수십 분씩 걸리고 여러 번 반복해야 하는데, 그때마다 이 무거운 재생을 다시 하는
것은 말이 안 된다. 그래서 **한 번 만든 octomap을 파일로 떼어낸다.**

떼어내면 부수 효과가 셋이다.
  1. 측정 스택이 가벼워진다 — `enable_camera:=false`로 띄운 뒤 주입만 하면 된다.
  2. **같은 장애물로 반복 측정**할 수 있다. octomap은 재생할 때마다 미세하게
     달라지므로(레이캐스팅 순서, 프레임 타이밍), 파일로 고정하지 않으면
     "armed pose가 나아졌나"와 "octomap이 달라졌나"를 구분할 수 없다.
  3. 저장소에 남는다 — 나중에 이 수치를 재현하려는 사람이 bag 143 MB를 재생하는
     대신 파일 하나를 주입하면 된다.

octomap 바이너리를 파이썬으로 읽는다
------------------------------------
`sensors_3d.yaml`의 주석에 *"octomap 메시지가 표준 .ot/.bt 파일 헤더 형식이
아니라서 파이썬으로 직접 디코딩도 실패함"* 이라는 기록이 있다. 맞는 관찰인데
결론이 반만 맞다 — **헤더가 없을 뿐 본문은 표준 octree 스트림**이다.
`.ot`/`.bt` 파일 파서에 그대로 물리면 헤더가 없어 실패하지만, 본문 규약대로
읽으면 그냥 읽힌다. 그래서 여기서는 **voxel 개수와 bbox를 직접 센다** —
"0바이트인가" 같은 간접 지표 대신 실제 점유 voxel을 보고 판단할 수 있다.

**형식이 둘이고 `Octomap.binary` 플래그가 어느 쪽인지 알려준다.** MoveIt이
`/get_planning_scene`으로 돌려주는 것은 `binary=False`(full) 쪽이다 — 실측에서
63350바이트 = 12670노드 x 5로 정확히 나누어떨어졌다.

  full (`binary=False`, `writeNodesRecurs`) — 노드당 **5바이트**
      float32 log-odds + 자식 존재 비트 1바이트. 자식 비트가 0이면 그 노드가
      리프다. 점유 판정은 log-odds >= 0 (확률 0.5).
  binary (`binary=True`, `writeBinaryNode`) — 노드당 **2바이트**
      자식 8개 x 2비트. 비트쌍 (b0,b1): (0,0) 자식 없음 / (1,0) free 리프 /
      (0,1) occupied 리프 / (1,1) 자식 있음 → 재귀.

full 쪽이 정보가 많다(리프별 점유 확률). 여기서는 점유 여부만 쓴다.

사용법
------
    # 재생 세션에서 octomap이 형성된 뒤
    python3 scripts/octomap_io.py capture --out bags/bed_look_octomap.bin

    # 가벼운 측정 스택에 되돌리기
    python3 scripts/octomap_io.py inject bags/bed_look_octomap.bin

    # 파일만 들여다보기 (move_group은 안 떠 있어도 된다.
    #  다만 역직렬화에 rclpy를 쓰므로 setup.bash는 source해야 한다)
    python3 scripts/octomap_io.py stats bags/bed_look_octomap.bin

    # 현재 살아있는 씬의 octomap 상태만 확인
    python3 scripts/octomap_io.py stats
"""

import argparse
import functools
import math
import os
import struct
import sys

print = functools.partial(print, flush=True)

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
for _p in ('src/mycobot_280_pick', 'src/pymoveit2'):
    sys.path.insert(0, os.path.join(_ROOT, _p))

# 파일 앞에 붙이는 자체 헤더. serialize_message의 CDR 바이트만 저장하면 나중에
# "이게 뭐였더라"가 되므로 최소한의 메타(프레임·해상도·바이트수)를 같이 담는다.
MAGIC = b'MCBOCT01'


# --------------------------------------------------------------------------
# 디코딩 (decode_octree 자체는 순수 파이썬 — 메시지 타입도 rclpy도 안 쓴다)
# --------------------------------------------------------------------------

def _child_center(center, child_half, i):
    """octomap computeChildIdx 규약: bit0=x, bit1=y, bit2=z."""
    return (center[0] + (child_half if i & 1 else -child_half),
            center[1] + (child_half if i & 2 else -child_half),
            center[2] + (child_half if i & 4 else -child_half))


def decode_octree(data, resolution, binary, tree_depth=16):
    """octree 스트림 → occupied 리프 목록 [(x, y, z, size), ...]

    좌표는 octomap 자체 좌표계(원점 중심) 기준이다. 프레임 좌표로 쓰려면
    OctomapWithPose.origin을 곱해야 하는데, 이 로봇에서는 항상 단위변환이라
    (occupancy_map_monitor가 origin을 건드리지 않는다) 그대로 써도 된다.

    pruned 리프는 상위 깊이에 있을 수 있으므로 size를 같이 돌려준다 —
    voxel 개수만 세면 큰 덩어리 하나와 작은 점 하나가 같은 무게가 된다.
    """
    n = len(data)
    root_half = resolution * (2 ** tree_depth) / 2.0
    leaves = []
    # 두 형식 모두 **깊이 우선, 자식 0..7 순서**로 기록된다. 스트림을 그
    # 순서대로 소비해야 하므로 재귀가 가장 안전하다(깊이는 최대 16).
    pos = 0

    def recurse_full(center, half):
        nonlocal pos
        if pos + 5 > n:
            raise ValueError(f'full 스트림이 {pos}바이트에서 끊겼다 (전체 {n})')
        (value,) = struct.unpack_from('<f', data, pos)
        mask = data[pos + 4]
        pos += 5
        if mask == 0:
            # 자식이 없으면 리프. 점유 판정은 log-odds >= 0 (확률 0.5).
            if value >= 0.0:
                leaves.append((center[0], center[1], center[2], half * 2.0))
            return
        child_half = half / 2.0
        for i in range(8):
            if mask & (1 << i):
                recurse_full(_child_center(center, child_half, i), child_half)

    def recurse_binary(center, half):
        nonlocal pos
        if pos + 2 > n:
            raise ValueError(f'binary 스트림이 {pos}바이트에서 끊겼다 (전체 {n})')
        bits = data[pos] | (data[pos + 1] << 8)
        pos += 2
        child_half = half / 2.0
        for i in range(8):
            b0 = (bits >> (i * 2)) & 1
            b1 = (bits >> (i * 2 + 1)) & 1
            if b0 == 0 and b1 == 0:
                continue                              # 자식 없음(미지)
            c = _child_center(center, child_half, i)
            if b0 == 1 and b1 == 1:
                recurse_binary(c, child_half)         # 자식 있음
            elif b0 == 0 and b1 == 1:
                leaves.append((c[0], c[1], c[2], child_half * 2.0))

    sys.setrecursionlimit(max(sys.getrecursionlimit(), 4 * tree_depth + 200))
    (recurse_binary if binary else recurse_full)((0.0, 0.0, 0.0), root_half)
    if pos != n:
        raise ValueError(f'스트림에 {n - pos}바이트가 남았다 — 형식이 다르다 '
                         f'(binary={binary})')
    return leaves


def decode_msg(oct_msg):
    return decode_octree(bytes(oct_msg.data), oct_msg.resolution, oct_msg.binary)


def encode_octree(leaves, resolution, tree_depth=16, log_odds=3.5):
    """occupied 리프 목록 → **full 형식**(`binary=False`) 스트림.

    `decode_octree`의 역방향이다. 형식은 노드당 5바이트
    (float32 log-odds + 자식 존재 비트 1바이트), 깊이 우선·자식 0..7 순서.

    **pruned 리프(size > resolution)는 최말단 voxel로 펼쳐서 쓴다.** 그냥
    중심 하나로 접으면 40mm 리프가 20mm voxel 하나가 되어 부피가 1/8로 줄고
    위치까지 어긋난다(40mm 노드의 중심은 20mm 격자의 모서리다). 실제 베드
    octomap에 40mm 리프가 섞여 있으므로 이 처리가 필요하다.

    다시 pruning(자식 8개가 모두 점유면 부모로 합치기)은 하지 않는다 — 안 해도
    octomap이 정상적으로 읽고, 노드 수가 문제될 규모가 아니다.

    `log_odds`는 점유 확률의 로그오즈. MoveIt의 판정 기준은 `>= 0`이고
    octomap 기본 clamping 상한이 3.5이므로 그 값을 쓴다(확실히 점유).
    """
    # 좌표 → octomap **키**. 키는 부호 없는 정수이고 원점이 tree_max_val
    # (= 2^(tree_depth-1) = 32768)에 온다. 이 오프셋을 빼먹으면 디코더가
    # 좌표를 원점 반대편 655m 근처로 복원한다(실제로 그렇게 틀렸었다).
    offset = 1 << (tree_depth - 1)
    keys = set()
    for leaf in leaves:
        x, y, z = leaf[0], leaf[1], leaf[2]
        size = leaf[3] if len(leaf) > 3 else resolution
        n = max(1, int(round(size / resolution)))
        if n == 1:
            keys.add((int(math.floor(x / resolution)) + offset,
                      int(math.floor(y / resolution)) + offset,
                      int(math.floor(z / resolution)) + offset))
            continue
        # pruned 리프를 최말단 격자로 펼친다. 중심에서 size/2를 뺀 곳이 시작.
        i0 = int(math.floor((x - size / 2.0) / resolution + 0.5)) + offset
        j0 = int(math.floor((y - size / 2.0) / resolution + 0.5)) + offset
        k0 = int(math.floor((z - size / 2.0) / resolution + 0.5)) + offset
        for di in range(n):
            for dj in range(n):
                for dk in range(n):
                    keys.add((i0 + di, j0 + dj, k0 + dk))
    if not keys:
        return b''
    if any(not (0 <= c < (1 << tree_depth)) for k in keys for c in k):
        raise ValueError(f'좌표가 tree_depth={tree_depth} 범위를 벗어난다')

    # 깊이 d의 노드 키 = 최말단 키를 (tree_depth - d)비트 오른쪽 시프트한 것.
    # 깊이 0(루트)에서는 전부 0이 되므로 루트는 항상 (0,0,0)이다.
    levels = [{(kx >> (tree_depth - d), ky >> (tree_depth - d),
                kz >> (tree_depth - d)) for kx, ky, kz in keys}
              for d in range(tree_depth + 1)]

    out = bytearray()

    def emit(depth, key):
        """깊이 depth의 노드 하나를 쓰고 자식으로 재귀."""
        if depth == tree_depth:
            out.extend(struct.pack('<f', log_odds))
            out.append(0)                     # 자식 없음 = 리프
            return
        i, j, k = key
        mask = 0
        children = []
        for c in range(8):
            # decode의 _child_center와 같은 규약: bit0=x, bit1=y, bit2=z.
            # 자식 키 = 부모 키를 왼쪽으로 밀고 해당 비트를 붙인 것.
            ck = (i * 2 + (1 if c & 1 else 0),
                  j * 2 + (1 if c & 2 else 0),
                  k * 2 + (1 if c & 4 else 0))
            if ck in levels[depth + 1]:
                mask |= (1 << c)
                children.append(ck)
        # 내부 노드도 값이 필요하다. octomap 관례대로 점유로 채운다.
        out.extend(struct.pack('<f', log_odds))
        out.append(mask)
        for ck in children:
            emit(depth + 1, ck)

    sys.setrecursionlimit(max(sys.getrecursionlimit(), 4 * tree_depth + 200))
    emit(0, (0, 0, 0))
    return bytes(out)


def box_leaves(xyz_min, xyz_max, resolution):
    """축정렬 상자를 채우는 voxel 중심 목록. encode_octree의 입력용."""
    import itertools
    def rng(lo, hi):
        i0 = int(math.floor(lo / resolution))
        i1 = int(math.floor((hi - 1e-9) / resolution))
        return range(i0, i1 + 1)
    out = []
    for i, j, k in itertools.product(rng(xyz_min[0], xyz_max[0]),
                                     rng(xyz_min[1], xyz_max[1]),
                                     rng(xyz_min[2], xyz_max[2])):
        out.append(((i + 0.5) * resolution, (j + 0.5) * resolution,
                    (k + 0.5) * resolution, resolution))
    return out


def summarize(leaves, resolution, label=''):
    if not leaves:
        print(f'{label}점유 voxel 0개 — octomap이 비어 있다')
        return None
    xs = [l[0] for l in leaves]
    ys = [l[1] for l in leaves]
    zs = [l[2] for l in leaves]
    # 부피는 pruned 리프 크기를 반영해야 실제 점유량이 나온다.
    volume = sum(l[3] ** 3 for l in leaves)
    sizes = sorted({round(l[3], 4) for l in leaves})
    print(f'{label}점유 voxel {len(leaves)}개, 해상도 {resolution*1000:.0f}mm, '
          f'점유 부피 {volume*1e6:.0f}cm^3')
    print(f'{label}  x {min(xs):+.3f}~{max(xs):+.3f}  '
          f'y {min(ys):+.3f}~{max(ys):+.3f}  z {min(zs):+.3f}~{max(zs):+.3f}')
    if len(sizes) > 1:
        print(f'{label}  리프 크기 {len(sizes)}종 (pruned): '
              + ', '.join(f'{s*1000:.0f}mm' for s in sizes[:6]))
    return {'count': len(leaves), 'volume_m3': volume,
            'bbox': (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))}


def near_targets_report(leaves, dets, radii=(0.05, 0.10)):
    """토마토 주변에 voxel이 얼마나 있는가 — 접근 경로를 막는 것이 이것이다.

    베드 전체 voxel 수는 커도 대부분 팔이 갈 일 없는 곳에 있다. 실제로 문제가
    되는 것은 **열매 근처**의 줄기·지지대이므로 그것만 따로 센다.
    """
    if not dets:
        return
    print('\n토마토 주변 voxel (열매를 지운 클라우드에서 나온 것 = 줄기·지지대·잎)')
    for r in radii:
        per = []
        for d in dets:
            p = (d['base_x'], d['base_y'], d['base_z'])
            per.append(sum(1 for l in leaves if math.dist(l[:3], p) <= r))
        covered = sum(1 for c in per if c > 0)
        print(f'  반경 {r*100:.0f}cm 이내: 토마토 {covered}/{len(dets)}개가 '
              f'voxel을 가짐, 개당 {min(per)}~{max(per)}개 '
              f'(합 {sum(per)})')


# --------------------------------------------------------------------------
# ROS 쪽
# --------------------------------------------------------------------------

def _ros_imports():
    try:
        import rclpy
        from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene
        from octomap_msgs.msg import OctomapWithPose
        from rclpy.node import Node
        from rclpy.serialization import deserialize_message, serialize_message
    except ImportError as exc:
        sys.exit(f'import 실패: {exc}\n  source /opt/ros/jazzy/setup.bash 를 먼저 할 것')
    return (rclpy, Node, PlanningScene, PlanningSceneComponents, GetPlanningScene,
            OctomapWithPose, serialize_message, deserialize_message)


def fetch_octomap(node, rclpy, GetPlanningScene, PlanningSceneComponents,
                  timeout=15.0):
    client = node.create_client(GetPlanningScene, '/get_planning_scene')
    if not client.wait_for_service(timeout_sec=timeout):
        sys.exit('/get_planning_scene 없음 — move_group이 떠 있는지 확인할 것')
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.OCTOMAP
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if fut.result() is None:
        sys.exit('/get_planning_scene 응답 없음')
    return fut.result().scene.world.octomap


def cmd_capture(args):
    (rclpy, Node, PlanningScene, PlanningSceneComponents, GetPlanningScene,
     OctomapWithPose, serialize_message, _) = _ros_imports()
    rclpy.init()
    node = Node('octomap_capture')
    owp = fetch_octomap(node, rclpy, GetPlanningScene, PlanningSceneComponents)

    oct_msg = owp.octomap
    print(f'octomap id="{oct_msg.id}" binary={oct_msg.binary} '
          f'resolution={oct_msg.resolution} frame="{oct_msg.header.frame_id}" '
          f'data={len(oct_msg.data)}바이트')
    if not oct_msg.data:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit('octomap이 비어 있다 — 아직 형성되지 않았거나 클라우드가 안 들어온다')

    leaves = decode_msg(oct_msg)
    summarize(leaves, oct_msg.resolution)

    payload = serialize_message(owp)
    with open(args.out, 'wb') as fh:
        fh.write(MAGIC)
        fh.write(struct.pack('<I', len(payload)))
        fh.write(payload)
    print(f'\n저장: {args.out} ({len(payload)}바이트 직렬화)')

    node.destroy_node()
    rclpy.shutdown()


def load_octomap_file(path):
    """파일 → OctomapWithPose. inject 외에 스윕 스크립트에서도 쓴다."""
    from rclpy.serialization import deserialize_message
    from octomap_msgs.msg import OctomapWithPose
    with open(path, 'rb') as fh:
        blob = fh.read()
    if not blob.startswith(MAGIC):
        raise ValueError(f'{path}: 헤더가 {MAGIC!r}가 아니다')
    size = struct.unpack('<I', blob[len(MAGIC):len(MAGIC) + 4])[0]
    body = blob[len(MAGIC) + 4:len(MAGIC) + 4 + size]
    return deserialize_message(body, OctomapWithPose)


def inject_octomap(node, rclpy, PlanningScene, owp, settle=25):
    """살아있는 move_group에 octomap을 주입한다.

    **주의**: occupancy_map_monitor가 새 클라우드를 받으면 이 값을 덮어쓴다.
    측정용 스택은 카메라·필터 노드 없이 띄워 클라우드 발행자가 없게 할 것.
    """
    if not hasattr(node, '_octo_scene_pub'):
        node._octo_scene_pub = node.create_publisher(PlanningScene,
                                                    '/planning_scene', 10)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.octomap = owp
    node._octo_scene_pub.publish(scene)
    for _ in range(settle):
        rclpy.spin_once(node, timeout_sec=0.05)


def clear_octomap(node, rclpy, timeout=15.0):
    """씬의 octomap을 비운다 — `/clear_octomap` 서비스를 쓴다.

    **빈 OctomapWithPose를 diff로 발행하는 방법은 안 된다.**
    `PlanningScene::setPlanningSceneDiffMsg`가
        if (!world.collision_objects.empty() || !world.octomap.octomap.data.empty())
    로 걸러서, data가 비어 있으면 world 처리를 아예 호출하지 않는다. 즉 "빈
    octomap을 넣는다"는 조용히 무시되고 **직전 octomap이 그대로 남는다** —
    씬을 바꿨다고 믿으며 옛 장애물로 재게 되는, 함정 3과 똑같은 사고가 된다.
    """
    from std_srvs.srv import Empty
    if not hasattr(node, '_clear_client'):
        node._clear_client = node.create_client(Empty, '/clear_octomap')
        if not node._clear_client.wait_for_service(timeout_sec=timeout):
            raise SystemExit('/clear_octomap 없음 — move_group이 떠 있는지 확인할 것')
    fut = node._clear_client.call_async(Empty.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)


def cmd_clear(args):
    (rclpy, Node, PlanningScene, PlanningSceneComponents, GetPlanningScene,
     OctomapWithPose, _, _d) = _ros_imports()
    rclpy.init()
    node = Node('octomap_clear')
    clear_octomap(node, rclpy)
    got = fetch_octomap(node, rclpy, GetPlanningScene, PlanningSceneComponents)
    print(f'clear 후: {len(got.octomap.data)}바이트')
    node.destroy_node()
    rclpy.shutdown()


def cmd_inject(args):
    (rclpy, Node, PlanningScene, PlanningSceneComponents, GetPlanningScene,
     OctomapWithPose, _, _d) = _ros_imports()
    owp = load_octomap_file(args.path)
    leaves = decode_msg(owp.octomap)
    summarize(leaves, owp.octomap.resolution, label='파일: ')

    rclpy.init()
    node = Node('octomap_inject')

    # 주입이 실제로 남았는지 되읽어 확인한다. 덮어쓰는 경로가 둘이다:
    #   (1) occupancy_map_monitor — 클라우드 발행자가 있으면 새 클라우드로 갈아친다
    #   (2) 앞선 프로세스가 쏜 `/clear_octomap` — 서비스 호출이 비동기라
    #       **우리 주입 뒤에 도착할 수 있다.** 실제로 이것 때문에 정상적인
    #       파일이 "거부됐다"고 한 번 오진했다.
    # (2)는 재시도로 넘어가므로 몇 번 다시 쏜다.
    n_in = len(owp.octomap.data)
    for attempt in range(1, 4):
        inject_octomap(node, rclpy, PlanningScene, owp)
        got = fetch_octomap(node, rclpy, GetPlanningScene, PlanningSceneComponents)
        n_out = len(got.octomap.data)
        if n_out == n_in:
            break
        print(f'  주입 {attempt}회차 실패({n_out}바이트) — 재시도')
    print(f'\n주입 후 확인: {n_out}바이트 (원본 {n_in}바이트) — '
          + ('일치' if n_in == n_out else '**불일치, 덮어써졌다**'))
    node.destroy_node()
    rclpy.shutdown()
    if n_in != n_out:
        sys.exit(1)


# 실물 작업대의 벽(사용자 실측, 2026-08-02). g_base 기준:
# 오른쪽(-Y)에 서 있고, 높이 150mm, X로 베이스 뒤 500mm부터 베이스 앞까지.
# 두께는 실측이 없어 10mm로 둔다 — voxel 해상도(20mm)보다 얇아도
# box_leaves가 최소 한 겹은 채운다.
#
# **거리 100mm는 g_base 원점이 아니라 "베이스 스테이지 끝"에서 잰 값이다.**
# 스테이지 반폭을 100mm로 보아 y = -(0.100 + 0.100) = -0.20으로 둔다.
# 처음에 원점 기준으로 오해해 -0.10에 두었더니 **look pose와 armed pose가
# 둘 다 팔꿈치(joint3)부터 벽에 박혀** 15개 목표가 전부 플래닝 불가였다.
#
# /check_state_validity로 잰 경계(높이 150mm 기준):
#     y=-0.10  look 충돌 / armed 충돌(joint3,joint4)
#     y=-0.14  look OK   / armed 충돌(joint3,joint4)
#     y=-0.18  look OK   / armed 충돌(joint5)
#     y=-0.20  look OK   / armed OK      <- 두 자세가 성립하는 최소 거리
# 높이로 대신 맞출 수도 있다 — y=-0.10이면 벽이 80mm 이하여야 둘 다 OK다.
#
# 즉 **이 값은 armed pose와 직접 결합돼 있다.** armed pose는 look pose에서
# J1을 +40도 돌린 자세인데, 팔이 뒤로 접혀 있어 그 회전이 팔을 -Y로 89mm
# 밀어낸다(flange y=-0.035 -> -0.124). 벽을 더 당기거나 armed pose의 J1을
# 키우려면 이 검사를 다시 할 것.
WALL_Y_M = -0.20
WALL_THICKNESS_M = 0.01
WALL_X_MIN_M = -0.50
WALL_X_MAX_M = 0.0
WALL_Z_TOP_M = 0.15


def wall_box():
    """벽의 (xyz_min, xyz_max). 상수를 한 곳에서만 읽게 한다."""
    half = WALL_THICKNESS_M / 2.0
    return ((WALL_X_MIN_M, WALL_Y_M - half, 0.0),
            (WALL_X_MAX_M, WALL_Y_M + half, WALL_Z_TOP_M))


# [2026-08-02] 데모용 장애물 — **베드와 수확통 사이**를 가로막는 기둥.
#
# 목적이 벽과 다르다. 벽은 "실측한 작업대가 팔에 영향을 주는가"를 재려고 넣은
# 것이고(영향 없음이 결론이었다), 이쪽은 **있을 때와 없을 때 경로가 달라지는
# 것을 보여주려고** 일부러 길목에 세우는 것이다.
#
# 위치 제약이 양쪽에서 온다:
#   - 베드 쪽 정렬 자세(flange 반경 0.13~0.175, 방위 -14~+20도)를 막으면 안 된다.
#     막으면 "돌아간다"가 아니라 "못 간다"가 되어 데모가 아니다.
#   - 통 자세(그리퍼 끝단 (-0.006, +0.135, +0.129))를 막아도 같은 이유로 안 된다.
# 그래서 둘 **사이**를 고른다: x는 통(0)과 베드(0.25) 사이, y는 정렬 자세
# (+0.06 이하)보다 바깥이고 통(+0.135)에 닿지 않는 구간.
OBSTACLE_X_MIN_M = 0.06
OBSTACLE_X_MAX_M = 0.14
OBSTACLE_Y_MIN_M = 0.08
OBSTACLE_Y_MAX_M = 0.16
OBSTACLE_Z_TOP_M = 0.30


def obstacle_box():
    """데모 장애물의 (xyz_min, xyz_max)."""
    return ((OBSTACLE_X_MIN_M, OBSTACLE_Y_MIN_M, 0.0),
            (OBSTACLE_X_MAX_M, OBSTACLE_Y_MAX_M, OBSTACLE_Z_TOP_M))


def merge_box(owp, lo, hi, verbose=True):
    """octomap 메시지에 축정렬 상자를 voxel로 합친 **새 메시지**를 돌려준다.

    파일로 굳히는 경로(add-box), 살아있는 씬에 바로 넣는 경로(obstacle add),
    스윕이 주입 직전에 합치는 경로가 **같은 코드**를 쓰게 하려고 뺐다. 셋이
    갈리면 "파일로 만든 장애물"과 "화면에 뜬 장애물"이 조용히 달라진다.
    """
    import copy

    out = copy.deepcopy(owp)
    res = out.octomap.resolution
    base = decode_msg(out.octomap)
    added = box_leaves(lo, hi, res)
    # 중복 제거 — 이미 그 자리에 voxel이 있으면 한 번만 쓴다.
    have = {(round(l[0] / res), round(l[1] / res), round(l[2] / res))
            for l in base}
    fresh = [l for l in added
             if (round(l[0] / res), round(l[1] / res), round(l[2] / res)) not in have]
    merged = base + fresh
    if verbose:
        print(f'  상자 voxel {len(added)}개 중 새로 추가 {len(fresh)}개')

    out.octomap.binary = False
    out.octomap.id = 'OcTree'
    blob = encode_octree(merged, res)
    # Octomap.data는 int8[]이라 128~255를 그대로 넣으면 OverflowError가 난다.
    out.octomap.data = [b - 256 if b > 127 else b for b in blob]
    # 되읽어 검산한다 — 인코더가 틀리면 move_group이 조용히 무시하거나 죽는다.
    check = decode_octree(blob, res, False)
    n_expanded = sum(max(1, round(l[3] / res)) ** 3 for l in merged)
    if len(check) != n_expanded:
        raise SystemExit(f'인코딩 검산 실패: {n_expanded} -> {len(check)}')
    return out, check


def save_octomap_file(owp, path):
    """OctomapWithPose -> 파일. load_octomap_file의 짝이고 형식도 같다."""
    from rclpy.serialization import serialize_message
    payload = serialize_message(owp)
    with open(path, 'wb') as fh:
        fh.write(MAGIC)
        fh.write(struct.pack('<I', len(payload)))
        fh.write(payload)
    print(f'저장 완료 ({len(payload)}바이트)')


def cmd_add_box(args):
    owp = load_octomap_file(args.path)
    res = owp.octomap.resolution
    print(f'입력: {args.path}')
    summarize(decode_msg(owp.octomap), res, label='  ')

    if args.wall:
        lo, hi = wall_box()
        label = '벽 프리셋'
    elif args.obstacle:
        lo, hi = obstacle_box()
        label = '장애물 프리셋(베드-통 사이)'
    else:
        lo, hi = tuple(args.xyz_min), tuple(args.xyz_max)
        label = '상자'
    print(f'\n{label}: x {lo[0]:+.2f}~{hi[0]:+.2f}, y {lo[1]:+.3f}~{hi[1]:+.3f}, '
          f'z {lo[2]:.2f}~{hi[2]:.2f}')

    merged_owp, check = merge_box(owp, lo, hi)
    print(f'\n출력: {args.out}')
    summarize(check, res, label='  ')
    save_octomap_file(merged_owp, args.out)


def cmd_obstacle(args):
    """[2026-08-02] **살아있는 씬**에 데모 장애물을 넣고 뺀다.

    파일을 새로 만들지 않는다 — base를 읽어 메모리에서 상자를 합쳐 주입(add)
    하거나, base만 다시 주입(remove)한다. RViz를 켜 둔 채 한 줄로 넣었다 뺐다
    할 수 있어야 "있을 때 vs 없을 때"를 눈으로 비교할 수 있다.

    주의: `--octomap-acm`(그리퍼/손목 완화)을 켠 상태에서는 이 장애물을
    **손목까지 통과한다**(문서 6.3절). 회피를 보여주려면 완화 없이 볼 것.
    """
    (rclpy, Node, PlanningScene, PlanningSceneComponents, GetPlanningScene,
     _OctomapWithPose, _a, _b) = _ros_imports()

    # [2026-08-02] **실물에서는 이쪽을 쓴다.** octomap voxel로 넣으면 D435
    # 클라우드 갱신과 coord_to_goal_node의 /clear_octomap에 지워진다(문서 5.6b).
    # collision object는 둘 다에 안 지워진다.
    if args.as_object:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import scene_objects
        # [2026-08-03] 기둥과 벽을 각각/같이 넣을 수 있다. **id가 달라야**
        # 하나만 뺄 수 있다 — 같은 id로 두 번 쏘면 나중 것이 앞 것을 덮는다.
        wanted = {'pillar': [('pillar', obstacle_box(),
                              scene_objects.OBSTACLE_OBJECT_ID)],
                  'wall': [('wall', wall_box(), scene_objects.WALL_OBJECT_ID)]}
        wanted['both'] = wanted['pillar'] + wanted['wall']
        items = wanted[args.preset]
        if args.action == 'add' and (args.xyz_min != [0, 0, 0]
                                     or args.xyz_max != [0, 0, 0]):
            items = [('직접지정', (tuple(args.xyz_min), tuple(args.xyz_max)),
                      scene_objects.OBSTACLE_OBJECT_ID)]

        rclpy.init()
        node = Node('obstacle_object')
        for name, (lo, hi), oid in items:
            if args.action == 'add':
                print(f'collision object 추가[{name}]: x {lo[0]:+.2f}~{hi[0]:+.2f}, '
                      f'y {lo[1]:+.3f}~{hi[1]:+.3f}, z {lo[2]:.2f}~{hi[2]:.2f}')
            else:
                print(f'collision object 제거[{name}]')
            scene_objects.publish_obstacle(node, lo, hi,
                                           remove=(args.action == 'remove'),
                                           object_id=oid)
        # 되읽어 확인한다 — 발행이 디스커버리 전에 나가면 조용히 사라진다.
        client = node.create_client(GetPlanningScene, '/get_planning_scene')
        client.wait_for_service(timeout_sec=10)
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
        ids = set(o.id for o in fut.result().scene.world.collision_objects)
        want = (args.action == 'add')
        ok = True
        for name, _box, oid in items:
            here = oid in ids
            ok = ok and (here == want)
            print(f'씬 확인[{name}]: {"있음" if here else "없음"} '
                  + ('(의도대로)' if here == want else '**의도와 다름**'))
        here = ok
        node.destroy_node()
        rclpy.shutdown()
        if not ok:
            sys.exit(1)
        return

    owp = load_octomap_file(args.base)
    res = owp.octomap.resolution
    print(f'base: {args.base}')
    summarize(decode_msg(owp.octomap), res, label='  ')

    if args.action == 'add':
        if args.xyz_min != [0, 0, 0] or args.xyz_max != [0, 0, 0]:
            lo, hi = tuple(args.xyz_min), tuple(args.xyz_max)
        else:
            lo, hi = obstacle_box()
        print(f'\n장애물 추가: x {lo[0]:+.2f}~{hi[0]:+.2f}, '
              f'y {lo[1]:+.2f}~{hi[1]:+.2f}, z {lo[2]:.2f}~{hi[2]:.2f}')
        owp, leaves = merge_box(owp, lo, hi)
    else:
        print('\n장애물 제거 — base만 다시 주입한다')
        leaves = decode_msg(owp.octomap)

    rclpy.init()
    node = Node('octomap_obstacle')
    # 주입이 남았는지 되읽고 재시도한다 — 앞 프로세스가 부른 /clear_octomap이
    # **비동기라 우리 주입 뒤에 도착할 수 있다**(함정 13).
    n_in = len(owp.octomap.data)
    n_out = -1
    for attempt in range(1, 4):
        inject_octomap(node, rclpy, PlanningScene, owp)
        got = fetch_octomap(node, rclpy, GetPlanningScene, PlanningSceneComponents)
        n_out = len(got.octomap.data)
        if n_out == n_in:
            break
        print(f'  주입 {attempt}회차 실패({n_out}바이트) — 재시도')
    print(f'\n씬 확인: voxel {len(leaves)}개, {n_out}바이트 '
          + ('(일치)' if n_in == n_out else '**불일치, 덮어써졌다**'))
    node.destroy_node()
    rclpy.shutdown()
    if n_in != n_out:
        sys.exit(1)


def cmd_stats(args):
    dets = []
    if args.targets and os.path.exists(args.targets):
        import json
        with open(args.targets, encoding='utf-8') as fh:
            dets = json.load(fh)['detections']

    if args.path:
        owp = load_octomap_file(args.path)
        oct_msg = owp.octomap
        print(f'파일: {args.path}')
    else:
        (rclpy, Node, PlanningScene, PlanningSceneComponents, GetPlanningScene,
         OctomapWithPose, _, _d) = _ros_imports()
        rclpy.init()
        node = Node('octomap_stats')
        owp = fetch_octomap(node, rclpy, GetPlanningScene, PlanningSceneComponents)
        oct_msg = owp.octomap
        node.destroy_node()
        rclpy.shutdown()
        print('살아있는 planning scene:')

    print(f'  id="{oct_msg.id}" binary={oct_msg.binary} '
          f'resolution={oct_msg.resolution} frame="{oct_msg.header.frame_id}" '
          f'data={len(oct_msg.data)}바이트')
    if not oct_msg.data:
        print('  비어 있음')
        return
    leaves = decode_msg(oct_msg)
    summarize(leaves, oct_msg.resolution, label='  ')
    near_targets_report(leaves, dets)


def main():
    p = argparse.ArgumentParser(description='planning scene octomap 저장/주입/분석')
    sub = p.add_subparsers(dest='cmd', required=True)

    c = sub.add_parser('capture', help='살아있는 씬의 octomap을 파일로 저장')
    c.add_argument('--out', default='bags/bed_look_octomap.bin')
    c.set_defaults(func=cmd_capture)

    i = sub.add_parser('inject', help='파일의 octomap을 살아있는 씬에 주입')
    i.add_argument('path')
    i.set_defaults(func=cmd_inject)

    s = sub.add_parser('stats', help='octomap 내용 요약(파일 또는 살아있는 씬)')
    s.add_argument('path', nargs='?', default=None)
    s.add_argument('--targets', default='bags/lab_bed_detections.json')
    s.set_defaults(func=cmd_stats)

    cl = sub.add_parser('clear', help='살아있는 씬의 octomap을 비운다')
    cl.set_defaults(func=cmd_clear)

    ab = sub.add_parser('add-box',
                        help='octomap 파일에 축정렬 상자를 voxel로 합쳐 새 파일 생성')
    ab.add_argument('path')
    ab.add_argument('out')
    ab.add_argument('--obstacle', action='store_true',
                    help='데모 장애물 프리셋(베드-통 사이 기둥)')
    ab.add_argument('--wall', action='store_true',
                    help='실측 벽 프리셋을 쓴다(y=-0.10, x -0.50~0, z 0~0.15)')
    ab.add_argument('--xyz-min', nargs=3, type=float, default=[0, 0, 0])
    ab.add_argument('--xyz-max', nargs=3, type=float, default=[0, 0, 0])
    ab.set_defaults(func=cmd_add_box)

    ob = sub.add_parser(
        'obstacle', help='살아있는 씬에 데모 장애물을 넣고 뺀다(베드-통 사이 기둥)')
    ob.add_argument('action', choices=['add', 'remove'])
    ob.add_argument('--base', default='bags/bed_look_octomap_wall.bin',
                    help='장애물을 얹을 바탕 octomap 파일')
    ob.add_argument('--xyz-min', nargs=3, type=float, default=[0, 0, 0],
                    help='프리셋 대신 직접 지정(add일 때만)')
    ob.add_argument('--xyz-max', nargs=3, type=float, default=[0, 0, 0])
    ob.add_argument('--preset', choices=['pillar', 'wall', 'both'],
                    default='pillar',
                    help='pillar=베드-통 사이 기둥, wall=베이스 오른쪽 벽'
                         '(y=-0.20, 높이 150mm), both=둘 다. --as-object에만 적용')
    ob.add_argument('--as-object', action='store_true',
                    help='octomap voxel 대신 **collision object**로 넣는다. '
                         '실물에서는 이쪽만 살아남는다(카메라 갱신과 '
                         '/clear_octomap에 안 지워진다)')
    ob.set_defaults(func=cmd_obstacle)

    args = p.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
