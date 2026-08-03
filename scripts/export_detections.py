#!/usr/bin/env python3
"""YOLO가 look pose에서 판단한 토마토 후보를 g_base 좌표로 변환해 파일로 뽑는다.

왜 필요한가
-----------
목표는 **로봇도 베드도 없는 노트북에서** 수확 경로를 시뮬레이션하는 것이다
(지점마다 수확 가능 여부, 관절 상태, 계획 품질). 그러려면 "실험실 베드에
토마토가 어디에 어떤 상태로 있었나"가 고정된 데이터로 남아야 한다.

rosbag(트랙 A)은 장면을 통째로 남기지만 재생 스택을 띄워야 쓸 수 있다. 이
스크립트는 그중 **판단 결과만** 뽑아 JSON/CSV로 굳힌다 — ROS 없이도 읽히고,
오프라인 스윕(scripts/sweep_bed_offline.py)의 목표 목록으로 바로 쓸 수 있으며,
RViz에 마커로 띄워 보고서용 그림을 만들 수도 있다.

무엇이 담기나
-------------
`tomato_candidates`(yolo_d435_detector_node)의 클러스터마다:
  class_id / class_name   ripe · unripe · rotten · disease
  camera_xyz              camera_color_optical_frame 기준 3D 좌표(m)
  base_xyz                g_base 기준 3D 좌표(m) — 플래닝에 쓰는 좌표계
  radius_m                bbox 픽셀 크기를 핀홀 역산한 추정 반지름
  confidence              클러스터 평균 confidence
  count                   누적 구간 동안 관측된 횟수(신뢰도 대용)

주의
----
- `tomato_candidates`는 팔이 look pose에 있을 때 ACCUMULATION_WINDOW_SEC(5초)
  누적이 끝나는 **순간 1회만** 발행된다. 이미 판단이 끝난 뒤라면 이 스크립트가
  기다려도 안 온다 — yolo_d435_detector_node를 재시작하면 다시 판단한다.
- 좌표 변환은 **팔이 look pose에 있는 동안** 해야 한다. camera_link가 joint6에
  얹힌 eye-in-hand라, 팔이 움직인 뒤 변환하면 좌표가 틀어진다.

사용법
------
    python3 scripts/export_detections.py                       # bed_detections.json
    python3 scripts/export_detections.py --out lab_bed --timeout 60
"""

import argparse
import csv
import json
import math

# [2026-08-02] ROS import를 **선택적**으로 둔다 — `--compare`는 파일 둘만 읽는
# 모드라 ROS가 없는 자리(보정본을 손보는 노트북 등)에서도 돌아야 한다. 없으면
# Node 정의를 건너뛰고, 실제로 필요할 때(main의 수집 경로) 이유를 말하며 멈춘다.
try:
    from geometry_msgs.msg import PointStamped
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
    from tf2_geometry_msgs import do_transform_point
    import tf2_ros
    HAVE_ROS = True
except ImportError as _exc:      # noqa: F841 — 메시지는 main에서 쓴다
    HAVE_ROS = False
    _ROS_IMPORT_ERROR = _exc
    # 아래 class 정의(어노테이션 포함)가 import 시점에 깨지지 않도록 자리만 채운다.
    Node = object
    PointStamped = Float32MultiArray = object

BASE_LINK_NAME = 'g_base'
CAMERA_FRAME = 'camera_color_optical_frame'
# yolo_d435_detector_node의 모델 클래스 순서(model.names)와 동일.
NAME_BY_CLASS_ID = {0: 'ripe', 1: 'unripe', 2: 'rotten', 3: 'disease'}
# tomato_candidates 한 항목의 길이: class_id, x, y, z, confidence, radius_m, count
STRIDE = 7


class ExportNode(Node):

    def __init__(self):
        super().__init__('export_detections')
        self.rows = None
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self.create_subscription(
            Float32MultiArray, 'tomato_candidates', self._on_candidates, 10
        )

    def _on_candidates(self, msg: Float32MultiArray) -> None:
        if self.rows is not None:
            return
        data = list(msg.data)
        if not data:
            # look pose를 벗어나면 stale 방지용 빈 배열이 온다. 판단 결과가 아니다.
            return
        try:
            transform = self._buffer.lookup_transform(
                BASE_LINK_NAME, CAMERA_FRAME, rclpy.time.Time()
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f'TF 변환 실패, 다음 메시지를 기다림: {exc}')
            return

        rows = []
        for i in range(0, len(data) - len(data) % STRIDE, STRIDE):
            class_id, x, y, z, conf, radius_m, count = data[i:i + STRIDE]
            point = PointStamped()
            point.header.frame_id = CAMERA_FRAME
            point.point.x, point.point.y, point.point.z = x, y, z
            based = do_transform_point(point, transform).point
            rows.append({
                'class_id': int(class_id),
                'class_name': NAME_BY_CLASS_ID.get(int(class_id), f'id{int(class_id)}'),
                'camera_x': round(x, 4),
                'camera_y': round(y, 4),
                'camera_z': round(z, 4),
                'base_x': round(based.x, 4),
                'base_y': round(based.y, 4),
                'base_z': round(based.z, 4),
                'base_radius': round(math.hypot(based.x, based.y), 4),
                'base_azimuth_deg': round(math.degrees(math.atan2(based.y, based.x)), 2),
                'radius_m': round(radius_m, 4),
                'confidence': round(conf, 4),
                'count': int(count),
            })
        self.rows = rows


def compare_files(path_a, path_b, max_pair_m=0.10):
    """[2026-08-02] 원본 검출과 **사람이 보정한 파일**의 차이를 표로 찍는다.

    왜 필요한가 — YOLO 좌표는 실제 파지에서 오차가 있어(bbox depth가 물체의
    카메라 쪽 표면) 사람이 손으로 고친 파일을 쓰는 경로가 있다
    (harvest_sequence_node의 target_source:=file). 그런데 손으로 고치다 보면
    **단위(m/mm)와 부호**를 틀리기 쉽고, 틀려도 파일은 멀쩡해 보인다. 실제로
    로봇을 움직여 보기 전에 여기서 걸러야 한다.

    짝짓기는 **가장 가까운 것끼리**다(순서가 바뀌어도 된다). max_pair_m보다 멀면
    짝이 없는 것으로 본다 — 그 자체가 "너무 많이 옮겼다"는 신호다.
    """
    def load(path):
        with open(path, encoding='utf-8') as fh:
            return json.load(fh)['detections']

    a, b = load(path_a), load(path_b)
    print(f'원본  {path_a}: {len(a)}개')
    print(f'보정본 {path_b}: {len(b)}개\n')

    def xyz(d):
        return (d['base_x'], d['base_y'], d['base_z'])

    used = set()
    print(f'{"#":>2} {"class":<8}{"원본 xyz(m)":>26}'
          f'{"dx(mm)":>9}{"dy(mm)":>9}{"dz(mm)":>9}{"거리(mm)":>11}')
    unmatched_b = list(range(len(b)))
    for i, da in enumerate(a, 1):
        pa = xyz(da)
        best, best_d = None, None
        for j, db in enumerate(b):
            if j in used:
                continue
            d = math.dist(pa, xyz(db))
            if best_d is None or d < best_d:
                best, best_d = j, d
        if best is None or best_d > max_pair_m:
            print(f'{i:>2} {da.get("class_name", "?"):<8}'
                  f'({pa[0]:+.3f},{pa[1]:+.3f},{pa[2]:+.3f})'
                  f'{"  — 보정본에 짝 없음":>40}')
            continue
        used.add(best)
        unmatched_b.remove(best)
        pb = xyz(b[best])
        dx, dy, dz = (1000 * (pb[k] - pa[k]) for k in range(3))
        print(f'{i:>2} {da.get("class_name", "?"):<8}'
              f'({pa[0]:+.3f},{pa[1]:+.3f},{pa[2]:+.3f})'
              f'{dx:+9.1f}{dy:+9.1f}{dz:+9.1f}{best_d * 1000:11.1f}')
    for j in unmatched_b:
        pb = xyz(b[j])
        print(f'   {b[j].get("class_name", "?"):<8}'
              f'({pb[0]:+.3f},{pb[1]:+.3f},{pb[2]:+.3f})'
              f'{"  <- 보정본에만 있음":>40}')

    print('\n단위는 mm다. 수십 mm를 넘는 값이 보이면 m/mm를 헷갈렸는지 확인할 것 '
          '— 이 표의 목적이 그거다.')


def main():
    p = argparse.ArgumentParser(description='YOLO 토마토 후보를 g_base 좌표로 내보내기')
    p.add_argument('--out', default='bed_detections', help='출력 파일 이름(확장자 제외)')
    p.add_argument('--timeout', type=float, default=45.0, help='대기 시간(초)')
    # [2026-08-02] ROS 없이 파일 둘만 비교하는 모드.
    p.add_argument('--compare', nargs=2, metavar=('원본.json', '보정본.json'),
                   default=None,
                   help='두 좌표 파일의 차이를 mm로 찍는다(ROS 불필요)')
    args = p.parse_args()

    if args.compare:
        compare_files(*args.compare)
        return

    if not HAVE_ROS:
        raise SystemExit(
            f'ROS2 환경이 없어 검출 수집을 할 수 없다({_ROS_IMPORT_ERROR}). '
            'source /opt/ros/jazzy/setup.bash 후 다시 실행할 것. '
            '(--compare는 ROS 없이도 된다)')

    rclpy.init()
    node = ExportNode()
    node.get_logger().info(
        f'tomato_candidates 대기 중... (최대 {args.timeout:.0f}초)')
    # [2026-08-03] 안 오는 이유가 하나가 아니다. 실물에서 60초를 기다렸다가
    # 빈손으로 끝난 적이 있어(판단이 얼린 채였고 팔은 look pose에 서 있었다),
    # 무엇을 확인해야 하는지 여기서 바로 알려준다.
    node.get_logger().info(
        '안 오면: (1) 판단이 얼려 있는지 — 수확 시퀀스를 emergency_stop으로 '
        '멈추면 얼린 채 남는다. '
        'ros2 service call /yolo_d435_detector_node/set_judgment_enabled '
        'std_srvs/srv/SetBool "{data: true}" 로 풀면 look pose에 서 있는 채로도 '
        '다시 누적한다. (2) 팔이 look pose에 있는지(게이트는 /joint_states로 '
        '판정한다). (3) 그래도 안 오면 yolo_d435_detector_node 재시작.')
    deadline = node.get_clock().now().nanoseconds * 1e-9 + args.timeout
    while node.rows is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds * 1e-9 > deadline:
            break

    if node.rows is None:
        node.get_logger().error(
            '후보를 받지 못했다 — 위 세 가지를 순서대로 확인할 것. '
            '판단은 look pose에 **들어오는 순간** 1회만 돌므로, 이미 서 있는 '
            '상태로 기다리기만 해서는 영영 안 온다.')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    rows = node.rows
    keys = list(rows[0].keys())
    with open(f'{args.out}.json', 'w', encoding='utf-8') as fh:
        json.dump({'frame': BASE_LINK_NAME, 'detections': rows}, fh,
                  indent=2, ensure_ascii=False)
    with open(f'{args.out}.csv', 'w', newline='', encoding='utf-8') as fh:
        writer = csv.DictWriter(fh, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)

    print(f'\n{len(rows)}개 저장: {args.out}.json / {args.out}.csv (frame={BASE_LINK_NAME})\n')
    print(f'{"클래스":<9}{"g_base x":>10}{"y":>9}{"z":>9}{"반지름m":>9}{"반경m":>8}'
          f'{"방위°":>8}{"conf":>7}{"관측":>6}')
    for r in sorted(rows, key=lambda v: -v['confidence']):
        print(f'{r["class_name"]:<9}{r["base_x"]:10.3f}{r["base_y"]:9.3f}{r["base_z"]:9.3f}'
              f'{r["radius_m"]:9.3f}{r["base_radius"]:8.3f}{r["base_azimuth_deg"]:8.1f}'
              f'{r["confidence"]:7.2f}{r["count"]:6d}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
