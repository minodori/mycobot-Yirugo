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

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_geometry_msgs import do_transform_point
import tf2_ros

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


def main():
    p = argparse.ArgumentParser(description='YOLO 토마토 후보를 g_base 좌표로 내보내기')
    p.add_argument('--out', default='bed_detections', help='출력 파일 이름(확장자 제외)')
    p.add_argument('--timeout', type=float, default=45.0, help='대기 시간(초)')
    args = p.parse_args()

    rclpy.init()
    node = ExportNode()
    node.get_logger().info(
        f'tomato_candidates 대기 중... (최대 {args.timeout:.0f}초) — 판단이 이미 '
        f'끝났다면 yolo_d435_detector_node를 재시작할 것'
    )
    deadline = node.get_clock().now().nanoseconds * 1e-9 + args.timeout
    while node.rows is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds * 1e-9 > deadline:
            break

    if node.rows is None:
        node.get_logger().error('후보를 받지 못했다. 위 주의사항 참고.')
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
