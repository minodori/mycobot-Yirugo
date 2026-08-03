#!/usr/bin/env python3
"""검출 좌표 파일을 RViz에 **번호를 붙여** 그린다.

왜 필요한가
-----------
`export_detections.py`가 뽑은 목록에서 "#4를 쓰자"고 정해도, 화면과 실물에서
그게 어느 열매인지 알 방법이 없었다. 좌표만 봐서는 좌우/높이를 머리로 환산해야
하고, 그러다 엉뚱한 열매를 기준으로 보정하면 그 보정이 전부 헛것이 된다.

이 스크립트는 목록 전체를 구로 그리고 그 옆에 **번호를 띄운다**. 고른 것만
색을 달리해 바로 구분되게 한다.

토픽은 `harvest_sequence_node`와 같은 `/harvest_targets`를 쓴다 — RViz 설정
(harvest_view.rviz)에 이미 그 디스플레이가 있어서 따로 추가할 필요가 없다.
수확 시퀀스를 시작하면 그쪽이 다시 그리므로 덮어써도 문제가 없다.

사용법
------
    python3 scripts/show_detections.py --targets bags/today.json --pick 4 7
    python3 scripts/show_detections.py --targets bags/today_fixed.json
"""

import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

BASE_LINK_NAME = 'g_base'

# 고른 것 = 노랑(크게), 나머지 = 어두운 회색. 라벨은 흰색.
PICKED_COLOR = (1.00, 0.85, 0.10, 1.00)
OTHER_COLOR = (0.40, 0.40, 0.40, 0.35)


def main():
    p = argparse.ArgumentParser(description='검출 목록을 번호와 함께 RViz에 표시')
    p.add_argument('--targets', default='bags/today.json')
    p.add_argument('--pick', nargs='*', type=int, default=[],
                   help='강조할 번호(1부터). 비우면 전부 같은 색')
    p.add_argument('--label-height', type=float, default=0.035,
                   help='번호를 구 위 몇 m에 띄울지')
    p.add_argument('--seconds', type=float, default=0.0,
                   help='이 시간 뒤 종료(0이면 Ctrl+C까지 유지)')
    args = p.parse_args()

    with open(args.targets, encoding='utf-8') as fh:
        dets = json.load(fh)['detections']
    picked = set(args.pick)

    rclpy.init()
    node = Node('show_detections')
    # latched — RViz를 나중에 켜도 보이게.
    pub = node.create_publisher(
        MarkerArray, '/harvest_targets',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    array = MarkerArray()
    for i, d in enumerate(dets, 1):
        on = (not picked) or (i in picked)
        r = float(d.get('radius_m', 0.017))

        sphere = Marker()
        sphere.header.frame_id = BASE_LINK_NAME
        sphere.header.stamp = node.get_clock().now().to_msg()
        sphere.ns = 'harvest_targets'
        sphere.id = i
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(d['base_x'])
        sphere.pose.position.y = float(d['base_y'])
        sphere.pose.position.z = float(d['base_z'])
        sphere.pose.orientation.w = 1.0
        scale = 2.0 * r * (1.5 if on and picked else 1.0)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = scale
        cr, cg, cb, ca = PICKED_COLOR if on else OTHER_COLOR
        sphere.color = ColorRGBA(r=cr, g=cg, b=cb, a=ca)
        array.markers.append(sphere)

        # 번호 라벨. **ASCII만 쓴다** — RViz 폰트 아틀라스가 33~166만 담아서
        # 한글이나 도 기호는 보이지 않으면서 자리만 차지한다(핸드오프 함정 14).
        text = Marker()
        text.header = sphere.header
        text.ns = 'harvest_labels'
        text.id = i
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(d['base_x'])
        text.pose.position.y = float(d['base_y'])
        text.pose.position.z = float(d['base_z']) + args.label_height
        text.pose.orientation.w = 1.0
        text.scale.z = 0.022
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0 if on else 0.5)
        text.text = f'#{i}'
        array.markers.append(text)

    print(f'{args.targets}: {len(dets)}개 표시'
          + (f', 강조 {sorted(picked)}' if picked else ''))
    for i, d in enumerate(dets, 1):
        if picked and i not in picked:
            continue
        side = '왼쪽' if d['base_y'] > 0 else '오른쪽'
        print(f"  #{i} {d.get('class_name', '?'):8s} {side} y={d['base_y']:+.3f} "
              f"z={d['base_z']:.3f} r={d.get('radius_m', 0) * 1000:.1f}mm")

    # latched지만 RViz가 붙을 때까지 몇 번 더 쏜다(디스커버리 여유).
    import time
    end = time.time() + (args.seconds if args.seconds > 0 else 1e9)
    published = 0
    while rclpy.ok() and time.time() < end:
        if published < 5:
            pub.publish(array)
            published += 1
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
