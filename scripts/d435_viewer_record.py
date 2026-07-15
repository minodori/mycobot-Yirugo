#!/usr/bin/env python3
"""
D435 RGB + Depth 뷰어 + bag 녹화 스크립트

기능:
  - 왼쪽: RGB 스트림, 오른쪽: Depth 컬러맵 스트림 (화면 확인용)
  - 동시에 원본 데이터(컬러+뎁스+intrinsics)를 .bag 파일로 저장
    (화면 캡처가 아니라 pyrealsense2 내장 레코딩이라 뎁스 원본 값이 보존됨)

사전 준비:
  pip install pyrealsense2 opencv-python numpy --break-system-packages

사용법:
  python3 d435_viewer_record.py                    # 녹화 없이 뷰어만
  python3 d435_viewer_record.py --out tomato_bed.bag  # 뷰어 + 녹화

조작:
  q 또는 ESC : 종료
"""

import argparse
import time
import numpy as np
import cv2
import pyrealsense2 as rs


def main():
    parser = argparse.ArgumentParser(description="D435 RGB+Depth 뷰어/녹화")
    parser.add_argument(
        "--out",
        type=str,
        default=None,
        help="저장할 .bag 파일 경로 (지정 안 하면 녹화 없이 뷰어만 실행)",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    pipeline = rs.pipeline()
    config = rs.config()

    # 컬러 + 뎁스 스트림 설정 (동기화됨)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)

    # 녹화 파일 지정 시, 원본 스트림 그대로 .bag에 기록
    if args.out is not None:
        config.enable_record_to_file(args.out)
        print(f"[INFO] 녹화 시작: {args.out}")

    profile = pipeline.start(config)

    # 실제 협상된 USB 연결 등급 (예: "3.2", "2.1") - 케이블/포트가 USB3 규격인지 확인하는 가장 확실한 방법
    device = profile.get_device()
    try:
        usb_type = device.get_info(rs.camera_info.usb_type_descriptor)
    except RuntimeError:
        usb_type = "?"
    is_usb3 = usb_type.startswith("3")

    # 컬러-뎁스 정렬 (컬러 픽셀 <-> 뎁스 픽셀 1:1 매칭용, 좌표 변환 단계에서 필요)
    align = rs.align(rs.stream.color)

    window_name = "D435 - RGB (left) / Depth (right)"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    # 대역폭 측정용 롤링 윈도우 (1초마다 갱신)
    window_start = time.time()
    window_bytes = 0
    window_frames = 0
    measured_fps = 0.0
    measured_mbps = 0.0

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # 뎁스는 시각화를 위해서만 컬러맵 입힘 (저장되는 원본과는 별개)
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
            )

            combined = np.hstack((color_image, depth_colormap))

            # 프레임당 디코딩된 데이터량 누적 (실제 USB 와이어 전송량과는 다를 수 있는 근사치)
            window_bytes += color_image.nbytes + depth_image.nbytes
            window_frames += 1
            elapsed = time.time() - window_start
            if elapsed >= 1.0:
                measured_fps = window_frames / elapsed
                measured_mbps = (window_bytes / elapsed) / (1024 * 1024)
                window_start = time.time()
                window_bytes = 0
                window_frames = 0

            usb_color = (0, 255, 0) if is_usb3 else (0, 0, 255)
            lines = [
                (f"Res: {args.width}x{args.height} @ {args.fps}fps (set) / {measured_fps:.1f}fps (measured)", (255, 255, 255)),
                (f"Data rate: {measured_mbps:.1f} MB/s (decoded, wire rate may differ)", (255, 255, 255)),
                (f"USB: {usb_type}" + ("" if is_usb3 else "  <- USB3 cable/port 확인 필요"), usb_color),
            ]
            for i, (text, color) in enumerate(lines):
                y = 25 + i * 22
                cv2.putText(combined, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(combined, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

            cv2.imshow(window_name, combined)

            key = cv2.waitKey(1)
            if key in (ord("q"), 27):  # q 또는 ESC
                break
            # 창의 X 버튼으로 닫혔는지 확인 (waitKey는 이걸 감지 못 함)
            # Qt 백엔드는 창이 닫히면 guiReceiver가 NULL이 되어 값 대신 예외를 던지기도 함
            try:
                visible = cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE)
            except cv2.error:
                break
            if visible < 1:
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        if args.out is not None:
            print(f"[INFO] 녹화 완료: {args.out}")


if __name__ == "__main__":
    main()