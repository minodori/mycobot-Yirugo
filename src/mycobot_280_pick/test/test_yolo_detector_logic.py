#!/usr/bin/env python3
"""
[Tier2/Tier3] yolo_d435_detector_node.py의 클러스터링/반지름 추정/look pose
판정 순수 함수에 대한 유닛 테스트. so101-ros-physical-ai 자매 프로젝트가 같은
게이팅+누적판단 재설계를 검증했던 방식(합성 검출 데이터로 핵심 함수를 직접
호출)을 그대로 따름 — rclpy 노드/YOLO 모델 로드 없이 순수 로직만 검증.
"""

from mycobot_280_pick.yolo_d435_detector_node import (
    _cluster_detections,
    _estimate_radius_m,
    _is_near_look_pose,
    CLUSTER_DISTANCE_M,
    JOINT_NAMES,
    LOOK_POSE_JOINT_POSITIONS,
    LOOK_POSE_TOLERANCE_RAD,
    MAX_ESTIMATED_RADIUS_M,
    MIN_ESTIMATED_RADIUS_M,
)


def test_cluster_merges_nearby_same_class_detections():
    detections = [
        (0, 0.100, 0.050, 0.300, 0.6),
        (0, 0.102, 0.048, 0.302, 0.9),
    ]
    clusters = _cluster_detections(detections)

    assert len(clusters) == 1
    assert clusters[0]['count'] == 2
    assert clusters[0]['x'] == (0.100 + 0.102) / 2
    assert clusters[0]['confidence'] == (0.6 + 0.9) / 2


def test_cluster_keeps_far_apart_same_class_separate():
    far_offset = CLUSTER_DISTANCE_M * 5
    detections = [
        (0, 0.100, 0.050, 0.300, 0.8),
        (0, 0.100 + far_offset, 0.050, 0.300, 0.8),
    ]
    clusters = _cluster_detections(detections)

    assert len(clusters) == 2
    assert all(c['count'] == 1 for c in clusters)


def test_cluster_keeps_different_class_separate_even_at_same_position():
    detections = [
        (0, 0.100, 0.050, 0.300, 0.8),
        (3, 0.100, 0.050, 0.300, 0.8),
    ]
    clusters = _cluster_detections(detections)

    assert len(clusters) == 2
    assert {c['class_id'] for c in clusters} == {0, 3}


def test_cluster_selection_prefers_observation_count_over_single_high_confidence():
    """so101에서 검증된 선택 기준: confidence 최댓값 단발이 아니라 관측
    횟수가 더 많은(안정적인) 클러스터를 우선 채택."""
    detections = [
        # 클러스터 A: 1회만 관측, confidence는 높음
        (0, 0.300, 0.000, 0.300, 0.95),
        # 클러스터 B: 2회 관측(멀리 떨어짐), 평균 confidence는 A보다 낮음
        (0, 0.000, 0.300, 0.300, 0.55),
        (0, 0.002, 0.302, 0.302, 0.65),
    ]
    clusters = _cluster_detections(detections)
    best = sorted(clusters, key=lambda c: (c['count'], c['confidence']), reverse=True)[0]

    assert best['count'] == 2
    assert best['confidence'] == (0.55 + 0.65) / 2


def test_estimate_radius_within_clamp_range():
    # 평범한 크기의 bbox(약 4cm 지름 물체가 0.3m 거리에 있는 정도로 역산)
    fx = fy = 600.0
    depth_m = 0.3
    bbox_px = 0.04 * fx / depth_m  # ~= 물체 지름(0.04m)에 해당하는 픽셀 크기
    radius = _estimate_radius_m(0.0, 0.0, bbox_px, bbox_px, depth_m, fx, fy)

    assert MIN_ESTIMATED_RADIUS_M <= radius <= MAX_ESTIMATED_RADIUS_M


def test_estimate_radius_clamps_extreme_bbox_sizes():
    fx = fy = 600.0
    depth_m = 0.3

    tiny_radius = _estimate_radius_m(0.0, 0.0, 1.0, 1.0, depth_m, fx, fy)
    assert tiny_radius == MIN_ESTIMATED_RADIUS_M

    huge_radius = _estimate_radius_m(0.0, 0.0, 500.0, 500.0, depth_m, fx, fy)
    assert huge_radius == MAX_ESTIMATED_RADIUS_M


def test_is_near_look_pose_true_within_tolerance():
    positions = dict(zip(JOINT_NAMES, LOOK_POSE_JOINT_POSITIONS))
    positions[JOINT_NAMES[0]] += LOOK_POSE_TOLERANCE_RAD * 0.5

    assert _is_near_look_pose(positions) is True


def test_is_near_look_pose_false_outside_tolerance():
    positions = dict(zip(JOINT_NAMES, LOOK_POSE_JOINT_POSITIONS))
    positions[JOINT_NAMES[0]] += LOOK_POSE_TOLERANCE_RAD * 2.0

    assert _is_near_look_pose(positions) is False


def test_is_near_look_pose_false_when_joint_missing():
    positions = dict(zip(JOINT_NAMES, LOOK_POSE_JOINT_POSITIONS))
    del positions[JOINT_NAMES[-1]]

    assert _is_near_look_pose(positions) is False
