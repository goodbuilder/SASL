from math import cos, radians, sin

from wiris_stereo_distance.calibration import CameraIntrinsics
from wiris_stereo_distance.triangulation import (
    BODY_FRAME,
    CAMERA_FRAME,
    CAMERA_MOUNT_PROFILE,
    WORLD_FRAME,
    body_frd_vector_to_camera,
    camera_translation_right_in_left_camera,
    camera_vector_to_body_frd,
    invert_left_to_right_camera_transform,
    normalized_ray_from_pixel,
    stereo_center_point_from_left_camera,
    triangulate_pixels,
    triangulate_pixels_with_pixhawk_mount,
    triangulate_rays,
    world_ned_vector_to_body_frd,
)


def yaw_quaternion(deg: float) -> tuple[float, float, float, float]:
    half = radians(deg) * 0.5
    return (cos(half), 0.0, 0.0, sin(half))


def rounded_tuple(values: tuple[float, float, float], digits: int = 6) -> tuple[float, float, float]:
    return tuple(round(value, digits) for value in values)


def intrinsics():
    return CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[],
        reprojection_error_px=0.1,
        image_count=20,
    )


def test_normalized_ray_from_pixel_uses_camera_matrix() -> None:
    ray = normalized_ray_from_pixel(640.0, 360.0, intrinsics())

    assert ray == (0.0, 0.0, 1.0)


def test_triangulate_rays_returns_expected_forward_distance_for_parallel_stereo() -> None:
    left_ray = normalized_ray_from_pixel(690.0, 360.0, intrinsics())
    right_ray = normalized_ray_from_pixel(640.0, 360.0, intrinsics())

    result = triangulate_rays(left_ray, right_ray, (0.3, 0.0, 0.0))

    assert result is not None
    assert round(result.distance_m, 1) == 5.4
    assert round(result.baseline_m, 1) == 0.3


def test_stereo_center_point_sets_centerline_estimated_x_to_zero() -> None:
    left_ray = normalized_ray_from_pixel(665.0, 360.0, intrinsics())
    right_ray = normalized_ray_from_pixel(615.0, 360.0, intrinsics())

    result = triangulate_rays(left_ray, right_ray, (0.3, 0.0, 0.0))

    assert result is not None
    center_point = stereo_center_point_from_left_camera(result.point_left_m, (0.3, 0.0, 0.0))
    assert rounded_tuple(result.point_left_m) == (0.15, 0.0, 5.4)
    assert rounded_tuple(center_point) == (0.0, 0.0, 5.4)


def test_triangulate_pixels_uses_intrinsics_attitude_and_translation() -> None:
    result = triangulate_pixels(
        left_pixel=(690.0, 360.0),
        right_pixel=(640.0, 360.0),
        left_intrinsics=intrinsics(),
        right_intrinsics=intrinsics(),
        left_q=(1.0, 0.0, 0.0, 0.0),
        right_q=(1.0, 0.0, 0.0, 0.0),
        translation_right_in_left_m=(0.3, 0.0, 0.0),
    )

    assert result is not None
    assert round(result.distance_m, 1) == 5.4


def test_fixed_camera_mount_maps_optical_axes_to_px4_frd() -> None:
    assert WORLD_FRAME == "ned"
    assert BODY_FRAME == "px4_frd"
    assert CAMERA_FRAME == "opencv_optical"
    assert CAMERA_MOUNT_PROFILE == "front_camera_z_to_body_x_x_to_body_y"
    assert camera_vector_to_body_frd((1.0, 0.0, 0.0)) == (0.0, 1.0, 0.0)
    assert camera_vector_to_body_frd((0.0, 1.0, 0.0)) == (0.0, 0.0, 1.0)
    assert camera_vector_to_body_frd((0.0, 0.0, 1.0)) == (1.0, 0.0, 0.0)
    assert body_frd_vector_to_camera((0.0, 1.0, 0.0)) == (1.0, 0.0, 0.0)
    assert body_frd_vector_to_camera((0.0, 0.0, 1.0)) == (0.0, 1.0, 0.0)
    assert body_frd_vector_to_camera((1.0, 0.0, 0.0)) == (0.0, 0.0, 1.0)


def test_world_ned_baseline_rotates_to_camera_frame_with_fixed_mount() -> None:
    left_body_vector = world_ned_vector_to_body_frd((0.0, 0.3, 0.0), (1.0, 0.0, 0.0, 0.0))

    assert body_frd_vector_to_camera(left_body_vector) == (0.3, 0.0, 0.0)


def test_camera_translation_uses_gnss_baseline_with_nominal_shared_rig() -> None:
    translation = camera_translation_right_in_left_camera(
        baseline_right_in_left_world_m=(0.0, 0.3, 0.0),
        left_body_to_world_q=(1.0, 0.0, 0.0, 0.0),
        right_body_to_world_q=yaw_quaternion(90.0),
    )

    assert translation is not None
    assert rounded_tuple(translation) == (0.3, 0.0, 0.0)


def test_pixhawk_mount_triangulation_matches_forward_camera_geometry() -> None:
    result = triangulate_pixels_with_pixhawk_mount(
        left_pixel=(690.0, 360.0),
        right_pixel=(640.0, 360.0),
        left_intrinsics=intrinsics(),
        right_intrinsics=intrinsics(),
        left_body_to_world_q=(1.0, 0.0, 0.0, 0.0),
        right_body_to_world_q=(1.0, 0.0, 0.0, 0.0),
        translation_right_in_left_m=(0.3, 0.0, 0.0),
    )

    assert result is not None
    assert round(result.distance_m, 1) == 5.4


def test_invert_checkerboard_left_to_right_transform_preserves_right_origin_contract() -> None:
    rotation_right_to_left, translation_right_in_left = invert_left_to_right_camera_transform(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [-0.3, 0.0, 0.0],
    )

    assert rotation_right_to_left == [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    assert rounded_tuple(translation_right_in_left) == (0.3, 0.0, 0.0)
