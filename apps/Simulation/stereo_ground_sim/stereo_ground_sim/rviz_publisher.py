"""ROS visualization publisher for StereoDistanceNode.

All Marker/MarkerArray point coordinates are Gazebo world ENU metres. Optical
RDF basis vectors may be consumed by the pure geometry builder, but are never
serialized directly as marker positions. Public measurement topics remain
baseline-center FRD and are outside this publisher.
"""

import json
import math
import time

from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from stereo_ground_sim.rviz_geometry import camera_frustum_geometry, ray_line_geometry
from stereo_ground_sim.visualization_payloads import ray_metrics_document


class RvizVisualizationPublisher:
    def __init__(self, node, *, world_frame, ray_display_names):
        self.node = node
        self.world_frame = world_frame
        self.ray_display_names = dict(ray_display_names)

    def _publish_marker(self):
        with self.node.lock:
            metrics = dict(self.node.metrics)

        distance = metrics.get("distance_m")
        disparity = metrics.get("disparity_px")
        actual = metrics.get("actual_distance_m", self.node._actual_distance())
        error = metrics.get("distance_error_m")
        estimated_right = metrics.get("estimated_right_m")
        estimated_down = metrics.get("estimated_down_m")
        estimated_forward = metrics.get("estimated_forward_m")
        phase = metrics.get("mission_phase", "unknown")
        if distance is None:
            distance_text = "Distance: --"
        else:
            distance_text = f"Distance: {distance:.2f} m"

        if disparity is None:
            disparity_text = "Disparity: --"
        else:
            disparity_text = f"Disparity: {disparity:.1f} px"
        if error is None:
            error_text = "Error: --"
        else:
            error_text = f"Error: {error:+.2f} m"
        if estimated_right is None or estimated_down is None or estimated_forward is None:
            position_text = "Estimated FRD: --"
        else:
            position_text = (
                f"Estimated FRD: {estimated_forward:.2f}, "
                f"{estimated_right:+.2f}, {estimated_down:+.2f} m"
            )
        actual_text = "Actual: --" if actual is None else f"Actual: {actual:.2f} m"

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "stereo"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 12.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 7.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.35
        marker.color.a = 1.0
        marker.text = "\n".join(
            [
                distance_text,
                actual_text,
                error_text,
                position_text,
                disparity_text,
                f"Baseline: {self.node.baseline_m:.2f} m",
                f"Phase: {phase}",
                f"Status: {metrics.get('status', 'unknown')}",
                f"Updated: {time.strftime('%H:%M:%S')}",
            ]
        )
        self.node.marker_pub.publish(marker)

    @staticmethod
    def _marker_point(vector):
        return Point(x=float(vector[0]), y=float(vector[1]), z=float(vector[2]))

    @staticmethod
    def _point_add(a, b):
        return (
            float(a[0]) + float(b[0]),
            float(a[1]) + float(b[1]),
            float(a[2]) + float(b[2]),
        )

    @staticmethod
    def _point_scale(vector, scalar):
        return (float(vector[0]) * scalar, float(vector[1]) * scalar, float(vector[2]) * scalar)

    def _camera_fov_markers(self, stamp):
        """Render camera frustums as translucent faces plus visible corner edges."""

        try:
            width = max(1.0, float(getattr(self.node, "camera_image_width_px", 1280)))
            height = max(1.0, float(getattr(self.node, "camera_image_height_px", 720)))
            horizontal_fov = float(getattr(self.node, "horizontal_fov_rad", 0.154997710698))
        except (TypeError, ValueError):
            return []
        if not math.isfinite(horizontal_fov) or horizontal_fov <= 0.0:
            return []
        depth_m = min(max(1.0, float(getattr(self.node, "camera_clip_far_m", 2500.0))), 2500.0)

        def markers_for_camera(label, marker_id, camera, color):
            geometry = camera_frustum_geometry(
                camera,
                width_px=width,
                height_px=height,
                horizontal_fov_rad=horizontal_fov,
                depth_m=depth_m,
            )

            faces = Marker()
            faces.header.frame_id = self.world_frame
            faces.header.stamp = stamp
            faces.ns = "stereo_camera_fov"
            faces.id = marker_id
            faces.type = Marker.TRIANGLE_LIST
            faces.action = Marker.ADD
            faces.scale.x = faces.scale.y = faces.scale.z = 1.0
            faces.color.r, faces.color.g, faces.color.b = color
            faces.color.a = 0.20
            faces.points = [self.node._marker_point(point) for point in geometry["triangles_world_enu_m"]]
            faces.text = f"{label}_camera_fov_faces"

            edges = Marker()
            edges.header.frame_id = self.world_frame
            edges.header.stamp = stamp
            edges.ns = "stereo_camera_fov"
            edges.id = marker_id + 1
            edges.type = Marker.LINE_LIST
            edges.action = Marker.ADD
            edges.scale.x = 3.0
            edges.color.r, edges.color.g, edges.color.b = color
            edges.color.a = 0.9
            edges.points = [self.node._marker_point(point) for point in geometry["lines_world_enu_m"]]
            edges.text = f"{label}_camera_fov_edges"
            return [faces, edges]

        snapshot = self.node._camera_oracle_snapshot()
        if snapshot is not None:
            left_camera, right_camera = (snapshot[0].pose, snapshot[1].pose)
        else:
            self.node._ensure_camera_frames()
            left_camera, right_camera = self.node.nominal_left_camera, self.node.nominal_right_camera
        return (
            markers_for_camera("left", 50, left_camera, (0.95, 0.95, 0.95))
            + markers_for_camera("right", 60, right_camera, (0.55, 0.72, 0.85))
        )

    def _publish_camera_fov_markers(self):
        """Publish the configured camera frustums without requiring a ray result.

        Contract: marker points are in ``gazebo_world_enu`` metres.  Camera
        poses supply ENU basis vectors; ``right/down/forward`` are used only
        to construct points before they are serialized into that world frame.
        """

        publisher = getattr(self.node, "fov_marker_pub", None)
        if publisher is None:
            return
        stamp = self.node.get_clock().now().to_msg()
        markers = self.node._camera_fov_markers(stamp)
        if markers:
            publisher.publish(MarkerArray(markers=markers))
        camera_publisher = getattr(self.node, "camera_marker_pub", None)
        if camera_publisher is not None:
            camera_publisher.publish(MarkerArray(markers=self.node._camera_position_markers(stamp)))

    def _camera_position_markers(self, stamp):
        """Render the two configured camera origins in ``gazebo_world_enu`` metres."""

        snapshot = self.node._camera_oracle_snapshot()
        if snapshot is not None:
            left_camera, right_camera = (snapshot[0].pose, snapshot[1].pose)
        else:
            self.node._ensure_camera_frames()
            left_camera, right_camera = self.node.nominal_left_camera, self.node.nominal_right_camera
        markers = []
        for marker_id, camera_pose, color in (
            (30, left_camera, (0.95, 0.95, 0.95)),
            (31, right_camera, (0.55, 0.65, 0.70)),
        ):
            camera = Marker()
            camera.header.frame_id = self.world_frame
            camera.header.stamp = stamp
            camera.ns = "stereo_cameras"
            camera.id = marker_id
            camera.type = Marker.CUBE
            camera.action = Marker.ADD
            camera.pose.position = self.node._marker_point(camera_pose.center_world_enu_m)
            camera.pose.orientation.w = 1.0
            # Scale affects only overview visibility; coordinates remain ENU metres.
            camera.scale.x = camera.scale.y = camera.scale.z = 12.0
            camera.color.r, camera.color.g, camera.color.b, camera.color.a = (*color, 1.0)
            markers.append(camera)
        return markers


    def _publish_ray_metrics(
        self,
        *,
        pair_id=None,
        nominal_pose=None,
        optical_pose=None,
        ideal=None,
        gps_baseline=None,
        global_baseline=None,
        attitude=None,
        operational=None,
        attitude_true_north=None,
        gt_source=None,
        gt_fallback_used=None,
    ):
        publisher = getattr(self.node, "ray_metrics_pub", None)
        if publisher is None:
            return
        payload = ray_metrics_document(
            pair_id=pair_id,
            gt_source=gt_source,
            gt_fallback_used=gt_fallback_used,
            nominal_pose=nominal_pose,
            optical_pose=optical_pose,
            ideal=ideal,
            gps_baseline=gps_baseline,
            global_baseline=global_baseline,
            attitude=attitude,
            operational=operational,
            attitude_true_north=attitude_true_north,
        )
        if payload:
            publisher.publish(String(data=json.dumps(payload, sort_keys=True)))


    def _publish_actual_drone_marker(self, position_world_enu_m, orientation_xyzw, stamp_ns):
        """Render the authoritative Gazebo vehicle-reference pose as a drone mesh."""

        publisher = getattr(self.node, "ray_marker_pub", None)
        if publisher is None:
            return
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp.sec = int(stamp_ns) // 1_000_000_000
        marker.header.stamp.nanosec = int(stamp_ns) % 1_000_000_000
        marker.ns = "stereo_actual_drone"
        marker.id = 40
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.position = self.node._marker_point(position_world_enu_m)
        (
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ) = (float(value) for value in orientation_xyzw)
        # Match the model SDF visual scale.  This affects only RViz rendering.
        marker.scale.x = marker.scale.y = marker.scale.z = self.node._mesh_visual_scale
        marker.color.r = marker.color.g = marker.color.b = 0.72
        marker.color.a = 1.0
        marker.mesh_resource = self.node._mesh_resource
        # Keep the mesh readable against RViz's dark background; the source
        # geometry remains the actual Gazebo X500 asset.
        marker.mesh_use_embedded_materials = False
        publisher.publish(MarkerArray(markers=[marker]))

    def _publish_ray_markers(
        self, nominal_pose=None, optical_pose=None, ideal=None,
        gps_baseline=None, global_baseline=None, attitude=None,
        operational=None, attitude_true_north=None, uam_gps_ray=None,
    ):
        """Publish stereo rays in World ENU, including the UAM GPS ray.

        ``gps_baseline`` and ``global_baseline`` retain the World ENU camera origins
        used by their baseline-only triangulations.  They are visualization
        only; public measurement topics remain baseline-center FRD.
        """

        publisher = getattr(self.node, "ray_marker_pub", None)
        if publisher is None:
            return
        stamp = self.node.get_clock().now().to_msg()

        def line(marker_id, namespace, name, color, start, end, width):
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = width
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 1.0)
            marker.points = [self.node._marker_point(start), self.node._marker_point(end)]
            marker.text = name
            return marker

        def marker_group(result, *, label, marker_id, left_color, right_color, gap_color, midpoint_color):
            geometry = ray_line_geometry(result)
            if geometry is None:
                return []
            namespace = f"stereo_rays_{label}"
            display_name = self.ray_display_names[label]
            left_origin = geometry["left_origin_world_enu_m"]
            left_target = geometry["left_target_world_enu_m"]
            right_origin = geometry["right_origin_world_enu_m"]
            right_target = geometry["right_target_world_enu_m"]
            midpoint = geometry["midpoint_world_enu_m"]
            target = Marker()
            target.header.frame_id = self.world_frame
            target.header.stamp = stamp
            target.ns = namespace
            target.id = marker_id + 3
            target.type = Marker.SPHERE
            target.action = Marker.ADD
            target.pose.position = self.node._marker_point(midpoint)
            target.pose.orientation.w = 1.0
            # This is a 2 km overview; sub-metre lines disappear at the
            # default RViz viewport scale.  Geometry remains exact, only the
            # diagnostic rendering is widened.
            target.scale.x = target.scale.y = target.scale.z = 12.0
            target.color.r, target.color.g, target.color.b, target.color.a = (*midpoint_color, 1.0)
            return [
                line(marker_id, namespace, f"left_{display_name}_ray", left_color, left_origin, left_target, 2.0),
                line(marker_id + 1, namespace, f"right_{display_name}_ray", right_color, right_origin, right_target, 2.0),
                line(marker_id + 2, namespace, f"{display_name}_closest_ray_gap", gap_color, left_target, right_target, 1.5),
                target,
            ]

        markers = marker_group(
            nominal_pose,
            label="nominal_pose",
            marker_id=0,
            # Match rqt: red means YOLO pixels + nominal optical poses.
            left_color=(220.0 / 255.0, 32.0 / 255.0, 32.0 / 255.0),
            right_color=(220.0 / 255.0, 32.0 / 255.0, 32.0 / 255.0),
            gap_color=(1.0, 0.45, 0.45),
            midpoint_color=(220.0 / 255.0, 32.0 / 255.0, 32.0 / 255.0),
        )
        markers.extend(
            marker_group(
                optical_pose,
                label="optical_pose",
                marker_id=10,
                # Match rqt: teal means YOLO pixels + Gazebo optical poses.
                left_color=(0.0, 170.0 / 255.0, 170.0 / 255.0),
                right_color=(0.0, 170.0 / 255.0, 170.0 / 255.0),
                gap_color=(0.3, 0.9, 0.9),
                midpoint_color=(0.0, 170.0 / 255.0, 170.0 / 255.0),
            )
        )
        markers.extend(
            marker_group(
                attitude,
                label="attitude",
                marker_id=50,
                # Hot pink denotes PX4 attitude with Gazebo optical centers.
                left_color=(1.0, 0.1, 0.75),
                right_color=(1.0, 0.1, 0.75),
                gap_color=(1.0, 0.55, 0.9),
                midpoint_color=(1.0, 0.1, 0.75),
            )
        )
        markers.extend(
            marker_group(
                operational,
                label="operational",
                marker_id=70,
                # White denotes the combined PX4 attitude/raw-GPS geometry.
                left_color=(0.95, 0.95, 0.95),
                right_color=(0.95, 0.95, 0.95),
                gap_color=(0.7, 0.7, 0.7),
                midpoint_color=(0.95, 0.95, 0.95),
            )
        )
        markers.extend(
            marker_group(
                attitude_true_north,
                label="attitude_true_north",
                marker_id=90,
                # Green-cyan denotes PX4 attitude with a live GPS-baseline
                # declination correction applied (see gps_baseline.py
                # declination_from_gps_baseline_rad).
                left_color=(0.0, 0.85, 0.4),
                right_color=(0.0, 0.85, 0.4),
                gap_color=(0.45, 0.95, 0.7),
                midpoint_color=(0.0, 0.85, 0.4),
            )
        )
        markers.extend(
            marker_group(
                ideal,
                label="ideal",
                marker_id=20,
                # Match rqt: blue means GT-projected, quantized nominal pixels.
                left_color=(32.0 / 255.0, 86.0 / 255.0, 1.0),
                right_color=(32.0 / 255.0, 86.0 / 255.0, 1.0),
                gap_color=(0.45, 0.65, 1.0),
                midpoint_color=(32.0 / 255.0, 86.0 / 255.0, 1.0),
            )
        )
        markers.extend(
            marker_group(
                gps_baseline,
                label="gps_baseline",
                marker_id=30,
                # Purple distinguishes raw PX4 GPS baseline from optical rays.
                left_color=(0.65, 0.25, 0.95),
                right_color=(0.65, 0.25, 0.95),
                gap_color=(0.82, 0.62, 1.0),
                midpoint_color=(0.65, 0.25, 0.95),
            )
        )
        markers.extend(
            marker_group(
                global_baseline,
                label="global_baseline",
                marker_id=40,
                # Amber distinguishes global-position baseline from raw GPS.
                left_color=(1.0, 0.62, 0.0),
                right_color=(1.0, 0.62, 0.0),
                gap_color=(1.0, 0.82, 0.35),
                midpoint_color=(1.0, 0.62, 0.0),
            )
        )
        if uam_gps_ray is not None and uam_gps_ray.get("computation_status") == "computed":
            geometry = uam_gps_ray.get("ray_visualization_world_enu")
            if geometry is None:
                geometry = {
                    "origin_world_enu_m": uam_gps_ray["origin_world_enu_m"],
                    "target_world_enu_m": uam_gps_ray["target_world_enu_m"],
                }
            namespace = "stereo_rays_uam_gps"
            origin = geometry["origin_world_enu_m"]
            target = geometry["target_world_enu_m"]
            gps_color = (134.0 / 255.0, 89.0 / 255.0, 45.0 / 255.0)
            markers.append(
                line(80, namespace, "UAM GPS Ray", gps_color, origin, target, 2.5)
            )
            target_marker = Marker()
            target_marker.header.frame_id = self.world_frame
            target_marker.header.stamp = stamp
            target_marker.ns = namespace
            target_marker.id = 81
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position = self.node._marker_point(target)
            target_marker.pose.orientation.w = 1.0
            target_marker.scale.x = target_marker.scale.y = target_marker.scale.z = 12.0
            target_marker.color.r, target_marker.color.g, target_marker.color.b, target_marker.color.a = (*gps_color, 1.0)
            markers.append(target_marker)
        if markers:
            publisher.publish(MarkerArray(markers=markers))
