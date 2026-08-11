"""Stereo triangulation and diagnostic-ray facade.

Pixel inputs use OpenCV optical (u right, v down) pixels. Camera poses are
World ENU with optical RDF bases. Pipeline outputs use baseline-center FRD
[forward, right, down] metres; no implicit axis aliasing is performed here.
"""

from stereo_ground_sim.diagnostic_ray_pipeline import (
    DiagnosticRayOrchestrator,
    diagnostic_analysis,
    diagnostic_delta,
    finalize_diagnostic_ray,
)
from stereo_ground_sim.research_record_fields import (
    calibration_research_record_fields,
    diagnostic_ray_record_fields,
    diagnostic_research_record_fields,
)
from stereo_ground_sim.stereo_measurement_pipeline import (
    triangulate_pixel_centers as triangulate_pixel_centers_pipeline,
)

_CAMERA_ORACLE_DEFAULT = object()

TRIANGULATION_FACADE_METHODS = [
    "_triangulate_pixel_centers_with_cameras",
    "_triangulate_pixel_centers",
    "_triangulate_dkl_ideal_from_reference_world",
    "_triangulate_pixel_centers_oracle",
    "_triangulate_pixel_centers_with_gps_baseline",
    "_gps_baseline_sources",
    "_triangulate_gps_baselines",
    "_finalize_diagnostic_ray",
    "_diagnostic_delta",
    "_diagnostic_analysis",
    "_diagnostic_ray_record_fields",
    "_diagnostic_research_record_fields",
    "_calibration_research_record_fields",
    "_diagnostic_rays"
]


class TriangulationFacade:
    def __init__(self, node, *, dkl_projection, visualization_builder, gps_baseline_anchor_policy, camera_oracle_unset):
        self.node = node
        self.dkl_projection = dkl_projection
        self.visualization_builder = visualization_builder
        self.gps_baseline_anchor_policy = gps_baseline_anchor_policy
        self.camera_oracle_unset = camera_oracle_unset

    def _triangulate_pixel_centers_with_cameras(
        self,
        left_u_px,
        left_v_px,
        right_u_px,
        right_v_px,
        width,
        height,
        *,
        left_camera_pose,
        right_camera_pose,
    ):
        """Triangulate pixels using explicitly supplied optical camera poses."""

        self.node._ensure_camera_frames()
        return triangulate_pixel_centers_pipeline(
            (left_u_px, left_v_px, right_u_px, right_v_px),
            intrinsics=self.node._camera_intrinsics(width, height),
            left_camera_pose=left_camera_pose,
            right_camera_pose=right_camera_pose,
            baseline_center_frd=self.node.baseline_center_frd,
            visualization_builder=self.visualization_builder,
        )

    def _triangulate_pixel_centers(self, left_u_px, left_v_px, right_u_px, right_v_px, width, height):
        """Canonical Nominal/Raw: use the fixed configured PX4 bias only.

        The configured ``CAM_*_PX4_ATTITUDE_*`` values are immutable for a
        simulation run. Runtime PX4 noise is intentionally excluded here;
        the Attitude diagnostic uses the latest ``VehicleAttitude`` samples
        for that time-varying path.
        """

        self.node._ensure_camera_frames()
        return self.node._triangulate_pixel_centers_with_cameras(
            left_u_px,
            left_v_px,
            right_u_px,
            right_v_px,
            width,
            height,
            left_camera_pose=self.node.nominal_px4_left_camera,
            right_camera_pose=self.node.nominal_px4_right_camera,
        )

    def _triangulate_dkl_ideal_from_reference_world(self, point_world_enu_m, width, height):
        """Triangulate DKL-style ideal pixels in the public baseline FRD frame.

        This is deliberately a parallel simulation reference, not an input to
        YOLO, Optical, or PX4 baseline processing.  Its target and camera poses
        are World ENU; the returned triangulation follows the normal public
        ``baseline_center_frd`` F/R/D(/Range) contract.
        """

        self.node._ensure_camera_frames()
        try:
            intrinsics = self.node._camera_intrinsics(width, height)
            pixels = self.dkl_projection(
                point_world_enu_m,
                self.node.nominal_left_camera,
                self.node.nominal_right_camera,
                intrinsics,
                width,
                height,
            )
        except (TypeError, ValueError, ArithmeticError):
            return None
        if pixels is None:
            return None
        # Ideal pixels are generated and reconstructed with the physical
        # zero-error optical poses.  Do not route this reference through the
        # PX4-biased Nominal/Raw ray.
        result = self.node._triangulate_pixel_centers_with_cameras(
            *pixels,
            width,
            height,
            left_camera_pose=self.node.nominal_left_camera,
            right_camera_pose=self.node.nominal_right_camera,
        )
        result.update(
            {
                "left_u_px": float(pixels[0]),
                "left_v_px": float(pixels[1]),
                "right_u_px": float(pixels[2]),
                "right_v_px": float(pixels[3]),
            }
        )
        return result

    def _triangulate_pixel_centers_oracle(
        self, left_u_px, left_v_px, right_u_px, right_v_px, width, height, *, camera_oracle_snapshot=_CAMERA_ORACLE_DEFAULT
    ):
        """Triangulate only with the validated Gazebo-observed optical poses."""

        self.node._ensure_camera_frames()
        snapshot = camera_oracle_snapshot
        if snapshot is _CAMERA_ORACLE_DEFAULT or snapshot is self.camera_oracle_unset:
            snapshot = self.node._camera_oracle_snapshot()
        if snapshot is None:
            return None
        left_camera, right_camera = snapshot
        return self.node._triangulate_pixel_centers_with_cameras(
            left_u_px,
            left_v_px,
            right_u_px,
            right_v_px,
            width,
            height,
            left_camera_pose=left_camera.pose,
            right_camera_pose=right_camera.pose,
        )

    def _triangulate_pixel_centers_with_gps_baseline(
        self, left_u_px, left_v_px, right_u_px, right_v_px, width, height, source
    ):
        """Parallel triangulation with one PX4-derived camera separation.

        The left nominal center is retained as the anchor.  This intentionally
        isolates the *baseline* error from any unknown global GPS translation.
        Optical attitudes and image measurements are unchanged.
        """

        poses, status, provenance = self.node._gps_baseline_camera_poses(source)
        if poses is None:
            return {"computation_status": status, **provenance}
        result = self.node._triangulate_pixel_centers_with_cameras(
            left_u_px,
            left_v_px,
            right_u_px,
            right_v_px,
            width,
            height,
            left_camera_pose=poses[0],
            right_camera_pose=poses[1],
        )
        result.update(provenance)
        result["baseline_source"] = (
            "px4_raw_vehicle_gps_position_baseline_only"
            if source == "gps_baseline"
            else "px4_vehicle_global_position_baseline_only"
        )
        return result

    def _gps_baseline_sources(self):
        source = self.node.gps_baseline_source
        return ("gps_baseline", "global_baseline") if source == "both" else (source,)

    def _triangulate_gps_baselines(
        self, left_u_px, left_v_px, right_u_px, right_v_px, width, height
    ):
        return {
            source: self.node._triangulate_pixel_centers_with_gps_baseline(
                left_u_px, left_v_px, right_u_px, right_v_px, width, height, source
            )
            for source in self.node._gps_baseline_sources()
        }

    @classmethod
    def _finalize_diagnostic_ray(cls, result, *, provenance=None, failure_reason=""):
        return finalize_diagnostic_ray(
            result, provenance=provenance, failure_reason=failure_reason
        )

    @classmethod
    def _diagnostic_delta(cls, test, truth):
        return diagnostic_delta(test, truth)

    @classmethod
    def _diagnostic_analysis(cls, truth, diagnostic_rays):
        return diagnostic_analysis(truth, diagnostic_rays)

    @staticmethod
    def _diagnostic_ray_record_fields(name, result):
        return diagnostic_ray_record_fields(name, result)

    @staticmethod
    def _diagnostic_research_record_fields(metrics):
        return diagnostic_research_record_fields(metrics)

    @staticmethod
    def _calibration_research_record_fields(pair, alignment, metrics):
        return calibration_research_record_fields(pair, alignment, metrics)

    def _diagnostic_rays(
        self,
        left_u,
        left_v,
        right_u,
        right_v,
        width,
        height,
        snapshot,
        *,
        operational_snapshot=None,
        observation_timestamp_ns=None,
    ):
        """Re-triangulate observed pixels with independently selected inputs."""
        orchestrator = DiagnosticRayOrchestrator(
            {
                "attitude_poses": self.node._px4_attitude_camera_poses,
                "true_north_attitude_poses": self.node._px4_attitude_true_north_camera_poses,
                "declination_attitude_poses": self.node._px4_attitude_declination_camera_poses,
                "gps_poses": self.node._gps_baseline_camera_poses,
                "triangulate": self.node._triangulate_pixel_centers_with_cameras,
            },
            gps_baseline_anchor_policy=self.gps_baseline_anchor_policy,
        )
        return orchestrator.compute(
            left_u,
            left_v,
            right_u,
            right_v,
            width,
            height,
            snapshot,
            operational_snapshot=operational_snapshot,
            observation_timestamp_ns=observation_timestamp_ns,
        )
