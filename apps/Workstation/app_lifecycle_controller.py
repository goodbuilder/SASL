"""Workstation startup and shutdown resource lifecycle controller."""

from __future__ import annotations

import time
from collections.abc import Mapping

from PyQt5 import QtCore, QtWidgets

from Workstation.run_evidence_controller import persistence_failure_error
from Workstation.workstation_core.camera_pane import (
    CAMERA_STOP_KILL_AFTER_S,
    CAMERA_STOP_TERMINATE_AFTER_S,
)


# Camera shutdown owns a 9-second MP4 EOS window inside its 12-second
# cooperative process-stop window. A shorter application deadline turns a
# valid slow mp4mux finalization into a forced kill.
SHUTDOWN_DEADLINE_S = CAMERA_STOP_TERMINATE_AFTER_S
SHUTDOWN_FORCE_WAIT_S = 1.0
SHUTDOWN_RESOURCE_REPORT_SCHEMA = "workstation-shutdown-resources-v1"

APP_LIFECYCLE_METHODS = [
    "_shutdown_save_settings",
    "_shutdown_stop_thread",
    "_shutdown_stop_auxiliary_workers",
    "_shutdown_stop_gnss_base_serial",
    "_shutdown_stop_target_drone",
    "_shutdown_stop_drone_telemetry",
    "_shutdown_verify_resources",
    "shutdown_for_close",
    "closeEvent",
    "_stop_calibration_preview_worker"
]


class AppLifecycleController:
    def __init__(self, panel):
        self.panel = panel
        self._linked_target_artifact_ids: set[str] = set()

    def _shutdown_save_settings(self) -> bool:
        flush_ok = True
        data_flusher = getattr(self.panel, "flush_workstation_data", None)
        if callable(data_flusher):
            try:
                if data_flusher() is False:
                    flush_ok = False
                    self.panel._log("[종료] measurement data flush 요청 실패")
            except Exception as exc:
                flush_ok = False
                self.panel._log(f"[종료] measurement data flush 요청 실패: {exc}")
        flusher = getattr(self.panel, "flush_workstation_run_evidence", None)
        if callable(flusher):
            try:
                if flusher() is False:
                    flush_ok = False
                    self.panel._log("[종료] run evidence flush 실패")
            except Exception as exc:
                flush_ok = False
                self.panel._log(f"[종료] run evidence flush 실패: {exc}")
        try:
            self.panel._save_current_settings()
        except Exception as exc:
            self.panel._log(f"[종료] 설정 저장 실패: {exc}")
            return False
        return flush_ok

    def _shutdown_stop_thread(self, attr: str) -> bool:
        thread = getattr(self.panel, attr, None)
        if thread is None:
            return True
        try:
            thread.quit()
            if thread.isRunning():
                return False
        except RuntimeError:
            pass
        if getattr(self.panel, attr, None) is thread:
            setattr(self.panel, attr, None)
        return True

    def _shutdown_stop_auxiliary_workers(self) -> bool:
        auto_capture_stopped = self.panel.stop_auto_calibration_capture()
        preview_stopped = self.panel._stop_calibration_preview_worker(wait=False)
        lrf_stopped = self.panel.stop_wiris_lrf_polling(wait=False)
        measurement_owner = getattr(
            self.panel, "_measurement_view_controller_owner", None
        )
        measurement_workers_stopped = (
            True
            if measurement_owner is None
            else bool(measurement_owner.close_method_evaluation_workers())
        )
        native_store_closer = getattr(
            self.panel,
            "_shutdown_native_frame_stores",
            None,
        )
        native_stores_stopped = (
            True
            if not callable(native_store_closer)
            else bool(native_store_closer())
        )
        data_owner = getattr(self.panel, "_data_storage_controller_owner", None)
        latest_xlsx_stopped = (
            True
            if data_owner is None
            else bool(
                data_owner.cancel_latest_measurement_xlsx_export(timeout_s=0.0)
            )
        )
        native_rings_stopped = True
        for side in ("left", "right"):
            pane = getattr(self.panel, f"{side}_pane", None)
            sweep = getattr(pane, "sweep_retired_native_frame_rings", None)
            if not callable(sweep):
                continue
            try:
                if sweep() is False:
                    native_rings_stopped = False
            except (RuntimeError, OSError) as exc:
                native_rings_stopped = False
                self.panel._log(
                    f"[종료] {side} retired native ring 정리 실패: {exc}"
                )
        attrs = (
            "camera_auto_detect_thread",
            "wiris_control_check_thread",
            "wiris_command_thread",
            "pnp_photo_capture_thread",
            "oz_photo_capture_thread",
            "wiris_zoom_thread",
            "wiris_time_sync_thread",
            "wiris_stip_thread",
            "camera_network_discovery_thread",
            "camera_network_apply_thread",
            "interface_status_thread",
            "pixhawk_usb_discovery_thread",
            "pixhawk_host_ip_discovery_thread",
            "pixhawk_network_discovery_thread",
            "drone_telemetry_usb_discovery_thread",
            "rtsp_check_thread",
            "rtsp_enable_thread",
            "gnss_base_detect_thread",
            "checkerboard_calibration_thread",
            "calibration_wiris_ftp_thread",
            "calibration_wiris_ftp_oz_thread",
            "calibration_wiris_oz_capture_thread",
            "calibration_wiris_oz_sync_thread",
            "calibration_wiris_oz_delete_thread",
            "calibration_wiris_oz_format_thread",
            "calibration_wiris_oz_gilc_thread",
            "wiris_image_format_thread",
            "optical_pose_wiris_ftp_thread",
            "optical_pose_wiris_ftp_oz_thread",
            "_measurement_media_verification_thread",
        )
        results = [self.panel._shutdown_stop_thread(attr) for attr in attrs]
        return bool(
            auto_capture_stopped
            and preview_stopped
            and lrf_stopped
            and measurement_workers_stopped
            and native_stores_stopped
            and latest_xlsx_stopped
            and native_rings_stopped
            and all(results)
        )

    def _shutdown_stop_gnss_base_serial(self) -> bool:
        self.panel.disconnect_gnss_base_serial()
        thread = getattr(self.panel, "gnss_base_serial_thread", None)
        return thread is None or not thread.isRunning()

    def _shutdown_stop_target_drone(self) -> bool:
        owner = getattr(self.panel, "_target_drone_controller_owner", None)
        if owner is None:
            return True
        snapshot = owner.target_drone_stream_snapshot()
        result = snapshot
        if snapshot.get("worker_alive") or snapshot.get("phase") in {
            "RUNNING",
            "STARTING",
        }:
            try:
                result = owner.stop_target_drone_stream(timeout_s=1.0)
            except Exception as exc:
                self.panel._log(f"[종료] Target Drone drain/stop 실패: {exc}")
                return False
        if not isinstance(result, dict):
            self.panel._log("[종료] Target Drone stop snapshot이 잘못된 형식입니다.")
            return False
        artifact = result.get("artifact") if isinstance(result, dict) else None
        if isinstance(artifact, dict):
            artifact_id = str(
                artifact.get("artifact_id")
                or artifact.get("sha256")
                or artifact.get("absolute_path")
                or ""
            )
            if artifact_id not in self._linked_target_artifact_ids:
                registrar = getattr(
                    self.panel,
                    "register_workstation_session_artifact",
                    None,
                )
                importer = getattr(
                    self.panel,
                    "import_workstation_session_artifact",
                    None,
                )
                try:
                    metadata = {
                        key: value
                        for key, value in artifact.items()
                        if key != "absolute_path"
                    }
                    if callable(registrar):
                        linkage = registrar(
                            artifact["absolute_path"],
                            role="target_drone_raw_mavlink_jsonl",
                            media_type="application/x-ndjson",
                            metadata=metadata,
                            expected_size_bytes=artifact.get("size_bytes"),
                            expected_sha256=artifact.get("sha256"),
                        )
                    elif callable(importer):
                        linkage = importer(
                            artifact["absolute_path"],
                            role="target_drone_raw_mavlink_jsonl",
                            media_type="application/x-ndjson",
                            metadata=metadata,
                        )
                    else:
                        self.panel._log(
                            "[종료] Target Drone artifact linkage owner가 없습니다."
                        )
                        return False
                except Exception as exc:
                    self.panel._log(f"[종료] Target Drone artifact linkage 실패: {exc}")
                    return False
                linkage_error = persistence_failure_error(linkage)
                if linkage_error is not None:
                    self.panel._log(
                        "[종료] Target Drone artifact linkage 실패: "
                        f"{linkage_error}"
                    )
                    return False
                if artifact_id:
                    self._linked_target_artifact_ids.add(artifact_id)
        return not bool(result.get("worker_alive"))

    def _shutdown_stop_drone_telemetry(self) -> bool:
        owner = getattr(self.panel, "_drone_telemetry_controller_owner", None)
        if owner is not None:
            try:
                owner.disconnect_drone_telemetry()
            except Exception as exc:
                self.panel._log(f"[종료] Drone Telemetry 연결 해제 실패: {exc}")
                return False
        thread = getattr(self.panel, "drone_telemetry_connection_thread", None)
        return thread is None or not thread.isRunning()

    def shutdown_for_close(self) -> bool:
        """Run the one close sequence shared by every top-level window."""

        coordinator = getattr(self.panel, "_shutdown_coordinator", None)
        if coordinator is None:
            return False
        now_s = time.monotonic()
        started_s = getattr(self.panel, "_shutdown_started_monotonic_s", None)
        if started_s is None:
            started_s = now_s
            self.panel._shutdown_started_monotonic_s = started_s
            self.panel._shutdown_in_progress = True
            self.panel._pending_measurement_run_start = False
            cancel_measurement = getattr(self.panel, "cancel_measurement_start", None)
            if callable(cancel_measurement):
                cancel_measurement("Application shutdown cancelled queued Measurement start")
        try:
            complete = bool(coordinator.run())
        except Exception as exc:  # noqa: BLE001 - shutdown must still reach its deadline.
            self.panel._log(f"[종료] lifecycle 단계 예외: {exc}")
            complete = False
        if complete:
            self.panel._shutdown_started_monotonic_s = None
            return True
        if now_s < self._shutdown_force_not_before_s(float(started_s)):
            return False
        self._force_shutdown_after_deadline()
        # Escalation is a request, not proof. Resume the ordered lifecycle so
        # camera EOS/failure evidence is appended before settings flush and
        # the final DataStorage close. A survivor keeps closeEvent retrying.
        try:
            complete = bool(coordinator.run())
        except Exception as exc:  # noqa: BLE001 - preserve truthful pending state.
            self.panel._log(f"[종료] 강제 정리 후 lifecycle 재시도 예외: {exc}")
            complete = False
        if complete:
            self.panel._shutdown_started_monotonic_s = None
            return True
        return False

    def _shutdown_force_not_before_s(self, started_s: float) -> float:
        """Protect the full camera cooperative window even after earlier steps."""

        not_before_s = float(started_s) + SHUTDOWN_DEADLINE_S
        for side in ("left", "right"):
            pane = getattr(self.panel, f"{side}_pane", None)
            worker = getattr(pane, "worker", None)
            requested_s = getattr(pane, "_stop_requested_monotonic_s", None)
            if requested_s is None:
                requested_s = getattr(worker, "_stop_requested_monotonic_s", None)
            try:
                requested_value = float(requested_s)
            except (TypeError, ValueError):
                continue
            not_before_s = max(
                not_before_s,
                requested_value + CAMERA_STOP_TERMINATE_AFTER_S,
            )
        return not_before_s

    @staticmethod
    def _thread_running(thread: object) -> bool:
        try:
            is_running = getattr(thread, "isRunning", None)
            if callable(is_running):
                return bool(is_running())
            is_alive = getattr(thread, "is_alive", None)
            return bool(is_alive()) if callable(is_alive) else False
        except (AttributeError, RuntimeError, TypeError):
            return False

    @staticmethod
    def _process_alive(process: object) -> bool:
        try:
            return bool(process.is_alive())
        except (AttributeError, AssertionError, OSError, ValueError):
            return False

    @staticmethod
    def _socket_open(socket_object: object) -> bool:
        try:
            return int(socket_object.fileno()) >= 0
        except (AttributeError, OSError, TypeError, ValueError):
            return getattr(socket_object, "closed", None) is False

    def _panel_attributes(self) -> dict[str, object]:
        try:
            return dict(vars(self.panel))
        except TypeError:
            return {}

    def _shutdown_resource_snapshot(self) -> dict[str, object]:
        """Report cleanup postconditions from live owners, not stop requests."""

        attributes = self._panel_attributes()
        running_threads: list[str] = []
        seen_threads: set[int] = set()

        def inspect_thread(label: str, candidate: object | None) -> None:
            if candidate is None or id(candidate) in seen_threads:
                return
            seen_threads.add(id(candidate))
            if self._thread_running(candidate):
                running_threads.append(str(label))

        for name, value in attributes.items():
            if name.endswith("_thread"):
                inspect_thread(name, value)
            elif name.endswith("_threads") and isinstance(value, Mapping):
                for key, thread in value.items():
                    inspect_thread(f"{name}[{key}]", thread)
        finder = getattr(self.panel, "findChildren", None)
        if callable(finder):
            try:
                for index, thread in enumerate(finder(QtCore.QThread)):
                    inspect_thread(f"qt-child[{index}]", thread)
            except (RuntimeError, TypeError):
                pass

        live_processes: list[str] = []
        seen_processes: set[int] = set()

        def inspect_process(label: str, candidate: object | None) -> None:
            if candidate is None or id(candidate) in seen_processes:
                return
            seen_processes.add(id(candidate))
            if self._process_alive(candidate):
                live_processes.append(str(label))

        for side in ("left", "right"):
            pane = getattr(self.panel, f"{side}_pane", None)
            worker = getattr(pane, "worker", None)
            inspect_process(f"camera-child:{side}", getattr(worker, "_process", None))
        for name, value in attributes.items():
            if name.endswith("_process"):
                inspect_process(name, value)

        open_sockets: list[str] = []
        seen_sockets: set[int] = set()

        def inspect_socket(label: str, candidate: object | None) -> None:
            if candidate is None or id(candidate) in seen_sockets:
                return
            seen_sockets.add(id(candidate))
            if self._socket_open(candidate):
                open_sockets.append(str(label))

        for name, value in attributes.items():
            if name.endswith("_socket"):
                inspect_socket(name, value)
            if name.endswith("_worker") and value is not None:
                inspect_socket(f"{name}._socket", getattr(value, "_socket", None))
                client = getattr(value, "_client", None)
                inspect_socket(
                    f"{name}._client._socket",
                    getattr(client, "_socket", None),
                )
        mavlink = getattr(self.panel, "mavlink_pixhawk", None)
        connections = getattr(mavlink, "connections", None)
        if isinstance(connections, Mapping):
            for side, connection in connections.items():
                label = f"mavlink:{side}"
                if label not in open_sockets:
                    open_sockets.append(label)
                inspect_socket(f"{label}.port", getattr(connection, "port", None))

        shared_memory: list[dict[str, object]] = []
        seen_rings: set[int] = set()

        def inspect_ring(label: str, ring: object | None) -> None:
            if ring is None or id(ring) in seen_rings:
                return
            seen_rings.add(id(ring))
            if bool(getattr(ring, "_unlinked", False)):
                return
            try:
                metrics = ring.metrics()
            except (AttributeError, RuntimeError, OSError, TypeError, ValueError):
                metrics = {}
            metrics = dict(metrics) if isinstance(metrics, Mapping) else {}
            shared_memory.append(
                {
                    "owner": str(label),
                    "name": str(getattr(ring, "shm_name", "") or ""),
                    "active_leases": max(0, int(metrics.get("active_leases", 0) or 0)),
                    "closed": bool(getattr(ring, "_closed", False)),
                    "unlinked": bool(getattr(ring, "_unlinked", False)),
                }
            )

        for side in ("left", "right"):
            pane = getattr(self.panel, f"{side}_pane", None)
            inspect_ring(f"{side}.current", getattr(pane, "native_frame_ring", None))
            for index, ring in enumerate(
                tuple(getattr(pane, "_retired_native_frame_rings", ()) or ())
            ):
                inspect_ring(f"{side}.retired[{index}]", ring)

        coordinator = getattr(self.panel, "_shutdown_coordinator", None)
        index = int(getattr(coordinator, "index", 0) or 0)
        steps = tuple(getattr(coordinator, "steps", ()) or ())
        current_step = (
            str(getattr(steps[index], "name", ""))
            if 0 <= index < len(steps)
            else ""
        )
        complete = not (
            running_threads or live_processes or open_sockets or shared_memory
        )
        return {
            "schema": SHUTDOWN_RESOURCE_REPORT_SCHEMA,
            "complete": complete,
            "coordinator_step": current_step,
            "running_threads": sorted(running_threads),
            "live_processes": sorted(live_processes),
            "open_sockets": sorted(open_sockets),
            "shared_memory": sorted(
                shared_memory,
                key=lambda row: str(row.get("owner") or ""),
            ),
        }

    def _publish_shutdown_resource_report(
        self,
        report: Mapping[str, object],
        *,
        phase: str,
    ) -> bool:
        payload = {**dict(report), "phase": str(phase)}
        self.panel._shutdown_resource_report = payload
        fingerprint = repr(payload)
        if fingerprint == getattr(
            self.panel,
            "_shutdown_resource_report_fingerprint",
            None,
        ):
            return True
        logger = getattr(self.panel, "_log", None)
        if callable(logger):
            if bool(payload.get("complete", False)):
                logger(f"[종료] 자원 정리 검증 완료 ({phase})")
            else:
                logger(
                    "[종료] 자원 정리 미완료 "
                    f"({phase}, step={payload.get('coordinator_step') or '-'}): "
                    f"threads={payload.get('running_threads')}, "
                    f"processes={payload.get('live_processes')}, "
                    f"sockets={payload.get('open_sockets')}, "
                    f"shared_memory={payload.get('shared_memory')}"
                )
        append = getattr(self.panel, "append_workstation_data", None)
        evidence_accepted = True
        if callable(append):
            try:
                evidence_accepted = append(
                    "events",
                    {
                        "event_type": "shutdown_resource_verification",
                        **payload,
                    },
                    priority=True,
                ) is not False
            except (OSError, RuntimeError, TypeError, ValueError) as exc:
                evidence_accepted = False
                if callable(logger):
                    logger(f"[종료] 자원 정리 evidence 기록 실패: {exc}")
        if evidence_accepted:
            self.panel._shutdown_resource_report_fingerprint = fingerprint
        elif callable(logger):
            logger("[종료] 자원 정리 evidence queue 거부; DataStorage close를 보류합니다.")
        return evidence_accepted

    def _shutdown_verify_resources(self) -> bool:
        for side in ("left", "right"):
            pane = getattr(self.panel, f"{side}_pane", None)
            sweep = getattr(pane, "sweep_retired_native_frame_rings", None)
            if callable(sweep):
                try:
                    sweep()
                except (RuntimeError, OSError):
                    pass
        report = self._shutdown_resource_snapshot()
        evidence_accepted = self._publish_shutdown_resource_report(
            report,
            phase="cooperative",
        )
        return bool(report["complete"] and evidence_accepted)

    def _force_shutdown_after_deadline(self) -> bool:
        """Escalate, then prove whether every known runtime resource stopped."""

        logger = getattr(self.panel, "_log", None)
        if callable(logger) and not bool(
            getattr(self.panel, "_shutdown_force_deadline_logged", False)
        ):
            logger(
                f"[종료] {SHUTDOWN_DEADLINE_S:.1f}초 deadline 초과; "
                "남은 worker를 강제 종료합니다."
            )
            self.panel._shutdown_force_deadline_logged = True

        camera_force_requested = getattr(
            self.panel,
            "_shutdown_force_camera_requested_s",
            None,
        )
        if not isinstance(camera_force_requested, dict):
            camera_force_requested = {}
            self.panel._shutdown_force_camera_requested_s = camera_force_requested
        camera_force_now_s = time.monotonic()
        for side in ("left", "right"):
            pane = getattr(self.panel, f"{side}_pane", None)
            worker = getattr(pane, "worker", None)
            if worker is None:
                continue
            request = getattr(worker, "request_stop", None)
            if callable(request):
                try:
                    request()
                except Exception:
                    pass
            requested_s = getattr(pane, "_stop_requested_monotonic_s", None)
            if requested_s is None:
                requested_s = getattr(worker, "_stop_requested_monotonic_s", None)
            if requested_s is None:
                requested_s = camera_force_requested.setdefault(
                    side,
                    camera_force_now_s,
                )
            try:
                elapsed_s = max(0.0, camera_force_now_s - float(requested_s))
            except (TypeError, ValueError):
                elapsed_s = 0.0
            method_name = (
                "kill_child_process"
                if elapsed_s >= CAMERA_STOP_KILL_AFTER_S
                else "terminate_child_process"
                if elapsed_s >= CAMERA_STOP_TERMINATE_AFTER_S
                else ""
            )
            method = getattr(worker, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass

        threads: list[object] = []
        seen: set[int] = set()
        protected_camera_threads: set[int] = set()

        for side in ("left", "right"):
            pane = getattr(self.panel, f"{side}_pane", None)
            worker = getattr(pane, "worker", None)
            process = getattr(worker, "_process", None)
            thread = getattr(pane, "thread", None)
            if process is None or thread is None:
                continue
            requested_s = getattr(pane, "_stop_requested_monotonic_s", None)
            if requested_s is None:
                requested_s = getattr(worker, "_stop_requested_monotonic_s", None)
            if requested_s is None:
                requested_s = camera_force_requested.get(side, camera_force_now_s)
            try:
                elapsed_s = max(0.0, camera_force_now_s - float(requested_s))
            except (TypeError, ValueError):
                elapsed_s = 0.0
            # Do not terminate the QThread while it can still drain the
            # child's terminal/finalization IPC. Once child death is proven,
            # allow one kill-join interval before treating the Qt loop itself
            # as the surviving resource.
            if self._process_alive(process) or elapsed_s < (
                CAMERA_STOP_KILL_AFTER_S + SHUTDOWN_FORCE_WAIT_S
            ):
                protected_camera_threads.add(id(thread))

        def add_thread(candidate: object | None) -> None:
            if candidate is None or id(candidate) in seen:
                return
            seen.add(id(candidate))
            threads.append(candidate)

        for name, value in self._panel_attributes().items():
            if name.endswith("_thread"):
                add_thread(value)
            elif name.endswith("_threads") and isinstance(value, Mapping):
                for thread in value.values():
                    add_thread(thread)
        finder = getattr(self.panel, "findChildren", None)
        if callable(finder):
            try:
                for thread in finder(QtCore.QThread):
                    add_thread(thread)
            except (RuntimeError, TypeError):
                pass

        running: list[object] = []
        for thread in threads:
            try:
                if not thread.isRunning():
                    continue
                interrupter = getattr(thread, "requestInterruption", None)
                if callable(interrupter):
                    interrupter()
                thread.quit()
                if id(thread) in protected_camera_threads:
                    running.append(thread)
                    continue
                terminator = getattr(thread, "terminate", None)
                if callable(terminator):
                    terminator()
                running.append(thread)
            except (AttributeError, RuntimeError, TypeError):
                continue

        force_deadline_s = time.monotonic() + SHUTDOWN_FORCE_WAIT_S
        for thread in running:
            remaining_ms = max(0, int((force_deadline_s - time.monotonic()) * 1000.0))
            if remaining_ms <= 0:
                break
            waiter = getattr(thread, "wait", None)
            if callable(waiter):
                try:
                    waiter(min(100, remaining_ms))
                except (AttributeError, RuntimeError, TypeError):
                    pass

        for side in ("left", "right"):
            pane = getattr(self.panel, f"{side}_pane", None)
            sweep = getattr(pane, "sweep_retired_native_frame_rings", None)
            if callable(sweep):
                try:
                    sweep()
                except (RuntimeError, OSError):
                    pass

        report = self._shutdown_resource_snapshot()
        evidence_accepted = self._publish_shutdown_resource_report(
            report,
            phase="forced",
        )
        # DataStorage remains open. These are requests made only after the
        # forced/incomplete report has been offered to persistence; the
        # coordinator owns the final close barrier.
        for name in ("flush_workstation_run_evidence", "flush_workstation_data"):
            flusher = getattr(self.panel, name, None)
            if not callable(flusher):
                continue
            try:
                if flusher() is False and callable(logger):
                    logger(f"[종료] {name} 실패")
            except Exception as exc:  # noqa: BLE001 - report remains authoritative.
                if callable(logger):
                    logger(f"[종료] {name} 실패: {exc}")
        return bool(report["complete"] and evidence_accepted)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override.
        if not self.panel.shutdown_for_close():
            event.ignore()
            QtCore.QTimer.singleShot(250, self.panel.close)
            return
        QtWidgets.QWidget.closeEvent(self.panel, event)

    def _stop_calibration_preview_worker(self, *_args, wait: bool = True) -> bool:
        thread = getattr(self.panel, "_calibration_preview_thread", None)
        if thread is None:
            return True
        thread.quit()
        if thread.isRunning():
            if not wait or not thread.wait(1000):
                return False
        self.panel._calibration_preview_thread = None
        self.panel._calibration_preview_worker = None
        return True
