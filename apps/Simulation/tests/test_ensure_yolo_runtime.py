from __future__ import annotations

import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT = ROOT_DIR / "scripts" / "ensure_yolo_runtime.sh"


class EnsureYoloRuntimeTests(unittest.TestCase):
    def test_venv_creation_supports_explicit_isolation(self) -> None:
        text = SCRIPT.read_text(encoding="utf-8")

        self.assertIn("create_virtualenv", text)
        self.assertIn("venv_args+=(--system-site-packages)", text)
        self.assertIn("-m virtualenv", text)
        self.assertIn("virtualenv_args=(--clear)", text)
        self.assertIn('if [[ "$YOLO_VENV_ISOLATED" == true ]]', text)
        self.assertIn("configure_system_site_packages", text)
        self.assertIn("include-system-site-packages = %s", text)

    def test_existing_venv_is_normalized_before_runtime_check(self) -> None:
        text = SCRIPT.read_text(encoding="utf-8")
        existing_venv_block = text[
            text.index('if [[ -x "${VENV_DIR}/bin/python" ]]') : text.index("if [[ -n \"$PREFERRED_PYTHON\" ]]")
        ]

        self.assertIn("configure_system_site_packages", existing_venv_block)
        self.assertLess(
            existing_venv_block.index("configure_system_site_packages"),
            existing_venv_block.index("python_has_yolo_runtime"),
        )

    def test_isolated_mode_never_returns_the_preferred_system_runtime(self) -> None:
        text = SCRIPT.read_text(encoding="utf-8")
        preferred_block = text[
            text.index('if [[ -n "$PREFERRED_PYTHON" ]]') : text.index('BUILDER_PYTHON="${BUILDER_PYTHON:-}"')
        ]

        self.assertIn('[[ "$YOLO_VENV_ISOLATED" != true ]]', preferred_block)

    def test_matching_cuda_wheels_do_not_reinstall_on_device_failure(self) -> None:
        text = SCRIPT.read_text(encoding="utf-8")
        existing_venv_block = text[
            text.index('if [[ -x "${VENV_DIR}/bin/python" ]]') : text.index('if [[ -n "$PREFERRED_PYTHON" ]]')
        ]

        self.assertIn("python_matches_torch_distribution", existing_venv_block)
        self.assertIn("report_backend_failure", existing_venv_block)
        self.assertIn("exit 1", existing_venv_block)
        self.assertNotIn("install_yolo_runtime", existing_venv_block)

    def test_torch_backend_can_require_cuda_wheels(self) -> None:
        text = SCRIPT.read_text(encoding="utf-8")

        self.assertIn("YOLO_TORCH_BACKEND", text)
        self.assertIn("YOLO_TORCH_CUDA_INDEX_URL", text)
        self.assertIn("torch.version.cuda", text)
        self.assertIn("torch.cuda.is_available()", text)
        self.assertIn("torch.zeros(1, device='cuda:0')", text)
        self.assertIn("python_matches_torch_distribution", text)
        self.assertIn("report_backend_success", text)
        self.assertIn("YOLO runtime verified:", text)
        self.assertIn("user_site_enabled={site.ENABLE_USER_SITE}", text)
        self.assertIn('torch.zeros(1, device="cuda:0")', text)
        self.assertIn("YOLO_TORCHVISION_VERSION", text)
        self.assertIn("YOLO_TORCHAUDIO_VERSION", text)
        self.assertIn("YOLO_ULTRALYTICS_VERSION", text)
        self.assertIn('ultralytics_package="ultralytics==$YOLO_ULTRALYTICS_VERSION"', text)
        self.assertIn("--force-reinstall --index-url", text)


if __name__ == "__main__":
    unittest.main()
