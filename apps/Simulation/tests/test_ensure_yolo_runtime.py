from __future__ import annotations

import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT = ROOT_DIR / "scripts" / "ensure_yolo_runtime.sh"


class EnsureYoloRuntimeTests(unittest.TestCase):
    def test_venv_creation_preserves_system_site_packages(self) -> None:
        text = SCRIPT.read_text(encoding="utf-8")

        self.assertIn("-m venv --system-site-packages", text)
        self.assertIn("ensure_system_site_packages", text)
        self.assertIn("include-system-site-packages = true", text)

    def test_existing_venv_is_normalized_before_runtime_check(self) -> None:
        text = SCRIPT.read_text(encoding="utf-8")
        existing_venv_block = text[
            text.index('if [[ -x "${VENV_DIR}/bin/python" ]]') : text.index("if [[ -n \"$PREFERRED_PYTHON\" ]]")
        ]

        self.assertIn("ensure_system_site_packages", existing_venv_block)
        self.assertLess(
            existing_venv_block.index("ensure_system_site_packages"),
            existing_venv_block.index("python_has_yolo_runtime"),
        )

    def test_torch_backend_can_require_cuda_wheels(self) -> None:
        text = SCRIPT.read_text(encoding="utf-8")

        self.assertIn("YOLO_TORCH_BACKEND", text)
        self.assertIn("YOLO_TORCH_CUDA_INDEX_URL", text)
        self.assertIn("torch.version.cuda", text)
        self.assertIn("--force-reinstall --index-url", text)


if __name__ == "__main__":
    unittest.main()
