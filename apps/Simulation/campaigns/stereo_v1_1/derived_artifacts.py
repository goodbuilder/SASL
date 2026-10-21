"""Exclusive hardlinks between immutable files owned by one derived analysis.

Callers must provide the dedicated analysis root, never a campaign/raw root,
keep generated sources immutable, and preserve hardlink topology when archiving
if they rely on the storage saving. No cross-device copy fallback is permitted.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import stat


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def link_immutable_derived(source: Path, target: Path, owned_root: Path) -> str:
    """Return SHA256 after linking/verifying two owned immutable derived paths.

    Existing targets are never replaced. A failed post-link verification leaves
    the evidence in place and raises; it never deletes either artifact. EXDEV
    propagates so the caller must reconsider capacity instead of silently copying.
    This detects changes during the operation, not prohibited later mutations.
    """
    root = Path(owned_root).resolve(strict=True)
    if not root.is_dir():
        raise ValueError('owned_root must be an existing derived analysis directory')
    source = Path(source).resolve(strict=True)
    target = Path(target).resolve(strict=False)
    for path in (source, target):
        if path == root or not path.is_relative_to(root):
            raise ValueError('source and target must belong to the same derived analysis root')
        # Defense in depth if a caller accidentally supplies a campaign root.
        if any(part in {'원본_실행별', 'raw_images_or_lossless_bag'}
               for part in path.parts):
            raise ValueError('raw recording paths cannot be shared as derived artifacts')
    if not stat.S_ISREG(source.stat().st_mode):
        raise ValueError('source must be a regular file')
    source_before = _sha256(source)
    target.parent.mkdir(parents=True, exist_ok=True)
    # Recheck resolved paths after directory creation to reject symlink escapes.
    if target.parent.resolve() != target.parent or not target.parent.is_relative_to(root):
        raise ValueError('target parent changed during ownership validation')
    try:
        os.link(source, target)
    except FileExistsError:
        if not stat.S_ISREG(target.stat().st_mode):
            raise ValueError('existing target must be a regular file') from None
    target_after = _sha256(target)
    source_after = _sha256(source)
    if target_after != source_before or source_after != source_before:
        raise ValueError('immutable derived artifact changed or existing target differs')
    return source_before
