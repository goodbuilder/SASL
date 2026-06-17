#!/usr/bin/env bash
set -euo pipefail

label="${1:-snapshot}"
timestamp="$(date +%Y%m%d-%H%M%S)"
safe_label="$(printf '%s' "$label" | tr -cs '[:alnum:]_.-' '-')"
tag="snapshot-${timestamp}-${safe_label}"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Not inside a git repository." >&2
  exit 1
fi

git add -A

if git diff --cached --quiet; then
  echo "No staged changes. Existing HEAD: $(git rev-parse --short HEAD)"
else
  git commit -m "Snapshot ${timestamp}: ${label}"
fi

git tag -a "$tag" -m "Snapshot ${timestamp}: ${label}"
git push origin HEAD
git push origin "$tag"

echo "Created and pushed $tag"
