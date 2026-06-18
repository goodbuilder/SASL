#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/git_snapshot.sh [--dry-run] [label]

Stages all changes, commits them if needed, creates an annotated snapshot tag,
and pushes HEAD plus the tag to origin.

Options:
  --dry-run   Print the planned snapshot commands without changing git state.
  --help      Show this help.
USAGE
}

dry_run=false
case "${1:-}" in
  --help|-h)
    usage
    exit 0
    ;;
  --dry-run)
    dry_run=true
    shift
    ;;
esac

label="${1:-snapshot}"
timestamp="$(date +%Y%m%d-%H%M%S)"
safe_label="$(printf '%s' "$label" | tr -cs '[:alnum:]_.-' '-')"
tag="snapshot-${timestamp}-${safe_label}"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Not inside a git repository." >&2
  exit 1
fi

if [[ "$dry_run" == true ]]; then
  echo "Dry run snapshot label: $label"
  echo "Dry run tag: $tag"
  echo "Would run: git add -A"
  if git diff --quiet && git diff --cached --quiet; then
    echo "Working tree has no changes to commit."
  else
    echo "Would commit if staged changes exist after git add."
  fi
  echo "Would run: git tag -a $tag -m Snapshot ${timestamp}: ${label}"
  echo "Would run: git push origin HEAD"
  echo "Would run: git push origin $tag"
  exit 0
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
