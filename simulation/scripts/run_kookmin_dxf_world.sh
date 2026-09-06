#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
world_file="${script_dir}/../worlds/kookmin_track_from_dxf.world.sdf"

if ! command -v gz >/dev/null 2>&1; then
  echo "오류: 'gz' 명령을 찾을 수 없습니다. Gazebo Sim을 먼저 설치해 주세요." >&2
  exit 127
fi

if [[ ! -f "${world_file}" ]]; then
  echo "오류: 월드 파일이 없습니다: ${world_file}" >&2
  exit 1
fi

exec gz sim -r "${world_file}" "$@"
