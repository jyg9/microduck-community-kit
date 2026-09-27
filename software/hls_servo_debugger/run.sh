#!/usr/bin/env bash
# HLS 舵机调试工具启动脚本：自动创建 .venv 并安装依赖后启动 Web 服务。
set -euo pipefail

cd "$(dirname "$0")"
export UV_CACHE_DIR="${UV_CACHE_DIR:-${TMPDIR:-/tmp}/uv-cache-hls}"
export UV_LINK_MODE="${UV_LINK_MODE:-copy}"

if [ ! -x ".venv/bin/python" ]; then
  echo "[1/2] 创建虚拟环境 .venv ..."
  uv venv .venv --python python3
  echo "[2/2] 安装 Flask 与 pyserial ..."
  uv pip install --python .venv/bin/python 'Flask>=2.2,<3.0' 'pyserial>=3.5'
else
  echo "使用已有虚拟环境 .venv"
fi

exec .venv/bin/python -m hls_debugger "$@"
