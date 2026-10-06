#!/usr/bin/env bash
# HLS 舵机调试工具启动脚本。三种环境都用这一份：
#
#   ./run.sh                          有 uv 的开发机/台面：建 .venv 装 Flask+pyserial 再启动
#   ./run.sh --host 0.0.0.0           机器鸭核心板：没有 uv，退回系统 python3 + apt 装的依赖
#   ./run.sh --serial /dev/ttyACM1    指定默认串口（也可以设 DUCK_SERVO_PORT）
#
# 核心板上串口是 **/dev/ttyS2**（robotd.toml 的 bus.port）；台面调试器一般是 /dev/ttyACM1。
# 只在调用方没有指定、也没设环境变量时，本脚本才在发现 /dev/ttyS2 后把它当默认串口。
set -euo pipefail

cd "$(dirname "$0")"
export UV_CACHE_DIR="${UV_CACHE_DIR:-${TMPDIR:-/tmp}/uv-cache-hls}"
export UV_LINK_MODE="${UV_LINK_MODE:-copy}"

serial_given=0
for arg in "$@"; do
  case "$arg" in
    --serial|--serial=*) serial_given=1 ;;
  esac
done

DEFAULT_SERIAL_ARGS=()
if [ "$serial_given" -eq 0 ] && [ -z "${DUCK_SERVO_PORT:-}" ] && [ -e /dev/ttyS2 ]; then
  DEFAULT_SERIAL_ARGS=(--serial /dev/ttyS2)
fi

if [ -x ".venv/bin/python" ]; then
  echo "使用已有虚拟环境 .venv"
  exec .venv/bin/python -m hls_debugger ${DEFAULT_SERIAL_ARGS[@]+"${DEFAULT_SERIAL_ARGS[@]}"} "$@"
fi

if command -v uv >/dev/null 2>&1; then
  echo "[1/2] 创建虚拟环境 .venv ..."
  uv venv .venv --python python3
  echo "[2/2] 安装 Flask 与 pyserial ..."
  uv pip install --python .venv/bin/python 'Flask>=2.2,<3.0' 'pyserial>=3.5'
  exec .venv/bin/python -m hls_debugger ${DEFAULT_SERIAL_ARGS[@]+"${DEFAULT_SERIAL_ARGS[@]}"} "$@"
fi

# 没有 uv（核心板就是这样）：用系统 python3，依赖由发行版的 python3-flask / python3-serial 提供。
if python3 -c 'import flask, serial' >/dev/null 2>&1; then
  exec python3 -m hls_debugger ${DEFAULT_SERIAL_ARGS[@]+"${DEFAULT_SERIAL_ARGS[@]}"} "$@"
fi

cat >&2 <<'MSG'
没有 uv，系统 python3 也缺 Flask / pyserial。核心板（Armbian / Ubuntu）上装一下：

    apt-get update
    apt-get install -y python3-flask python3-serial

或者在开发机上先跑一次 ./run.sh，让它建好 .venv 一起拷过去。
MSG
exit 1
