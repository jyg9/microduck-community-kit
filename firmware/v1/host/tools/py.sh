#!/usr/bin/env bash
# Run a Python script with the project's virtualenv interpreter when there is
# one, otherwise with python3.
#
# The venv lives outside this directory (it is shared with the rest of the
# microduck workspace), and its location differs depending on where the script is
# called from, so the search is done here once instead of in every caller.
#
# Override with VENV_PY=/path/to/python, or PYTHON=... to force an interpreter.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"

for candidate in "${PYTHON:-}" "${VENV_PY:-}" \
                 "${here}/../../../.venv/bin/python" \
                 "${here}/../../.venv/bin/python" \
                 "${here}/../.venv/bin/python"; do
    if [ -n "${candidate}" ] && [ -x "${candidate}" ]; then
        exec "${candidate}" "$@"
    fi
done

exec python3 "$@"
