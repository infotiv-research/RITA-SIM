#!/bin/bash
set -euo pipefail

cd /ros2_ws

retry() {
    local attempts="$1"
    local delay_seconds="$2"
    local description="$3"
    shift 3

    local attempt=1
    while true; do
        if "$@"; then
            return 0
        fi

        local exit_code=$?
        if [ "$attempt" -ge "$attempts" ]; then
            echo "${description} failed after ${attempts} attempts."
            return "$exit_code"
        fi

        echo "${description} failed with exit code ${exit_code} (attempt ${attempt}/${attempts}). Retrying in ${delay_seconds}s..."
        sleep "$delay_seconds"
        attempt=$((attempt + 1))
    done
}

install_rosdeps() {
    sudo -H apt-get update
    rosdep install --from-paths src --ignore-src -r -y --skip-keys=keyboard
}

ensure_serial_repo() {
    if [ -d "src/serial/.git" ]; then
        echo "Serial package already exists, skipping clone"
        return 0
    fi

    if [ -e "src/serial" ]; then
        echo "src/serial exists but is not a git checkout, skipping automatic clone"
        return 0
    fi

    git clone -b ros2 https://github.com/tylerjw/serial.git src/serial
}

echo "🔧 Running rosdep update..."
retry 3 5 "rosdep update" rosdep update

echo "📦 Installing dependencies with rosdep..."
retry 3 10 "rosdep install" install_rosdeps

echo "Ensuring serial package is present..."
retry 3 5 "serial repository clone" ensure_serial_repo

echo "✅ Done!"
