#!/usr/bin/env bash
# Runs mainVRPTWDC.jl on every Solomon instance with durations 360, 480 and 600.
# Usage (from anywhere): bash Applications/VRPTWDC/runSolomon.sh
# Logs go to Applications/VRPTWDC/out/ (ignored by git).
APP="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$APP/out"

for f in "$APP"/Solomon/*.txt; do
    filename=$(basename "$f" .txt)
    for dur in 360 480 600; do
        echo "Running $filename with duration $dur..."
        julia --project="$APP" "$APP/mainVRPTWDC.jl" "$filename.txt" "$dur" 2>&1 | tee "$APP/out/${filename}_${dur}.log"
    done
done
