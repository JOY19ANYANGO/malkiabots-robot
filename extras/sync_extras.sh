#!/usr/bin/env bash
# Copy the files that run from the home folder into the repo. Run on the Pi before every commit.
set -e
cd "$(dirname "$0")"
cp -v ~/real_inputs.py scripts/ 2>/dev/null || echo "missing ~/real_inputs.py"
for f in hailo_waste_detector.py start_waste_detector.sh waste-detector.service; do
  cp -v ~/waste/$f waste_detector/ 2>/dev/null || echo "missing ~/waste/$f"
done
for f in gamefield_nopillars.pgm gamefield_nopillars.yaml mission.yaml; do
  cp -v ~/maps/$f maps/ 2>/dev/null || echo "missing ~/maps/$f"
done
if ! cmp -s maps/mission.yaml ../malkia_mission/config/mission.yaml; then
  echo "WARNING: ~/maps/mission.yaml and malkia_mission/config/mission.yaml differ - copy one over the other"
fi
echo "extras synced"
