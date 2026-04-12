#!/bin/bash

# Adjust the configuratoin below as needed, then run with:
# $ conda activate aas
# $ ./performance.sh

if [[ "$CONDA_DEFAULT_ENV" != "aas" ]]; then
    echo "Error: The 'aas' conda environment is not active."
    echo "Please activate it with: conda activate aas"
    exit 1
fi

# Configuration
MODES=("speedup" "vectorenv-speedup")
AUTOPILOTS=("px4" "ardupilot")
SENSOR_SCENARIOS=("both" "no_camera" "no_lidar" "none")
REPETITIONS=1
MAX_RETRIES=3

# Docker clean-up helper function
cleanup_docker() {
    # Check if there are running containers before trying to stop them
    if [ -n "$(docker ps -q)" ]; then
        docker stop $(docker ps -q) >/dev/null 2>&1
    fi
    docker container prune -f >/dev/null 2>&1
    docker network prune -f >/dev/null 2>&1
    # Wait to let the os release socket file handles
    sleep 3
}

suite_start_time=$(date +%s)
{
    for mode in "${MODES[@]}"; do

        # 1. Handle vehicle (quad) counts based on mode
        if [ "$mode" == "speedup" ]; then
            quad_counts="1" # quad_counts="1 2 4 6"
        else
            quad_counts="1" # quad_counts="1 2 3"
        fi

        for autopilot in "${AUTOPILOTS[@]}"; do
            for quads in $quad_counts; do
                for scenario in "${SENSOR_SCENARIOS[@]}"; do

                    # 2. Scenarios
                    case $scenario in
                        "both") sensor_flags="--camera --lidar"; desc="both sensors" ;;
                        "no_camera") sensor_flags="--no-camera --lidar"; desc="no camera" ;;
                        "no_lidar") sensor_flags="--camera --no-lidar"; desc="no lidar" ;;
                        "none") sensor_flags="--no-camera --no-lidar"; desc="neither sensor" ;;
                    esac
                    echo "Running: $mode | $autopilot | $quads quads | $desc"

                    # 3. Execution loop with retries
                    speedup_values=()
                    
                    for (( i=1; i<=REPETITIONS; i++ )); do
                        success=false
                        attempt=1
                        
                        while [ $attempt -le $MAX_RETRIES ]; do

                            output=$(python3 ../scripts/gym_run.py \
                                --mode "$mode" \
                                --autopilot "$autopilot" \
                                --num_quads "$quads" \
                                --repetitions 1 \
                                $sensor_flags 2>&1)

                            exit_code=$?

                            if [ $exit_code -eq 0 ]; then
                                # Case A: SUCCESS
                                # Parse the "Avg Speedup" from the output (expected format: "Avg Speedup:        99.99x wall-clock")
                                val=$(echo "$output" | grep "Avg Speedup:" | sed -E 's/.*: +([0-9.]+)x.*/\1/')
                                
                                if [ -n "$val" ]; then
                                    speedup_values+=($val)
                                    success=true
                                    break # Exit retry loop
                                fi
                            fi

                            # Case B: FAIL
                            # If we end up here, exit_code != 0 OR we failed to parse the value
                            echo ">> Run $i/$REPETITIONS failed (Attempt $attempt/$MAX_RETRIES). Cleaning up and retrying..."
                            cleanup_docker
                            attempt=$((attempt+1))
                        done

                        if [ "$success" = false ]; then
                            echo ">> CRITICAL: Failed run $i after $MAX_RETRIES attempts. Skipping rest of this scenario."
                            break 
                        fi
                    done

                    # 4. Calculate and print statistics
                    if [ ${#speedup_values[@]} -gt 0 ]; then
                        vals_string=$(IFS=,; echo "${speedup_values[*]}")
                        stats=$(python3 -c "
import numpy as np
vals = [$vals_string]
mean = np.mean(vals)
std = np.std(vals)
print(f'{mean:.2f} {std:.2f}')
                    ")
                        read avg_speedup std_speedup <<< "$stats"
                        echo "Avg Speedup:        ${avg_speedup}x ± ${std_speedup}x wall-clock (Avg of ${#speedup_values[@]} runs)"
                    else
                        echo "Avg Speedup:        FAILED (0 successful runs)"
                    fi

                    # 5. Elapsed time update
                    current_time=$(date +%s)
                    elapsed=$(( current_time - suite_start_time ))                    
                    echo "Elapsed Time: ${elapsed}s"

                    # 6. Cooldown between scenarios
                    cleanup_docker
                    sleep 5

                done
            done
        done
    done
} | grep --line-buffered -E "Running:|Avg Speedup:|Elapsed Time:|CRITICAL"
