#!/bin/bash
# E2E test orchestrator — runs all 8 scenarios and generates a report.
# Must be run from the test/ directory (where docker-compose.yml lives).
set -e

DURATION=15  # seconds per scenario (10s measurement + warmup)
RESULTS_DIR=/tmp/ntt_results

rm -rf "$RESULTS_DIR"
mkdir -p "$RESULTS_DIR"

# Build images
echo "=== Building test images ==="
docker compose build

echo "=== Starting containers ==="
docker compose up -d

# Wait for containers to be ready
sleep 3

# Helper: run a command in a container
talker() { docker exec ntt_talker bash -c "source /ros2_ws/install/setup.bash && $*"; }
listener() { docker exec ntt_listener bash -c "source /ros2_ws/install/setup.bash && $*"; }

# Helper: kill all ROS processes in both containers
cleanup() {
    echo "  Cleaning up..."
    docker exec ntt_talker bash -c "pkill -f 'ros2\|python3\|udp_sender\|tcp_sender\|publisher' 2>/dev/null || true"
    docker exec ntt_listener bash -c "pkill -f 'ros2\|python3\|udp_receiver\|tcp_receiver\|verify' 2>/dev/null || true"
    # Remove any tc rules
    docker exec ntt_talker bash -c "tc qdisc del dev eth0 root 2>/dev/null || true"
    sleep 2
}

# Helper: run a single scenario
# Usage: run_scenario <name> <protocol> <compress> <loss%> <fec>
run_scenario() {
    local name=$1 protocol=$2 compress=$3 loss=$4 fec=$5

    echo ""
    echo "============================================"
    echo "=== Scenario: $name"
    echo "=== Protocol=$protocol Compress=$compress Loss=$loss FEC=$fec"
    echo "============================================"

    cleanup

    # Apply packet loss if requested
    if [ "$loss" != "0" ]; then
        echo "  Applying ${loss}% packet loss on talker..."
        docker exec ntt_talker tc qdisc add dev eth0 root netem loss ${loss}%
    fi

    # Start receiver on listener
    if [ "$protocol" = "udp" ]; then
        local fec_param=""
        if [ "$fec" = "yes" ]; then
            fec_param="-p fec:=true"
        fi
        echo "  Starting UDP receiver..."
        docker exec -d ntt_listener bash -c "source /ros2_ws/install/setup.bash && \
            ros2 run nimbro_topic_transport udp_receiver --ros-args \
            -p port:=17002 $fec_param \
            2>&1 | tee /results/${name}_receiver.log"
    else
        echo "  Starting TCP receiver..."
        docker exec -d ntt_listener bash -c "source /ros2_ws/install/setup.bash && \
            ros2 run nimbro_topic_transport tcp_receiver --ros-args \
            -p port:=17001 \
            2>&1 | tee /results/${name}_receiver.log"
    fi

    sleep 1

    # Start sender on talker
    local compress_param="false"
    if [ "$compress" = "yes" ]; then
        compress_param="true"
    fi

    if [ "$protocol" = "udp" ]; then
        local fec_val="0.0"
        if [ "$fec" = "yes" ]; then
            fec_val="0.5"  # 50% repair symbols
        fi
        echo "  Starting UDP sender..."
        docker exec -d ntt_talker bash -c "source /ros2_ws/install/setup.bash && \
            ros2 run nimbro_topic_transport udp_sender --ros-args \
            -p destination_addr:=listener \
            -p destination_port:=17002 \
            -p 'topic_names:=[\"/main_chatter\"]' \
            -p 'topic_types:=[\"std_msgs/msg/String\"]' \
            -p 'topic_compress:=[$compress_param]' \
            -p 'topic_rates:=[50.0]' \
            -p fec:=$fec_val \
            2>&1 | tee /results/${name}_sender.log"
    else
        echo "  Starting TCP sender..."
        docker exec -d ntt_talker bash -c "source /ros2_ws/install/setup.bash && \
            ros2 run nimbro_topic_transport tcp_sender --ros-args \
            -p destination_addr:=listener \
            -p destination_port:=17001 \
            -p 'topic_names:=[\"/main_chatter\"]' \
            -p 'topic_types:=[\"std_msgs/msg/String\"]' \
            -p 'topic_compress:=[$compress_param]' \
            2>&1 | tee /results/${name}_sender.log"
    fi

    sleep 2

    # Start publishers on talker (200 noise + 1 main)
    echo "  Starting publishers (200 noise + 1 main)..."
    docker exec -d ntt_talker bash -c "source /ros2_ws/install/setup.bash && \
        python3 /test/publisher_node.py \
        2>&1 | tee /results/${name}_publisher.log"

    sleep 2

    # Start verification on listener
    echo "  Starting verification (${DURATION}s)..."
    docker exec ntt_listener bash -c "source /ros2_ws/install/setup.bash && \
        timeout $((DURATION + 10)) python3 /test/verify_node.py $DURATION $name" \
        2>&1 || true

    # Capture sender stats
    echo "  Capturing sender stats..."
    timeout 4 docker exec ntt_talker bash -c "source /ros2_ws/install/setup.bash && \
        ros2 topic echo /network/sender_stats --once" \
        > "$RESULTS_DIR/${name}_sender_stats.txt" 2>&1 || true

    echo "  Scenario $name complete."
}

# ============================
# Run all scenarios
# ============================

run_scenario "1_udp_baseline"      udp no  0 no
run_scenario "2_tcp_baseline"      tcp no  0 no
run_scenario "3_udp_compress"      udp yes 0 no
run_scenario "4_tcp_compress"      tcp yes 0 no
run_scenario "5_udp_loss2"         udp no  2 no
run_scenario "6_udp_compress_loss2" udp yes 2 no
run_scenario "7_udp_loss2_fec"     udp no  2 yes
run_scenario "8_udp_compress_loss2_fec" udp yes 2 yes

cleanup

echo ""
echo "============================================"
echo "=== All scenarios complete. Generating report..."
echo "============================================"

# Generate report
docker exec ntt_listener bash -c "source /ros2_ws/install/setup.bash && \
    python3 /test/generate_report.py" || true

echo ""
echo "=== Report saved to $RESULTS_DIR/report.md ==="
cat "$RESULTS_DIR/report.md" 2>/dev/null || echo "(report generation failed)"

echo ""
echo "=== Tearing down ==="
docker compose down
