#!/bin/bash

# Source environment setup (shared across all processes)
cd /home/user/pol_ws
source install/setup.bash


# Start each process in a subshell with inherited environment
(
    echo "Starting ROS polaris_joystick node, dbw node) ..."
    ros2 launch polaris_joystick joystick.launch.xml
) &
PID1=$!

(
    echo "Starting ROS mavros_commutator node, mavros_ugv_telemetry node, gps_reader node ..."
    ros2 launch mavros_commutator run.xml 
) &
PID2=$!

(
    echo "Starting microphone streamer (gst-launch-1.0 pipeline) on port 5008..."
    gst-launch-1.0 alsasrc device=hw:1,0 ! audioconvert ! audioresample ! mulawenc ! rtppcmupay ! udpsink host=172.25.64.194 port=5008
) &
PID3=$!

(
    echo "Starting ZED X (stereo camera) streamer on port 5002..."
    sleep 30
    ./build/ZED_Streaming_Sender/ZED_Streaming_Sender 5002
) &
PID4=$!

(
    echo "Starting ZED One (mono camera) streamer on port 5004..."
    sleep 30
    ./build/ZED_One_live/ZED_One_streaming_sender 5004

) &
PID5=$!

# Store all PIDs
PIDS=($PID1 $PID2 $PID3 $PID4 $PID5)

# Optional: trap for safe cleanup on Ctrl+C or termination
trap 'echo "Terminating..."; for pid in "${PIDS[@]}"; do kill $pid; done; exit' SIGINT SIGTERM

# Wait for all processes (or use sleep/monitor loop)
wait