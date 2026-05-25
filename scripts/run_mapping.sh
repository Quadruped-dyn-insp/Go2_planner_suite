#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

source "$SCRIPT_DIR/source_workspaces.sh"

usage() {
    echo "Usage: $0 [mola-online|mola-gui|mola-gui-fixed|sm2mm|view-mm] [args]"
    echo ""
    echo "Commands:"
    echo "  mola-online                 Run MOLA online mapping (start_active:=True)"
    echo "  mola-gui [bag_path]         Generate myMap.simplemap with base_link"
    echo "  mola-gui-fixed [bag_path]   Generate myMap.simplemap with fixed poses (Head_upper)"
    echo "  sm2mm                       Convert myMap.simplemap -> myMap.mm"
    echo "  view-mm                     Open myMap.mm in mm-viewer"
}

cmd="${1:-mola-online}"
case "$cmd" in
    mola-online)
        export MOLA_LO_PUBLISH_DESKEWED_SCANS=true
        cd "$HOME/ros2_mola_ws"
        ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
            start_active:=True \
            publish_localization_following_rep105:=False \
            start_mapping_enabled:=False \
            lidar_topic_name:="/livox/lidar" \
            imu_topic_name:="/livox/imu" \
            mola_tf_base_link:="base_link" \
            mola_deskew_method:="MotionCompensationMethod::IMU" \
            mola_initial_map_mm_file:="$(pwd)/myMap.mm"
        ;;

    mola-gui)
        BAG_PATH="${2:-$WORKSPACE_ROOT/rosbags/rosbag2_2026_03_03-14_46_17}"
        MOLA_GENERATE_SIMPLEMAP=true \
        MOLA_SIMPLEMAP_OUTPUT="myMap.simplemap" \
        MOLA_SIMPLEMAP_MIN_XYZ=0.2 \
        MOLA_LO_INITIAL_LOCALIZATION_METHOD="InitLocalization::PitchAndRollFromIMU" \
        MOLA_DESKEW_METHOD="MotionCompensationMethod::IMU" \
        MOLA_IMU_TOPIC="/livox/imu" \
        MOLA_LIDAR_TOPIC="/livox/lidar" \
        MOLA_TF_BASE_LINK="base_link" \
        mola-lo-gui-rosbag2 "$BAG_PATH"
        ;;

    mola-gui-fixed)
        BAG_PATH="${2:-/home/yasiru/Documents/rosbags/rosbag_004}"
        MOLA_USE_FIXED_LIDAR_POSE=true \
        MOLA_USE_FIXED_IMU_POSE=true \
        MOLA_GENERATE_SIMPLEMAP=true \
        MOLA_SIMPLEMAP_OUTPUT="myMap.simplemap" \
        MOLA_SIMPLEMAP_MIN_XYZ=0.2 \
        MOLA_LO_INITIAL_LOCALIZATION_METHOD="InitLocalization::PitchAndRollFromIMU" \
        MOLA_DESKEW_METHOD="MotionCompensationMethod::IMU" \
        MOLA_IMU_TOPIC="/livox/imu" \
        MOLA_LIDAR_TOPIC="/livox/lidar" \
        MOLA_TF_BASE_LINK="Head_upper" \
        mola-lo-gui-rosbag2 "$BAG_PATH"
        ;;

    sm2mm)
        sm2mm -i myMap.simplemap -o myMap.mm -p sm2mm_no_decim_imu_mls_keyframe_map.yaml
        ;;

    view-mm)
        mm-viewer myMap.mm -l libmola_metric_maps.so
        ;;

    -h|--help|help)
        usage
        ;;

    *)
        echo "Unknown command: $cmd"
        usage
        exit 1
        ;;
esac
