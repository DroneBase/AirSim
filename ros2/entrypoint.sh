#!/usr/bin/env bash

SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

DEBUG="false"
ZV2_METADATA="false"

function usage() {
    echo "Usage: ${0} [ --debug ] [ --zv2_metadata ]" 1>&2
}

function oops() {
    usage
    exit 1
}

function parse_args() {
    local OPTIND

    source_getopts_long

    while getopts_long "dz debug zv2_metadata" option "${@}"; do
        echo "option: ${option}"
        case "${option}" in
            "d" | "debug")
                DEBUG="true"
                ;;
            "z" | "zv2_metadata")
                ZV2_METADATA="true"
                ;;
            ":")
                echo "Error: -${OPTARG} requires an argument."
                oops
                ;;
            *)
                oops
                ;;
        esac
    done
}

function ensure_getopts_long() {
    if [ ! -d "${SCRIPT_DIR}/getopts_long" ]; then
        git clone https://github.com/UrsaDK/getopts_long.git "${SCRIPT_DIR}/getopts_long"
    fi
}

function source_getopts_long() {
    ensure_getopts_long
    source "${SCRIPT_DIR}/getopts_long/lib/getopts_long.bash"
}

function run() {
    ensure_getopts_long

    ROS2_LAUNCH_COMMAND=(ros2 launch airsim_ros_pkgs airsim_node.launch.py)

    if [ "${DEBUG}" == "true" ]; then
        ROS2_LAUNCH_COMMAND+=("debug:=True")
    fi

    if [ "${ZV2_METADATA}" == "true" ]; then
        ROS2_LAUNCH_COMMAND+=("zv2_metadata:=True")
    fi

    source /opt/ros/iron/setup.bash
    source /home/ros/AirSim/ros2/install/local_setup.bash

    ${ROS2_LAUNCH_COMMAND[@]}
}

echo "args: ${@}"

parse_args "${@}"
shift $(( OPTIND - 1 ))

run
