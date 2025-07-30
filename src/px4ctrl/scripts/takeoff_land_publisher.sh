#!/bin/bash

# PX4Ctrl 起飞降落话题发布脚本
# 使用方法：
# ./takeoff_land_publisher.sh takeoff    # 发布起飞命令 (true)
# ./takeoff_land_publisher.sh land       # 发布降落命令 (false)

if [ $# -eq 0 ]; then
    echo "使用方法："
    echo "  $0 takeoff    # 发布起飞命令"
    echo "  $0 land       # 发布降落命令"
    exit 1
fi

# 设置话题名称
TOPIC="/px4ctrl/takeoff_land"

case "$1" in
    "takeoff")
        echo "发布起飞命令..."
        rostopic pub -1 $TOPIC std_msgs/Bool "data: true"
        echo "起飞命令已发布！"
        ;;
    "land")
        echo "发布降落命令..."
        rostopic pub -1 $TOPIC std_msgs/Bool "data: false"
        echo "降落命令已发布！"
        ;;
    *)
        echo "错误：未知的命令 '$1'"
        echo "支持的命令：takeoff, land"
        exit 1
        ;;
esac
