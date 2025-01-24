#!/bin/bash
# SPDX-FileCopyrightText: 2025 Kaito Ito
# SPDX-License-Identifier: BSD-3-Clause


# ディレクトリの設定
dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source $dir/.bashrc
ros2 launch mypkg battery.launch.py &

timeout 10 ros2 topic echo /battery_state > /tmp/battery_state.log
timeout 10 ros2 topic echo /battery_level > /tmp/battery_level.log

# テスト結果の確認
echo "Checking /battery_state log..."
cat /tmp/battery_state.log | grep -E 'Charging|Not Charging|Unknown'

echo "Checking /battery_level log..."
cat /tmp/battery_level.log | grep -E '[0-9]+(\.[0-9]+)?'

# テスト結果のログファイルを表示
echo "Battery State Log:"
cat /tmp/battery_state.log

echo "Battery Level Log:"
cat /tmp/battery_level.log
