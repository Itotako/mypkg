#!/bin/bash -xv
# SPDX-FileCopyrightText: 2024 Takumi Kobayashi
# SPDX-License-Identifier: BSD-3-Clause

dir=~
[ "$1" != "" ] && dir="$1"

# ROS2 ワークスペースをビルド
cd $dir/ros2_ws
colcon build

# ROS2 環境を設定
source install/setup.bash
source $dir/.bashrc

# エラーチェック用関数
ng() {
    echo "${1}行目が違う"
    res=1
}

res=0

# acpi コマンドがインストールされているか確認
if ! command -v acpi &> /dev/null; then
    echo "acpi コマンドが見つかりません。インストールしてください。"
    exit 1
fi

# battery_state ノードをバックグラウンドで実行
ros2 run mypkg battery_state & 
NODE_PID=$!

# 少し待機してからトピックリストを確認
sleep 5

# トピック '/battery_state' が存在するか確認
ros2 topic list | grep "/battery_state" > /dev/null
if [ $? -ne 0 ]; then
    ng "battery_state トピックが見つかりません"
fi

# トピック '/battery_level' が存在するか確認
ros2 topic list | grep "/battery_level" > /dev/null
if [ $? -ne 0 ]; then
    ng "battery_level トピックが見つかりません"
fi

# トピックの出力を確認
timeout 10 ros2 topic echo /battery_state > output_state.log &
ECHO_STATE_PID=$!

timeout 10 ros2 topic echo /battery_level > output_level.log &
ECHO_LEVEL_PID=$!

# 少し待機してから出力を終了
sleep 5
kill "$ECHO_STATE_PID" 2>/dev/null
kill "$ECHO_LEVEL_PID" 2>/dev/null

# 出力ファイルに内容があるか確認
if [ -s output_state.log ]; then
    grep "Publishing State" output_state.log > /dev/null
    if [ $? -ne 0 ]; then
        ng "battery_state トピックの出力が正しくありません"
    fi
else
    ng "battery_state トピックの出力がありません"
fi

if [ -s output_level.log ]; then
    grep "Publishing Battery Level" output_level.log > /dev/null
    if [ $? -ne 0 ]; then
        ng "battery_level トピックの出力が正しくありません"
    fi
else
    ng "battery_level トピックの出力がありません"
fi

# 一時的なファイルを削除
rm -f output_state.log output_level.log

# 結果表示
[ "$res" = 0 ] && echo "OK"
exit "$res"

