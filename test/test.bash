#!/bin/bash -xv
# SPDX-FileCopyrightText: 2024 Kaito Ito
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
    echo "$1"
    res=1  # エラー発生時にresを1に設定
}

res=0  # 初期化

# battery_state_publisher ノードをバックグラウンドで実行
ros2 run mypkg battery_state_publisher & 
NODE_PID=$!

# 少し待機してからトピックリストを確認
sleep 5

# acpi コマンドを実行して結果を取得
result=$( /usr/bin/acpi )

# 出力結果を表示
echo "ACPI Output: $result"

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
    echo "battery_state トピックの出力が確認できました"
else
    ng "battery_state トピックの出力がありません"
fi

if [ -s output_level.log ]; then
    echo "battery_level トピックの出力が確認できました"
else
    ng "battery_level トピックの出力がありません"
fi

# 一時的なファイルを削除
rm -f output_state.log output_level.log

# 結果表示
if [ "$res" -eq 0 ]; then
    echo "OK"
else
    echo "エラーが発生しました"
fi

exit "$res"

