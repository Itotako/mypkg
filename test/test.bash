#!/bin/bash
# SPDX-FileCopyrightText: 2024 Kaito Ito
# SPDX-License-Identifier: BSD-3-Clause

set -e  # エラー発生時にスクリプトを停止

# ディレクトリの設定
dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws

# ワークスペースをビルドして環境をセットアップ
echo "Building the workspace..."
colcon build || { echo "Failed to build the workspace."; exit 1; }

echo "Sourcing the ROS2 environment..."
source install/setup.bash || { echo "Failed to source ROS2 setup."; exit 1; }

# `ros2` コマンドが使用可能か確認
command -v ros2 >/dev/null 2>&1 || { echo "ros2 command not found. Ensure ROS2 is correctly installed."; exit 1; }

# ログファイルの設定
LOG_FILE="/tmp/mypkg.log"
echo "Log file: $LOG_FILE"

# ノードを起動してログに出力
echo "Launching ROS2 node..."
timeout 20 ros2 launch mypkg battery_listen.launch.py > $LOG_FILE 2>&1 &
LAUNCH_PID=$!

# ノードの起動を確認
sleep 5  # ノードが起動するのを待機

# トピックのデータ確認
check_topic() {
  local topic_name="$1"
  local message="$2"
  if grep -q "$message" "$LOG_FILE"; then
    echo "$topic_name トピックにデータがパブリッシュされています。"
  else
    echo "$topic_name トピックにデータがパブリッシュされていません！"
    kill $LAUNCH_PID
    exit 1
  fi
}

check_topic "battery_state" "Publishing State:"
check_topic "battery_level" "Publishing Battery Level:"

# テスト成功
echo "テストが成功しました！"

# 起動したプロセスを終了
kill $LAUNCH_PID

