#!/bin/bash -xv
# SPDX-FileCopyrightText: 2024 Kaito Ito
# SPDX-License-Identifier: BSD-3-Clause


# リセット: 以前の状態を確認
ros2 topic list | grep "/battery_state" > /dev/null
if [ $? -eq 0 ]; then
  echo "battery_state トピックは既に存在します"
else
  echo "battery_state トピックが見つかりません"
  exit 1
fi

ros2 topic list | grep "/battery_level" > /dev/null
if [ $? -eq 0 ]; then
  echo "battery_level トピックは既に存在します"
else
  echo "battery_level トピックが見つかりません"
  exit 1
fi

# ノードをバックグラウンドで実行
ros2 run mypkg battery_state & 
NODE_PID=$!

# 少し待機してからトピックリストを確認
sleep 5

# トピックのメッセージを確認
echo "battery_state トピックの確認"
timeout 10 ros2 topic echo /battery_state > output_state.log &
ECHO_STATE_PID=$!
sleep 5
kill $ECHO_STATE_PID

echo "battery_level トピックの確認"
timeout 10 ros2 topic echo /battery_level > output_level.log &
ECHO_LEVEL_PID=$!
sleep 5
kill $ECHO_LEVEL_PID

# 出力が空でないか確認
if [ -s output_state.log ]; then
    echo "battery_state トピックの出力が確認できました"
else
    echo "battery_state トピックの出力がありません"
    exit 1
fi

if [ -s output_level.log ]; then
    echo "battery_level トピックの出力が確認できました"
else
    echo "battery_level トピックの出力がありません"
    exit 1
fi

# 一時ファイルを削除
rm -f output_state.log output_level.log

echo "テストが成功しました"
exit 0

