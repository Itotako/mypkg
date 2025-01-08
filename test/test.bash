#!/bin/bash
# SPDX-FileCopyrightText: 2024 Kaito Ito
# SPDX-License-Identifier: BSD-3-Clause

set -e  # エラー発生時にスクリプトを停止

# ディレクトリの設定
dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws

# ワークスペースをビルドして環境をセットアップ
colcon build
source install/setup.bash

# ログファイルの設定
LOG_FILE="/tmp/mypkg.log"

# ノードを起動してログに出力
timeout 20 ros2 launch mypkg battery_listen.launch.py > $LOG_FILE &

# ノードの起動を確認
sleep 5  # ノードが起動するのを待機

# battery_stateトピックのデータ確認
if grep -q "Publishing State:" $LOG_FILE; then
  echo "battery_state トピックにデータがパブリッシュされています。"
else
  echo "battery_state トピックにデータがパブリッシュされていません！"
  kill %1
  exit 1
fi

# battery_levelトピックのデータ確認
if grep -q "Publishing Battery Level:" $LOG_FILE; then
  echo "battery_level トピックにデータがパブリッシュされています。"
else
  echo "battery_level トピックにデータがパブリッシュされていません！"
  kill %1
  exit 1
fi

# テスト成功
echo "テストが成功しました！"

# 起動したプロセスを終了
kill %1

