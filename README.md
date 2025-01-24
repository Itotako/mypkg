# ROS 2 バッテリー監視ノード

[![test](https://github.com/Itotako/mypkg/actions/workflows/test.yml/badge.svg)](https://github.com/Itotako/mypkg/actions/workflows/test.yml)

**このリポジトリは, ROS 2のノードとして動作する「Battery State Publisher」を提供します. このノードは, バッテリーの充電状態と残量を取得し, それぞれ別々のトピックにパブリッシュします.**


## 主な機能
- 現在のバッテリーの充電状態（例: Charging, Not Charging, Unknown）を取得して`/battery_state`トピックに送信します.
- バッテリーの残量（%）を取得して`/battery_level`トピックに送信します.
- バッテリー残量が20%以下の場合に警告をログに出力します.

  
## テスト環境
- Ubuntu 22.04

## 動作確認済み環境
- Ubuntu 20.04
- ROS2 foxy
- Python 3.8.10

## 使用方法
1. 端末1でノードを起動します.
```bash
端末1$ ros2 run mypkg battery_state
```
2. 端末2でノードを起動します.
```bash
端末2$ ros2 run mypkg battery_listener
[INFO] [1737676624.885718312] [battery_monitor]: Received Battery State: Not Charging   # 実行例
[INFO] [1737676624.886110512] [battery_monitor]: Battery Level OK: 77.0%   # 実行例
･･･
```
 
## コードの仕組み
- `acpi`コマンドを使用して, バッテリー情報を取得します.
- バッテリーの充電状態と残量をパースして, 各トピックにパブリッシュします.
- バッテリー残量が20%以下になると, 警告をログに出力します.


## 注意事項
- このプログラムは, `acpi`コマンドを使用してバッテリー情報を取得します. そのため, `acpi`がインストールされていない場合は以下のコマンドでインストールしてください.
```bash
$ sudo apt update
$ sudo apt install acpi
```


## ライセンス
- このソフトウェアパッケージは，**3条項BSDライセンス**の下，再頒布および使用が許可されます．

© 2025 Kaito Ito
