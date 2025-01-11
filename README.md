# バッテリー状態発信器

![Test Battery State Publisher](https://github.com/Itotako/mypkg/actions/workflows/test.yml/badge.svg)

**このリポジトリは, ROS 2のノードとして動作する「Battery State Publisher」を提供します. このノードは, バッテリーの充電状態と残量を取得し, それぞれ別々のトピックにパブリッシュします.**


## 主な機能
- 現在のバッテリーの充電状態（例: Charging, Not Charging, Unknown）を取得して`battery_state`トピックに送信します.
- バッテリーの残量（%）を取得して`battery_level`トピックに送信します.
- バッテリー残量が20%以下の場合に警告をログに出力します.

  
## 必要な環境
- 以下の環境と条件を満たしている必要があります:
  - Ubuntu 20.04 (テスト環境: Ubuntu 22.04 LTS)
  - ROS 2 (Foxy)
  - ACPIコマンドが利用可能な環境


- 動作確認済み環境:
  - Ubuntu 20.04 + ROS 2 foxy
  - Python 3.8.10


## 使用方法
1. 一つ目の端末でノードを起動します.
```bash
ros2 run mypkg battery_state_publisher
```
2. `1.`とは別の二つ目の端末でトピックを確認します.
  - `battery_state`:バッテリーの充電状態
  - `battery_level`:バッテリーの残量
トピックのデータを受信するには以下を使用します.
```bash
ros2 topic echo /battery_state
ros2 topic echo /battery_level
```


## コードの仕組み
- `acpi`コマンドを使用して, バッテリー情報を取得します.
- バッテリーの充電状態と残量をパースして, 各トピックにパブリッシュします.
- バッテリー残量が20%以下になると, 警告をログに出力します.


## 注意事項
- このプログラムは, `acpi`コマンドを使用してバッテリー情報を取得します. そのため, `acpi`がインストールされていない場合は以下のコマンドでインストールしてください.
```bash
sudo apt update
sudo apt install acpi
```


## ライセンス
- このソフトウェアパッケージは，**3条項BSDライセンス**の下，再頒布および使用が許可されます．
© 2025 Kaito Ito
