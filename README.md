# rosconjp24_demo

本リポジトリでは [ROSCon JP 2024](https://roscon.jp/2024/) の発表におけるデモの動作手順を示します．

- タイトル：ROS 2のZenoh対応とZenohのROS 2対応
- 発表者：高瀬 英希 (東京大学)
- スライド資料： https://speakerdeck.com/takasehideki/rosconjp-20240925

## 環境構築

ローカル側とサーバ側の双方で準備が必要です．

### サーバ側の環境構築

発表者はMS Azureを使っていますが，他のクラウドサービス等でも同じように準備可能と思われます．

- Ubuntu 24.04 LTS のVMイメージを展開する
- ネットワーク設定で下記のポートを受信／送信ともに追加
  - 3389: リモートデスクトップ用
  - 7447: Zenoh通信用
- [リモートデスクトップの設定](https://learn.microsoft.com/ja-jp/azure/virtual-machines/linux/use-remote-desktop?tabs=azure-cli)
- [ROS 2 rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html) のインストール
  - `sudo apt install ros-rolling-desktop`
  - `echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc`
  - `sudo rosdep init && rosdep update`
- `sudo apt install ros-rolling-rmw-cyclonedds-cpp terminator`
- [rmw_zenoh](https://github.com/ros2/rmw_zenoh) のソースビルド
- [zenoh](https://github.com/eclipse-zenoh/zenoh/releases/tag/1.0.0-beta.3) と [zenoh-plugin-ros2dds Release 1.0.0-beta.3](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/tag/1.0.0-beta.3) の `-gnu-debian.zip` をDL&unzipして `sudo apt install ./*.deb`

