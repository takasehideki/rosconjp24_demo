# rosconjp24_demo

本リポジトリでは [ROSCon JP 2024](https://roscon.jp/2024/) の発表におけるデモの動作手順を示します．

- タイトル：ROS 2のZenoh対応とZenohのROS 2対応
- 発表者：高瀬 英希 (東京大学)
- スライド資料： https://speakerdeck.com/takasehideki/rosconjp-20240925

## 環境構築

ローカル側とクラウド側の双方で準備が必要です．

### ローカル側の環境構築

Docker環境を利用します．
Zenoh通信用のポート `7447` をホストとバインドしており，Linux以外のOSでも動作可能と思われます（発表時にはmacOSを利用）．

- イメージのビルド： `docker compose build`
- コンテナの起動： `docker compose up -d`
- コンテナの終了： `docker compose down`

以降のローカル側での実行手順は `docker compose exec app bash` によってコンテナに入ったシェル内での操作として示します．

### クラウド側の環境構築

発表者はMS Azureを使っていますが，自前のオンプレサーバや他のクラウドサービス等でも同じように準備可能と思われます．

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

## デモ１：zenoh-bridge-ros2dds（on p.11）

[zenoh-plugin-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) によってクラウドとローカルのROS 2システムを接続してみます．

クラウド側では２つ，ローカル側では４つのターミナルを立ち上げます．

### クラウド側

ターミナル１では `zenoh-bridge-ros2dds` を起動します．

```
zenoh-bridge-ros2dds
```

ターミナル２では `turtlesim` を起動しておきます．

```
source /opt/ros/rolling/setup.bash
ros2 run turtlesim turtlesim_node
```

### ローカル側

ターミナル１では `zenoh-bridge-ros2dds` を起動します．
引数では，JSONファイルとクラウドのIPアドレスを指定します（下記の `192.168.1.1` は例です）．

```
zenoh-bridge-ros2dds -c ROS2DDS.json5 -e "tcp/192.168.1.1:7447"
```

JSONファイルでは，pub/subを許可（allow）するトピック名を明示しています．
デフォルトのファイルからの変更は [こちら](https://github.com/takasehideki/rosconjp24_demo/commit/3c82fbfb6b562c86f9e54305f5009b09bcc4bc90) を参照ください．

ターミナル２では `turtle_teleop_key` を起動します．

```
source /opt/ros/rolling/setup.bash
ros2 run turtlesim turtle_teleop_key
```

十字キーに応じてクラウド上の turtlesim が動作することをご確認ください．

ターミナル３とターミナル４ではクラウド上の `turtlesim` のトピックを購読してみます．

```
source /opt/ros/rolling/setup.bash
ros2 topic echo /turtle1/pose
```

```
source /opt/ros/rolling/setup.bash
ros2 topic echo /turtle1/color_sensor
```

前者は値が購読できますが後者はできません．
これはJSONファイルで指定された期待通りの動作です:D

## デモ２：rmw_zenoh（on p.11）

[rmw_zenoh](https://github.com/ros2/rmw_zenoh) を使ってみましょう！

このデモではローカルのみを使用します．ターミナルは５つ立ち上げます．

### gossip scoutingによる接続

まずはデフォルトの接続方式であるgossip scoutingの動作を確認します．

ターミナル１でZenohルータを立ち上げます．

```
source ~/ws_rmw_zenoh/install/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

ターミナル２では出版ノードを実行します．

```
source /opt/ros/rolling/setup.bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run demo_nodes_cpp talker
```

ターミナル３では購読ノードを実行します．

```
source /opt/ros/rolling/setup.bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run demo_nodes_cpp listener
```

無事に通信できているでしょうか？

### multicast scoutingによる接続

次にmulticast scoutingで接続してみます．

`ZENOH_ROUTER_CHECK_ATTEMPTS=-1` としてZenohルータの探索を行わないようにして，セッション向けにカスタマイズしたJSONファイルを用います．
デフォルトのファイルからの変更は [こちら](https://github.com/takasehideki/rosconjp24_demo/commit/b15aba6b49b0f6003903c3207178d00ac2bed8ed) を参照ください．

なお，せっかくですのでここではターミナル１のZenohルータは `Ctrl+C` で終了させておきましょう．

ターミナル４では出版ノードをmulticast scoutingで実行します．

```
source /opt/ros/rolling/setup.bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp ZENOH_ROUTER_CHECK_ATTEMPTS=-1 ZENOH_SESSION_CONFIG_URI=./SESSION_MULTICAST.json5 ros2 run demo_nodes_cpp talker
```

ターミナル５では購読ノードを実行します．

```
source /opt/ros/rolling/setup.bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp ZENOH_ROUTER_CHECK_ATTEMPTS=-1 ZENOH_SESSION_CONFIG_URI=./SESSION_MULTICAST.json5 ros2 run demo_nodes_cpp listener
```

Zenohルータ無しでも無事に通信できているでしょうか？

## デモ３：rmw_zenoh from device to the cloud（on p.12）

最後に，rmw_zenohでNATを越えましょう！

クラウド側では２つ，ローカル側では２つのターミナルを立ち上げます．

### クラウド側

ターミナル１では通常通りにZenohルータを立ち上げます．

```
source /opt/ros/rolling/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

ターミナル２では rmw_zenoh で `turtlesim` を起動しておきます．

```
source /opt/ros/rolling/setup.bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run turtlesim turtlesim_node
```

### ローカル側

ターミナル１ではZenohルータを立ち上げます．
ここで，ルータ向けにカスタマイズしたJSONファイルを用います．
デフォルトのファイルからの変更は [こちら](https://github.com/takasehideki/rosconjp24_demo/commit/acfe72fd9c7ef05489da5a729bc50fa736e1add2) を参照ください．
本ファイル中の `10.20.30.40` は例なので，ご自身のクラウドのIPアドレスを記述してください．

```
source /opt/ros/rolling/setup.bash
ZENOH_ROUTER_CONFIG_URI=./ROUTER_MULTIHOST.json5 ros2 run rmw_zenoh_cpp rmw_zenohd
```

ターミナル２では rmw_zenoh で `turtle_teleop_key` を起動しておきます．

```
source /opt/ros/rolling/setup.bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run turtlesim turtle_teleop_key
```

Zenoh & rmw_zenoh すぎょいっ！！
