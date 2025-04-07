# kachaka_gazebo

## はじめに

kachaka_gazeboパッケージは、Gazebo Ignitionによるカチャカのシミュレーション環境を提供します。

## Requirements

以下の環境で動作確認しました。

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

## 使い方

基本的な起動コマンドは以下の通りです。

```bash
ros2 launch kachaka_gazebo simulation.launch.py
```

Gazebo IgnitionでGUIが不要な場合、`headless:=True`を指定することでヘッドレスモードで起動できます。

```bash
ros2 launch kachaka_gazebo simulation.launch.py headless:=True
```

また、worldファイルを指定することもできます。同梱のワールドファイル（`sample_world.sdf`）を用いる場合の実行例を以下に示します。

```bash
ros2 launch kachaka_gazebo simulation.launch.py world:=$HOME/dev_ws/src/kachaka_ros2_dev_kit/kachaka_gazebo/worlds/sample_world.sdf
```

## パラメータ

`simulation.launch.py`のパラメータを下表に示します。

|パラメータ|意味|デフォルト値|
|---|---|---|
|namespace|ネームスペース||
|use_sim_time|simulation/GazeboのClockを用いるか否か。|True|
|use_robot_state_pub|robot_state_publisherを用いるか否か。|True|
|headless|ヘッドレスモードで起動するか否か。|False|
|world|ワールドファイル|`kachaka_gazebo/worlds/depot.sdf`|
|x_pose|ロボットをspawnする座標（x）、単位はメートル|0.00|
|y_pose|ロボットをspawnする座標（y）、単位はメートル|0.00|
|z_pose|ロボットをspawnする座標（z）、単位はメートル|0.01|
|roll|ロボットをspawnする姿勢（roll）、単位はラジアン|0.00|
|pitch|ロボットをspawnする姿勢（pitch）、単位はラジアン|0.00|
|yaw|ロボットをspawnする姿勢（yaw）、単位はラジアン|0.00|

> [!CAUTION]
> `namespace`は現時点で機能しません。今後、対応予定です。

## サポートマトリクス

カチャカROS 2ブリッジで対応するトピックに対してkachaka_gazeboの対応状況を下表に示します。

|トピック名|kachaka_gazebo対応状況|備考|
|---|---|---|
|/kachaka/back_camera/camera_info|&#9989;||
|/kachaka/back_camera/image_raw|&#9989;||
|/kachaka/back_camera/image_raw/camera_info|&#10060;||
|/kachaka/back_camera/image_raw/compressed|&#9989;|`ros_gz_image`パッケージの`image_bridge`によりサポート|
|/kachaka/front_camera/camera_info|&#9989;||
|/kachaka/front_camera/image_raw|&#9989;||
|/kachaka/front_camera/image_raw/camera_info|&#10060;||
|/kachaka/front_camera/image_raw/compressed|&#9989;|`ros_gz_image`パッケージの`image_bridge`によりサポート|
|/kachaka/imu/imu|&#9989;||
|/kachaka/joint_states|&#9989;||
|/kachaka/layout/locations/list|&#10060;||
|/kachaka/layout/shelves/list|&#10060;||
|/kachaka/lidar/scan|&#9989;||
|/kachaka/manual_control/cmd_vel|&#9989;||
|/kachaka/mapping/map|&#9989;|`nav2_map_server`パッケージの`map_server`によりサポート|
|/kachaka/object_detection/result|&#10060;||
|/kachaka/odometry/odometry|&#10060;||
|/kachaka/robot_description|&#9989;||
|/kachaka/robot_info/battery_state|&#10060;||
|/kachaka/robot_info/version|&#10060;||
|/kachaka/tof_camera/camera_info|&#9989;||
|/kachaka/tof_camera/image_raw|&#9989;||
|/kachaka/tof_camera/image_raw/camera_info|&#10060;||
|/kachaka/tof_camera/image_raw/compressedDepth|&#9989;|`ros_gz_image`パッケージの`image_bridge`によりサポート|
|/kachaka/torch/back|&#10060;||
|/kachaka/torch/front|&#10060;||
|/kachaka/wheel_odometry/wheel_odometry|&#9989;||

## 注意点

### オドメトリ

公式のカチャカROS 2ブリッジと`kachaka_gazebo`パッケージが提供するGazeboでは以下の差異があります。

- カチャカROS 2ブリッジはフュージョンされたオドメトリをもとにbase_link、odom間のtfを発行します
- `kachaka_gazebo`パッケージが提供するGazebo環境は、ホイールオドメトリをもとにbase_link、odom間のtfを発行します

### 2D LiDAR設置高さ

<https://github.com/gazebosim/gz-sensors/issues/509>の問題を回避するため、[kachaka_description/urdf/_kachaka.urdf.xacro](../kachaka_description/urdf/_kachaka.urdf.xacro)に対して以下の変更を行っています。

```diff
-              xyz="0.156 0 0.1049" />
+              xyz="0.156 0 0.1549" />
```

### ToFカメラの設置位置

Gazebo Ignitionにおけるdepthカメラの問題を回避するため、[kachaka_description/urdf/_kachaka.urdf.xacro](../kachaka_description/urdf/_kachaka.urdf.xacro)に対して以下の変更を行っています。

```diff
-              xyz="0.221 0.0 0.0418" />
+              xyz="0.225 0.0 0.0418" />
```

## ライセンス

Apache License, Version 2.0
