
# ナビゲーション

ここではカチャカ実機を用いてナビゲーションを行う手順を説明します。

## ROS 2ブリッジ起動

<https://github.com/pf-robotics/kachaka-api/blob/main/docs/ROS2.md>を参考にしてROS 2ブリッジを起動します。

```bash
git clone https://github.com/pf-robotics/kachaka-api.git
cd ~/kachaka-api/tools/ros2_bridge
./start_bridge.sh <カチャカのIPアドレス>
```

## ナビゲーション起動

以下のコマンドを実行してナビゲーションを起動します。カチャカ実機を用いるため、`use_sim_time:=false`としています。

```bash
ros2 launch kachaka_nav2_bringup navigation_launch.py use_sim_time:=false
```
