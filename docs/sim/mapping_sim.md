# 地図生成

ここではシミュレータ環境を用いて地図生成を行う手順を説明します。

## シミュレータ起動

以下のコマンドを実行してGazebo Ignitionを起動します。

```bash
ros2 launch kachaka_gazebo simulation.launch.py
```

Gazebo IgnitionでGUIが不要な場合、`headless:=True`を指定することでヘッドレスモードで起動できます。

```bash
ros2 launch kachaka_gazebo simulation.launch.py headless:=True
```

## Slam Toolbox起動

以下のコマンドを実行してSlam Toolboxを起動します。シミュレータ環境を用いるため、`use_sim_time:=true`としています。

```bash
ros2 launch kachaka_mapping mapping_launch.py use_sim_time:=true
```

## キー操作による手動走行

以下のコマンドを実行してキー操作による手動走行を行います。

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/kachaka/manual_control/cmd_vel
```

操作キーは以下の通りです。

```plain
Moving around:
   u    i    o
   j    k    l
   m    ,    .
```

## 地図保存

地図が一通りできたら、以下のコマンドを実行して地図の保存を行います。`map_name`となっている箇所は保存したい地図名に置き換えて実行してください。

```bash
ros2 run nav2_map_server map_saver_cli -f map_name
```
