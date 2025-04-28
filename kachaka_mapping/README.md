# kachaka_mapping

## はじめに

kachaka_mappingパッケージは、Mappingを起動するためのパッケージです。

## Usage

基本的な起動コマンドは以下の通りです。

```bash
ros2 launch kachaka_mapping mapping_launch.py
```

## パラメータ

`mapping_launch.py`のパラメータを下表に示します。

|パラメータ|意味|デフォルト値|
|---|---|---|
|use_sim_time|simulation/GazeboのClockを用いるか否か。|false|
|slam_params_file|Slam Toolboxのパラメータファイル|`kachaka_mapping/params/mapper_params_online_sync.yaml`|

## ライセンス

Apache License, Version 2.0
