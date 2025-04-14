# kachaka_nav2_bringup

## はじめに

kachaka_nav2_bringupパッケージは、Nav2 stackを起動するためのパッケージです。

## 自己位置推定

### Usage

基本的な起動コマンドは以下の通りです。

```bash
ros2 launch kachaka_nav2_bringup localization_launch.py map:=<map_yaml_path>
```

### パラメータ

`localization_launch.py`のパラメータを下表に示します。

|パラメータ|意味|デフォルト値|
|---|---|---|
|namespace|ネームスペース||
|map|地図のYAMLファイル||
|use_sim_time|simulation/GazeboのClockを用いるか否か。|false|
|params_file|nav2_amclのパラメータファイル|`kachaka_nav2_bringup/params/localization_param.yaml`|
|use_respawn|ノードがクラッシュしたときに再度立ち上げるか否か。|False|
|log_level|ログレベル|info|

> [!CAUTION]
> `namespace`は現時点で機能しません。今後、対応予定です。

## ナビゲーション

### Usage

基本的な起動コマンドは以下の通りです。

```bash
ros2 launch kachaka_nav2_bringup navigation_launch.py
```

### パラメータ

`navigation_launch.py`のパラメータを下表に示します。

|パラメータ|意味|デフォルト値|
|---|---|---|
|namespace|ネームスペース||
|use_sim_time|simulation/GazeboのClockを用いるか否か。|false|
|autostart|Nav2 Stackが自動起動するかどうか。|true|
|params_file|Nav2 Stackのパラメータファイル|`kachaka_nav2_bringup/params/nav2_params.yaml`|
|use_respawn|ノードがクラッシュしたときに再度立ち上げるか否か。|False|
|log_level|ログレベル|info|

> [!CAUTION]
> `namespace`は現時点で機能しません。今後、対応予定です。

## ライセンス

Apache License, Version 2.0
