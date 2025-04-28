# kachaka_description

## はじめに

kachaka_descriptionパッケージは、カチャカのdescriptionを記述するパッケージです。

## Usage

基本的な起動コマンドは以下の通りです。

```bash
ros2 launch kachaka_description robot_display.launch.xml
```

## パラメータ

`robot_display.launch.xml`のパラメータを下表に示します。

|パラメータ|意味|デフォルト値|
|---|---|---|
|namespace|ネームスペース|`kachaka`|
|frame_prefix|frame_prefix|`/`|
|use_gui|joint_state_publisher_guiを使うか否か。|true|

## ライセンス

Apache License, Version 2.0
