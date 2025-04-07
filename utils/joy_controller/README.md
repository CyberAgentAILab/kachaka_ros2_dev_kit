# joy_controller

## はじめに

joy_controllerは、ジョイスティックの入力を受け取り、車両をマニュアル走行させるためのTwistメッセージを発行します。

## Subscribing Topics

None

## Publishing Topics

| Topic name      | Type              | Description                                                  |
| --------------- | ----------------- | ------------------------------------------------------------ |
| /kachaka/manual_control/cmd_vel    | geometry_msgs/Twist | The velocity command to control the robot. |

## Node Parameters

### `/operation/joy_linux/joy_linux_node`

Edit the parameter in `joy_controller/config/joy_linux_param.yaml`.

| Parameter name               | Type   | Description                                                  |
| ---------------------------- | ------ | ------------------------------------------------------------ |
| dev | String | The device name of the joystick. default: /dev/input/js0 |

### `/operation/joy_controller/joy_controller_node`

Edit the parameters in `joy_controller/config/joy_controller_param.yaml`.

| Parameter name               | Type   | Description                                                  |
| ---------------------------- | ------ | ------------------------------------------------------------ |
| sub_topic.joy | String | The topic name of the joystick message. |
| pub_topic.cmd_vel | String | The topic name of the velocity command. |
| joy_top_left_button_idx | Int | The index of the top left button to slow down the operational speed. |
| joy_top_right_button_idx | Int | The index of the top right button to enable the operation. |
| joy_left_stick_x_idx | Int | The index of the left stick x-axis. |
| joy_left_stick_y_idx | Int | The index of the left stick y-axis. |
| joy_right_stick_x_idx | Int | The index of the right stick x-axis. |
| joy_right_stick_y_idx | Int | The index of the right stick y-axis. |
| joy_autonomous_mode_idx | Int | The index of the button to switch to autonomous mode. |
| joy_manual_mode_idx | Int | The index of the button to switch to manual mode. (default: manual mode)|
| abs_max_linear_vel_x | Double | The maximum linear velocity [m/s] in the x-axis (vehicle forward is positive). |
| abs_max_linear_vel_y | Double | The maximum linear velocity [m/s] in the y-axis (vehicle left is positive). |
| abs_max_angular_vel_z | Double | The maximum angular velocity [rad/s] in the z-axis, counter-clockwise is positive. |

## 使い方

ノードを個別に起動するには、次のコマンドを実行します。

```bash
ros2 launch joy_controller joy_controller_launch.py
```

## Contribution

This package is contributed by [MizuhoAOKI](https://github.com/MizuhoAOKI) and [shimo-nu](https://github.com/shimo-nu).
