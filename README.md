# mini6dof_arm_project

このプロジェクトは小型で高機能な6軸のロボットアームのプラットフォームをROSで構築することを目標にした**個人の趣味プロジェクト**である。
このプロジェクトでは以下のタスクを達成することを目標にしている
- 電流監視ベースの外力推定
- 電流制御ベースのインピーダンス制御
- yoloなどの画像認識モジュールを用いた開発
- 深層予測学習ベースの物体のピッキングタスクの実現

[進捗を上げるtwitter](https://twitter.com/siodakaram6dfam)

# Progress

ロボットの動作生成をposition_controllerで制御するためのプラットフォームの構築が完成しました。
動作生成は以下の動画のような種類が現在用意できています。これらのプラットフォームはすべてROS1で作っています(順次ROS2に移行予定)

## demo mode

https://user-images.githubusercontent.com/107166405/200532276-9d09ea74-67c4-446f-ba8a-c22c28f4ddd0.mp4

## RCM mode

https://user-images.githubusercontent.com/107166405/200532558-7f651eaf-b58d-436d-8fd3-f8c7f01e9ea6.mp4

## YOLOv5 mode

https://user-images.githubusercontent.com/107166405/200532687-34732260-e526-4a11-95f8-509fd4c7b663.mp4


## PID gain tuning mode

[Dynamixel_PID_gain.webm](https://user-images.githubusercontent.com/107166405/200537556-3db1a773-beba-4350-b65a-5a4e5d36be3e.webm)

# Next Task

今後は、ロボットアームの**インピーダンス制御や外力推定**に向けて使用しているDynamixelサーボのトルク測定を行います。
ここでは二つのモーター特性評価を行います。

## 停動トルク測定

![Torque_measurement_unit](https://user-images.githubusercontent.com/107166405/200535368-3e2929aa-be9f-476b-b4e7-e44b864824dd.png)

このトルク測定装置はDynamixelシリーズの停動トルクをloadcellで測定するための装置である。

## 動的トルク測定

Dynamixelは高減速比アクチュエータであるために回転数によってモータ特性が変化すると考えられるために、パウダークラッチをブラシレスモータに代替した安価な動的トルク測定装置を製作する。

ここにおいてBrushlessモーターは小型なdirectdriveモーターを採用するためにdynamixelに対してトルクで劣るため、停動トルク測定装置で得られたデータを融合することで幅広いトルク、回転域でのモーターの特性評価を実現する。






