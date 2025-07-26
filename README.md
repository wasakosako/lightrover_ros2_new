# lightrover_ros2

このリポジトリは、Vstone社製ロボット「LightRover」をROS2で操作するためのサンプルパッケージです。

詳細なドキュメントは[LightRover WebDoc](https://vstoneofficial.github.io/lightrover_webdoc/)をご参照ください。

## 概要

`lightrover_ros2`は、LightRoverのシミュレーション、ナビゲーション、および基本的な制御をROS2環境で行うためのサンプルコードを提供します。
このパッケージ群を利用することで、LightRoverのモデルをGazeboシミュレータ上で動かしたり、SLAM（自己位置推定と地図作成）やナビゲーションを実行したり、ゲームパッドで操作したりすることが可能です。

## パッケージ構成

このワークスペースは、以下のROS2パッケージで構成されています。詳細については各パッケージのリンク先をご参照ください。

| パッケージ名 | 説明 |
| :--- | :--- |
| [`lightrover_description`](./lightrover_description/readme.md) | ロボットの3Dモデル（URDF）と、シミュレーション・可視化（RViz）のための設定ファイルが含まれています。 |
| `lightrover_interface` | カスタムのサービスやメッセージの定義が含まれています。 |
| [`lightrover_navigation`](./lightrover_navigation/readme.md) | SLAM（`slam_toolbox`）やナビゲーション（`Nav2`）を実行するための設定ファイルやLaunchファイルが含まれています。 |
| [`lightrover_ros`](./lightrover_ros/readme.md) | Pythonで書かれたメインのROSノード群です。オドメトリの計算、ゲームパッドによる操作、I2C通信を介したモーター制御などを行います。 |

## セットアップとビルド

1.  **依存関係のインストール**
    ROS2 Foxy Fitzroyがインストールされていることを確認してください。また、以下の追加パッケージが必要です。

    ```bash
    sudo apt-get update
    sudo apt-get install ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui ros-foxy-xacro ros-foxy-slam-toolbox ros-foxy-nav2-bringup
    ```

2.  **ワークスペースのビルド**
    ワークスペースのルートディレクトリで、`colcon`を使ってビルドを実行します。

    ```bash
    colcon build
    ```

3.  **環境設定の読み込み**
    ビルドが完了したら、以下のコマンドで環境設定を読み込みます。

    ```bash
    source install/setup.bash
    ```

## 実行方法 (Launchファイル)

このパッケージには、特定のタスクを実行するためのLaunchファイルがいくつか用意されています。

-   **モデル表示**:
    RViz上でLightRoverの3Dモデルを表示します。
    ```bash
    ros2 launch lightrover_description lightrover_view.launch.py
    ```

-   **SLAM (GMapping)**:
    `slam_toolbox`を使用して、環境地図の作成と自己位置推定を同時に行います。
    ```bash
    ros2 launch lightrover_ros lightrover_slam.launch.py
    ```

-   **ナビゲーション**:
    `Nav2`スタックを使用して、作成済みの地図上で自律走行を行います。
    ```bash
    ros2 launch lightrover_navigation bringup_launch.py
    ```

-   **ゲームパッド操作**:
    ゲームパッド（Joy-Conなど）でLightRoverを操作します。
    ```bash
    ros2 launch lightrover_ros pos_joycon.launch.py
    ```

## ライセンス

このプロジェクトの主要なパッケージは、Apache License 2.0 の下で公開されています。詳細については、各パッケージの`package.xml`ファイルをご参照ください。