# `lightrover_navigation` パッケージ

このパッケージは、ROS2のナビゲーションスタックである`Nav2`と、SLAMライブラリである`slam_toolbox`を使用して、LightRoverの自律移動を実現するための設定ファイルとLaunchファイルを提供します。

## 主要なファイル

-   `launch/bringup_launch.py`: `Nav2`の全機能を起動するためのメインのLaunchファイルです。SLAMモードとナビゲーションモードを切り替えることができます。
-   `launch/lightrover_navigation.launch.py`: `Nav2`のナビゲーション関連ノード（経路計画、障害物回避など）を起動します。
-   `launch/localization_launch.py`: `Nav2`の自己位置推定関連ノードを起動します。
-   `launch/online_async_launch.py`: `slam_toolbox`を非同期SLAMモードで起動します。
-   `config/nav2_params.yaml`: `Nav2`の各コンポーネント（コントローラー、プランナー、リカバリービヘイビアなど）のパラメータを設定します。
-   `maps/`: SLAMによって生成された地図（`.pgm`と`.yaml`）を保存するディレクトリです。

## 動作の仕組み

`bringup_launch.py`は、`Nav2`を起動するための包括的なスクリプトです。`slam`引数に応じて、以下のいずれかのモードで動作します。

-   **SLAMモード (`slam:=True`)**: `slam_toolbox`を起動し、LiDARセンサーからのデータを使って環境地図を作成しながら、同時にその地図上での自己位置を推定します。
-   **ナビゲーションモード (`slam:=False`)**: `map`引数で指定された既存の地図を読み込み、AMCL（Adaptive Monte Carlo Localization）を用いて自己位置を推定します。その後、`Nav2`のプランナーとコントローラーが目標地点までの経路を生成し、ロボットを自律的に走行させます。

このパッケージは、`lightrover_ros`パッケージが提供するオドメトリ情報（`/odom`）とセンサー情報（`/scan`）に依存しています。
