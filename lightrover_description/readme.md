# `lightrover_description` パッケージ

このパッケージは、LightRoverの3Dモデル（URDF）と、シミュレーションおよび可視化（RViz）のための設定ファイルを提供します。

## 主要なファイル

-   `urdf/lightrover_urdf.xacro`: LightRoverの構造（リンクやジョイント）、センサー、物理的な特性を定義するメインのxacroファイルです。このファイルから他のxacroファイルがインクルードされます。
-   `launch/lightrover_view.launch.py`: RVizを起動し、`robot_state_publisher`と`joint_state_publisher_gui`を使ってロボットモデルを表示するためのLaunchファイルです。これにより、ロボットの現在の状態を3Dで視覚的に確認できます。
-   `rviz/lightrover.rviz`: RVizの表示設定ファイルです。モデルの表示方法、TF（座標変換）フレーム、センサーデータなどをどのように表示するかを定義します。
-   `meshes/`: ロボットモデルを構成する各パーツの3Dメッシュファイル（`.dae`形式）が格納されています。

## 動作の仕組み

`lightrover_view.launch.py`を実行すると、以下のノードが起動します。

1.  **`robot_state_publisher`**: `robot_description`パラメータ（`lightrover_urdf.xacro`から生成）を読み込み、ロボットの各リンク間の静的なTF（座標変換）情報をブロードキャストします。
2.  **`joint_state_publisher_gui`**: GUIスライダーを介して各ジョイントの角度を手動で設定し、`/joint_states`トピックにパブリッシュします。これにより、RViz上でロボットの関節を動かすことができます。
3.  **`rviz2`**: 設定ファイル（`lightrover.rviz`）に基づいて起動し、`robot_state_publisher`からのTF情報と`/joint_states`トピックを購読して、ロボットモデルを3D空間に表示します。