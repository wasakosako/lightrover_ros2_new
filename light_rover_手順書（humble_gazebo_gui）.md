以下は ROS 2 Humble + Gazebo classic 11 前提の“最短で再現できる”手順書です。

---

# ① コンテナを立てる時から Gazebo GUI でロボットを動かすまで

## A.（推奨）Docker で GUI 有効のコンテナを起動（Linux/X11）

> NVIDIA GPU が無いなら `--gpus all` は省略可。Wayland / リモートは X11 転送の準備が必要。

```bash
# ホスト側
xhost +local:root

docker run --rm -it \
  --name lightrover \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device=/dev/dri \
  --gpus all \
  -v ~/ros2_humble_ws:/workspaces/ros2_humble_ws \
  osrf/ros:humble-desktop-full
```

## B. ワークスペース準備（コンテナ内）

```bash
apt-get update
apt-get install -y git python3-colcon-common-extensions \
  ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro rviz2 mesa-utils

mkdir -p /workspaces/ros2_humble_ws/src
cd /workspaces/ros2_humble_ws/src

# ここでプロジェクトを配置（zip 展開または git clone 等）
# 例: unzip /path/to/lightrover_ros2_new-main.zip -d .

cd /workspaces/ros2_humble_ws
rosdep update
rosdep install -y --from-paths src --ignore-src

colcon build --symlink-install

echo 'source /workspaces/ros2_humble_ws/install/setup.bash' >> ~/.bashrc
source install/setup.bash
```

## C. Gazebo + RViz を起動（GUI あり）

```bash
# 前回の Gazebo が残っていたら掃除（任意）
pkill -9 gzserver gzclient 2>/dev/null || true
rm -f ~/.gazebo/master* 2>/dev/null || true

# 起動
ros2 launch lightrover_description lightrover_gazebo.launch.py \
  use_gui:=true use_rviz:=true
```

**OK の目印**：

- `SpawnEntity: Successfully spawned entity [lightrover]`
- `Configured and activated diff_drive_controller` と `joint_state_broadcaster`

## D. 走行コマンドを送る

> 本プロジェクトは **非スタンプ Twist** を購読する設定（`use_stamped_vel: false`）。

```bash
# 新しい端末で
source /workspaces/ros2_humble_ws/install/setup.bash

# 直進（前進）
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 5

# その場旋回（左）
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 5

# 停止（ゼロ速度を一回送る）
ros2 topic pub -1 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

> `/cmd_vel` を使いたい場合は YAML 側で `use_stamped_vel: true` に変更し、送信先を `/diff_drive_controller/cmd_vel`（`geometry_msgs/TwistStamped`）にします。

## E. 動作チェック（必須の健康診断）

```bash
# コントローラ状態
ros2 control list_controllers
# 例）
# joint_state_broadcaster ... active
# diff_drive_controller  ... active

# JointState が出ているか
ros2 topic echo -n 1 /joint_states

# TF が流れているか
ros2 run tf2_ros tf2_echo base_footprint base_link

# Lidar などセンサ（定義している場合）
ros2 topic list | grep -E '/scan|/points'
```

> もし `diff_drive_controller` が active にならない場合：

```bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner diff_drive_controller
```

---

# ② プロジェクトの全体像（最低限使うところ）

```
lightrover_description/
├─ urdf/
│   ├─ lightrover_urdf.xacro       # ロボット本体（ベース/車輪/キャスタ/センサ）
│   └─ ...                          # サブ xacro, Gazebo 用 plugin 設定
├─ config/
│   └─ lightrover_controllers.yaml  # ros2_control + diff_drive の設定
├─ worlds/
│   └─ lightrover_sample.world      # Gazebo の world（地面・物理設定）
├─ rviz/
│   └─ lightrover.rviz              # RViz の表示プリセット
└─ launch/
    └─ lightrover_gazebo.launch.py  # gzserver/gzclient/robot_state_pub/スポーン/コントローラ
```

> Gazebo 上で“動かすだけ”なら、この `lightrover_description` が中心。他パッケージ（`lightrover_interface`, `lightrover_navigation`, `ydlidar_sdk`, `lightrover_ros`）は実機や拡張時に使用。

---

# ③ ロボットが動く仕組み（信号の行き先と行く末）

```
[利用者ノード / ros2 topic pub / teleop 等]
           │ (geometry_msgs/Twist)
           ▼
 /diff_drive_controller/cmd_vel_unstamped
           │  subscribe
           ▼
[diff_drive_controller] (ros2_controllers)
  ├─ 左右車輪の速度指令（velocity interface）
  └─ 必要に応じて odom を発行
           │
           ▼
[gazebo_ros2_control + GazeboSystem]
  ├─ 速度指令 → Gazebo のジョイントへ適用
  └─ ジョイント状態を取得
           │ (state)
           ▼
[joint_state_broadcaster] → /joint_states
           │
           │ /joint_states + URDF
           ▼
[robot_state_publisher] → /tf, /tf_static → (RViz の RobotModel 表示)

（センサ系）
[Gazebo LiDAR plugin] → /scan（LaserScan）や /points（PointCloud2）

（管理）
[controller_manager]
  ├─ spawner joint_state_broadcaster
  └─ spawner diff_drive_controller
```

---

## 付記：トラブル時のメモ

- Gazebo が落ちる／繋がらない：`pkill -9 gzserver gzclient` → `rm -f ~/.gazebo/master*` でリセット。
- FPS が極端に低い：GUI を閉じる（`use_gui:=false use_rviz:=false`）、物理ステップやセンサ周期の負荷を下げる、GPU を使う、ヘッドレスで `gzserver` のみ起動。
- 走らない：`ros2 control list_controllers` で **active** を確認。送信先トピックが `/diff_drive_controller/cmd_vel_unstamped` か再確認。

