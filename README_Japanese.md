# Arcanain 画像処理チュートリアル

## 説明
このパッケージは、画像のリサイズ、グレースケールへの変換、明度とコントラストの調整、画像の回転など、様々な画像処理タスクを行うための ROS 2 ノードを提供します。各ノードは画像トピックを購読し、画像を処理して別のトピックに結果を公開します。

## セットアップ

### 依存関係
- ROS 2（Foxy 以上を推奨）
- OpenCV 4.x
- cv_bridge
- image_transport

### パッケージのビルド
1. リポジトリを ROS 2 ワークスペース（例: `~/dev_ws/src/`）にクローンします。
2. ワークスペースのルートに移動します（例: `~/dev_ws`）。
3. 次のコマンドを使用してワークスペースをビルドします：
   ```
   colcon build --packages-select arcanain_imageprocessing_tutorial
   ```
4. セットアップファイルをソースにして、ワークスペースを ROS 2 環境に含めます：
   ```
   source install/setup.bash
   ```

## ノードの説明と使用方法

### ランダム画像パブリッシャー
このノードはランダムな画像を生成して公開します。
- **公開トピック**: `/random_image` (sensor_msgs/Image)
- **実行コマンド**:
  ```
  ros2 run arcanain_imageprocessing_tutorial random_image_publisher
  ```

### 画像サブスクライバー（サブビュー）
このノードは画像トピックを購読して、OpenCVを使用して画像を表示します。
- **購読トピック**: `/processed_image` (sensor_msgs/Image)
- **実行コマンド**:
  ```
  ros2 run arcanain_imageprocessing_tutorial image_subscriber
  ```

### 画像リサイズ パブリッシャー-サブスクライバー
このノードは画像のサイズを元の半分にリサイズします。
- **購読トピック**: `/image` (sensor_msgs/Image)
- **公開トピック**: `/resized_image` (sensor_msgs/Image)
- **実行コマンド**:
  ```
  ros2 run arcanain_imageprocessing_tutorial image_resize_pub_sub
  ```

### 画像グレースケール パブリッシャー-サブスクライバー
カラー画像をグレースケールに変換します。
- **購読トピック**: `/image` (sensor_msgs/Image)
- **公開トピック**: `/grayscale_image` (sensor_msgs/Image)
- **実行コマンド**:
  ```
  ros2 run arcanain_imageprocessing_tutorial image_grayscale_pub_sub
  ```

### 画像明度調整 パブリッシャー-サブスクライバー
画像の明度を調整します。
- **購読トピック**: `/image` (sensor_msgs/Image)
- **公開トピック**: `/brightness_adjusted_image` (sensor_msgs/Image)
- **実行コマンド**:
  ```
  ros2 run arcanain_imageprocessing_tutorial image_adjust_brightness_pub_sub
  ```

### 画像コントラスト調整 パブリッシャー-サブスクライバー
画像のコントラストを調整します。
- **購読トピック**: `/image` (sensor_msgs/Image)
- **公開トピック**: `/contrast_adjusted_image` (sensor_msgs/Image)
- **実行コマンド**:
  ```
  ros2 run arcanain_imageprocessing_tutorial image_adjust_contrast_pub_sub
  ```

### 画像回転 パブリッシャー-サブスクライバー
画像を90度回転させます。
- **購読トピック**: `/image` (sensor_msgs/Image)
- **公開トピック**: `/rotated_image` (sensor_msgs/Image)
- **実行コマンド**:
  ```
  ros2 run arcanain_imageprocessing_tutorial image_rotate_pub_sub
  ```

## ノードの実行方法
上記のいずれかのノードを実行するには、新しいターミナルを開き、ROS 2 環境をソースにして、各ノードに対応するコマンドを使用してください。トピック名が特定の設定に合っていることを確認するか、必要に応じてソースコード内のトピック名を変更してください。

