# Autoware AIチャレンジ2025 モジュール分析・改善計画

## プロジェクト現状分析

### 実行環境
- **作業ディレクトリ**: `/home/ishimitsu/Works/AutomotiveAIChallenge/aichallenge-2025`
- **メインlaunchファイル**: `reference.launch.xml`
- **目標**: 1周時間短縮によるパフォーマンス向上

## 現在のシステム構成と問題点

### 🎯 最大のボトルネック: Simple Trajectory Generator
**現在の設定**:
```xml
<param name="csv_path" value="raceline_awsim_30km.csv"/>  <!-- 15km → 30kmに変更済み -->
```

**問題**:
- 固定速度プロファイル（30km/h一律）
- カーブの曲率を考慮しない速度設定
- → 30km/h化により衝突・早曲がり問題が発生

### 🔧 実施済み調整: Simple Pure Pursuit
**変更したパラメータ**:
```xml
<param name="lookahead_gain" value="0.8"/>              <!-- 0.5 → 0.8 -->
<param name="lookahead_min_distance" value="5.0"/>      <!-- 3.5 → 5.0 -->
<param name="speed_proportional_gain" value="1.5"/>     <!-- 1.0 → 1.5 -->
```

**結果**:
- ✅ 衝突は回避されたが
- ❌ カーブを曲がるのが早すぎる問題が発生
- → パラメータチューニングでは根本解決困難

## 既存モジュール詳細分析

### 速度・センサー関連モジュール
1. **Vehicle Velocity Converter**
   - 機能: 車両速度 → TwistWithCovariance変換
   - 設定: `speed_scale_factor: 1.0`

2. **IMU Corrector**
   - 機能: IMUデータ補正・フレーム変換
   - 角速度標準偏差: 0.03 rad/s

3. **Racing Kart GNSS Poser**
   - 機能: GNSS位置情報からPose生成
   - メディアンフィルタリング実装

### 測位関連モジュール
4. **Gyro Odometer**
   - 機能: 車両速度とIMU角速度統合
   - 時間平均化による共分散計算

5. **EKF Localizer**
   - 機能: GNSS/IMU/車輪速度のカルマンフィルタ融合
   - 設定: `pose_additional_delay: 0.3秒`, TF更新30Hz

6. **My GNSS Poser** ⭐カスタムモジュール
   - 機能: 高度なGNSS-IMU融合測位
   - 外れ値検出、動的共分散、品質評価機能

### 制御関連モジュール
7. **Simple Pure Pursuit**
   - 現在の問題点: 単純な先読み距離計算、カーブ曲率非考慮

8. **Simple Trajectory Generator**
   - 現在の問題点: 固定速度プロファイル、カーブ最適化なし

## autoware_practice_* モジュール調査結果

### 発見されたディレクトリ構造
```
autoware_practice_course/
├── config/ (空)
├── src/
│   ├── avoidance/ (空)
│   ├── vehicle/ (空)
│   └── velocity_planning/scripts/ (空)

autoware_practice_driver/launch/ (空)
autoware_practice_evaluator/ (空)
autoware_practice_gyro_odometer/ (空)
autoware_practice_launch/ (空)
autoware_practice_lidar_simulator/ (空)
autoware_practice_msgs/ (空)
autoware_practice_problems/data/ (空)
autoware_practice_simulator/ (空)
autoware_practice_visualization/ (空)
autoware_practice_dummy_localizer/ (空)
```

### 重要な発見
- **すべてのautoware_practice_*ディレクトリは空**
- 実装ファイル（.cpp, .hpp, .py, .xml）は存在しない
- ただし、buildログには`autoware_practice_gnss_poser`の痕跡あり
- → 過去に実装されていた可能性があるが、現在はソースファイル不在

## 問題の根本原因と解決戦略

### 🔥 緊急の問題
1. **カーブ早曲がり**: 先読み距離が長すぎてコーナー手前で操舵開始
2. **速度プロファイル不適切**: 直線もカーブも同じ30km/hで走行

### 🎯 根本的解決アプローチ
**最優先: カーブ適応型軌道生成器の開発**

```cpp
// 目指す改善内容
struct OptimalSpeedProfile {
    double straight_speed = 35.0;     // 直線: 35km/h
    double gentle_curve_speed = 25.0;  // 緩カーブ: 25km/h  
    double sharp_curve_speed = 15.0;   // 急カーブ: 15km/h
    double transition_distance = 20.0; // 遷移区間: 20m
};
```

### 📋 開発優先順位
| 順位 | モジュール | 期待効果 | 開発難易度 | 実装場所 |
|------|-----------|----------|------------|----------|
| 🥇 | **カーブ適応型軌道生成器** | ⭐⭐⭐⭐⭐ | ⭐⭐ | autoware_practice_course |
| 🥈 | **改良Pure Pursuit** | ⭐⭐⭐⭐ | ⭐⭐⭐ | 新規作成 |
| 🥉 | **測位精度向上** | ⭐⭐⭐ | ⭐⭐⭐⭐ | my_gnss_poser改良 |

## 次のアクションプラン

### Phase 1: 軌道生成器開発 (最優先)
1. **CSVデータ解析**: 既存のraceline_awsim_30km.csvの軌道点・曲率分析
2. **曲率計算**: 各軌道点での曲率半径算出
3. **速度プロファイル設計**: カーブ毎の最適速度決定
4. **autoware_practice_course実装**: 新しい軌道生成器作成
5. **launch.xml更新**: simple_trajectory_generatorから置き換え

### Phase 2: 制御系改良
1. **Pure Pursuit改良**: 曲率適応先読み距離
2. **測位精度向上**: my_gnss_poser最適化

### Phase 3: パフォーマンステスト
1. **シミュレーション検証**: 各改善の効果測定
2. **ラップタイム計測**: 改善前後比較
3. **安全性確認**: 衝突・逸脱がないことを確認

## 技術的考慮事項

### 軌道生成器の実装要件
```cpp
class AdaptiveTrajectorygenerator {
    // 曲率に基づく速度計算
    double calculateOptimalSpeed(double curvature, double banking = 0.0);
    
    // 速度遷移の滑らかな補間  
    std::vector<TrajectoryPoint> smoothSpeedTransition(
        const std::vector<PathPoint>& path,
        const SpeedProfile& profile
    );
    
    // ROS2ノードとしての実装
    void publishTrajectory();
};
```

### launch.xml統合
```xml
<!-- 置き換え予定 -->
<!-- FROM: simple_trajectory_generator -->
<!-- TO: autoware_practice_adaptive_trajectory_generator -->
<node pkg="autoware_practice_course" 
      exec="adaptive_trajectory_generator_node" 
      name="adaptive_trajectory_generator">
  <param name="csv_path" value="raceline_awsim_30km.csv"/>
  <param name="max_speed" value="35.0"/>
  <param name="min_curve_speed" value="15.0"/>
  <param name="curvature_threshold" value="0.1"/>
</node>
```

## 現在の作業状況
- ✅ モジュール分析完了
- ✅ ボトルネック特定完了  
- ✅ autoware_practice調査完了
- ⏳ **次のステップ**: カーブ適応型軌道生成器の実装開始

## 重要な設定ファイル
- **メインlaunch**: `/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/reference.launch.xml`
- **軌道データ**: `/aichallenge/workspace/src/aichallenge_submit/simple_trajectory_generator/data/raceline_awsim_30km.csv`
- **実装予定**: `/aichallenge/workspace/src/aichallenge_submit/autoware_practice_course/`

---
*最終更新: 2025-08-20*
*状態: カーブ適応型軌道生成器実装準備完了*
