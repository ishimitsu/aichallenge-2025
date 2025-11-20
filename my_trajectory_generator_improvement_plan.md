# my_trajectory_generator モジュール改善計画

## 概要

simple_trajectory_generatorを基にしたmy_trajectory_generatorモジュールの段階的改善計画。
自動運転AIチャレンジ2025のレース性能向上を目的とした機能拡張とパフォーマンス最適化を実施する。

## 現状分析

### 基本機能
- CSVファイルから軌道データを読み込み
- 固定周期(1秒)でTrajectoryメッセージを配信
- 動的パラメータ変更対応（csv_path、z座標）
- CSV形式: `x,y,z,x_quat,y_quat,z_quat,w_quat,speed` (8カラム)

### 主要課題
1. **パフォーマンス問題**: 1秒間隔配信、メモリ非効率
2. **機能制限**: 固定速度、軌道補間なし、エラーハンドリング不足
3. **レース特化不足**: 最適化機能なし、安全マージン考慮なし

## 改善計画

### Phase 1: 基本パフォーマンス向上 ⚡
**目標**: 制御精度向上とリアルタイム性能改善
**期間**: 1-2日
**変更行数目安**: ~50行

#### 1.1 配信頻度最適化
- **現状**: 1秒間隔 (1Hz)
- **改善**: 20Hz配信 (50ms間隔)
- **実装**:
  ```cpp
  timer_ = create_wall_timer(std::chrono::milliseconds(50), ...);
  ```

#### 1.2 メモリ効率化
- **現状**: 毎回全軌道データコピー配信
- **改善**: ヘッダーのみ更新、データ再利用
- **実装**:
  ```cpp
  // 初期化時に軌道データ作成、配信時はタイムスタンプのみ更新
  trajectory_.header.stamp = this->now();
  ```

#### 1.3 基本パラメータ拡張
- **追加パラメータ**:
  - `publish_rate`: 配信周波数 (Hz)
  - `speed_scale`: 速度スケール係数
  - `max_speed`: 最大速度制限 (m/s)

### Phase 2: 軌道品質向上 📈
**目標**: 軌道精度とスムージング機能追加
**期間**: 2-3日
**変更行数目安**: ~100行

#### 2.1 軌道補間機能
- **目的**: CSV点間の軌道密度向上
- **手法**: 線形補間またはスプライン補間
- **実装**:
  ```cpp
  std::vector<TrajectoryPoint> interpolateTrajectory(
    const std::vector<TrajectoryPoint>& raw_points,
    double resolution = 0.5  // 補間間隔(m)
  );
  ```

#### 2.2 動的速度制御
- **現状**: CSV固定速度値使用
- **改善**: パラメータベース動的調整
- **実装**:
  ```cpp
  double adjusted_speed = std::min(
    csv_speed * speed_scale_, 
    max_speed_
  );
  ```

#### 2.3 軌道品質検証
- **機能**: 読み込み軌道の妥当性チェック
- **チェック項目**:
  - 点間距離の妥当性
  - 速度値の範囲確認
  - クォータニオンの正規化確認

### Phase 3: レース特化最適化 🏁
**目標**: コーナー対応と性能最適化
**期間**: 3-4日
**変更行数目安**: ~150行

#### 3.1 曲率ベース速度調整
- **機能**: コーナーでの自動減速制御
- **実装**:
  ```cpp
  double calculateCurvature(const TrajectoryPoint& p1, p2, p3);
  void adjustSpeedByCurvature(std::vector<TrajectoryPoint>& points);
  ```

#### 3.2 コーナー検出・分類
- **低速コーナー**: 曲率半径 < 20m
- **中速コーナー**: 20m ≤ 曲率半径 < 50m  
- **高速コーナー**: 曲率半径 ≥ 50m
- **速度調整**: コーナー種別に応じた最適速度設定

#### 3.3 安全マージン制御
- **機能**: 動的軌道幅・安全距離調整
- **パラメータ**: 
  - `safety_margin`: 安全マージン係数
  - `corner_safety_factor`: コーナー安全係数

### Phase 4: デバッグ・監視強化 🔍
**目標**: 開発効率向上とトラブルシューティング支援
**期間**: 1-2日
**変更行数目安**: ~80行

#### 4.1 詳細ログ出力
- 軌道統計情報 (点数、総距離、平均速度)
- コーナー検出結果
- 性能メトリクス (処理時間、メモリ使用量)

#### 4.2 ROS2診断対応
- **実装**: `diagnostic_updater`による健全性監視
- **監視項目**:
  - 軌道データ品質
  - 配信性能
  - エラー発生状況

#### 4.3 可視化支援
- **MarkerArray配信**: RVizでの軌道可視化
- **速度プロファイル表示**: カラーマップによる速度表現

## 実装ガイドライン

### コード品質基準
- **複雑度制限**: 関数複雑度 ≤ 10
- **コミット単位**: 機能単位での小規模コミット
- **テスト**: 各機能の単体テスト実装

### パフォーマンス目標
- **配信遅延**: < 5ms
- **メモリ使用量**: < 前バージョンの80%
- **CPU使用率**: < 5% (定常時)

### 設定ファイル設計
```yaml
# config/my_trajectory_generator.yaml
my_trajectory_generator:
  ros__parameters:
    # Basic settings
    publish_rate: 20.0
    csv_path: "$(find-pkg-share my_trajectory_generator)/data/raceline_awsim_30km.csv"
    z: 6.5
    
    # Speed control
    speed_scale: 1.0
    max_speed: 15.0
    
    # Trajectory processing
    enable_interpolation: true
    interpolation_resolution: 0.5
    enable_curvature_adjustment: true
    
    # Safety
    safety_margin: 1.2
    corner_safety_factor: 0.8
    
    # Debug
    enable_visualization: false
    log_level: "INFO"
```

## 検証・評価計画

### Phase別評価指標
1. **Phase 1**: 配信頻度、レスポンス時間
2. **Phase 2**: 軌道スムージング品質、速度制御精度
3. **Phase 3**: コーナー通過性能、ラップタイム短縮
4. **Phase 4**: デバッグ効率、問題特定時間

### 最終評価基準
- **性能**: AWSIM シミュレーションでのラップタイム
- **安定性**: 100周連続走行成功率 > 95%
- **保守性**: コード複雑度、テストカバレッジ

## 次のアクション

1. **Phase 1開始**: 配信頻度とパラメータ拡張から着手
2. **設定ファイル作成**: パラメータ管理の統一
3. **テスト環境準備**: 改善効果の定量評価体制構築

---
*作成日: 2025-09-16*
*最終更新: 2025-09-16*
