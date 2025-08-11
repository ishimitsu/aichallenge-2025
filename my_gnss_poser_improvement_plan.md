# my_gnss_poser 高性能化改善計画書

## プロジェクト概要
自動運転システムにおけるGNSS姿勢推定モジュール`my_gnss_poser`を、既存の`imu_gnss_poser`を超える高性能システムに改善するプロジェクト。

## 現在の状況
- **パッケージ場所**: `/home/ishimitsu/Works/AutomotiveAIChallenge/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/my_gnss_poser/`
- **現在のブランチ**: `improve-my-gnss-poser`
- **現在の実装**: 基本的なPoseStamped → PoseWithCovariance変換のみ
- **launch設定**: `reference.launch.xml`で`my_gnss_poser`パッケージを参照

## 既存システム比較分析

### my_gnss_poser（現在の実装）
```cpp
// 入力: /sensing/gnss/pose (PoseStamped)
// 出力: /localization/imu_gnss_poser/pose_with_covariance (PoseWithCovariance)
// 機能: 単純なメッセージ変換、固定共分散値設定
```

### imu_gnss_poser（比較対象）
```cpp
// 入力: 
//   - /sensing/gnss/pose_with_covariance (PoseWithCovarianceStamped)
//   - /sensing/imu/imu_raw (Imu)
//   - /localization/twist_with_covariance (TwistWithCovarianceStamped)
// 出力:
//   - /localization/imu_gnss_poser/pose_with_covariance (PoseWithCovarianceStamped) 
//   - /localization/initial_pose3d (PoseWithCovarianceStamped)
// 機能: IMU融合、EKF初期化管理、共分散制御、異常値補完
```

### imu_gnss_poserの制限事項
- 単純な共分散設定（固定値）
- 基本的なIMU融合（NaN/0チェックのみ）
- GNSS信号品質考慮不足
- 時系列処理なし
- 異常検知が基本的

## 改善計画

### Phase 1: 基礎強化
#### 1. 動的共分散推定機能
- **HDOP/VDOP値による精度評価**
  - GNSS精度指標を活用した動的共分散調整
  - 衛星配置による精度予測
- **衛星数に基づく信頼度調整**
  - 可視衛星数による信頼度スコア算出
  - マルチコンステレーション対応
- **移動状態検出による適応制御**
  - 静止/低速/高速状態の自動検出
  - 各状態に最適化されたフィルタパラメータ

#### 2. 信号品質監視
- **SNR値監視とフィルタリング**
  - 信号強度による品質評価
  - 低品質信号の自動除外
- **異常値検出とリジェクション**
  - 統計的外れ値検出
  - 連続値との整合性チェック
- **連続性チェック機能**
  - 位置ジャンプ検出
  - 時系列一貫性検証

### Phase 2: 融合強化
#### 3. 拡張カルマンフィルタ実装
- **IMU/GNSS/オドメトリ統合**
  - 多センサーデータ融合
  - 各センサーの特性を活かした重み付け
- **非線形運動モデル対応**
  - 車両運動学モデル組み込み
  - 曲線運動の高精度推定
- **適応的ノイズ推定**
  - 環境に応じたノイズパラメータ自動調整
  - 学習ベースのノイズモデル

#### 4. 時系列処理強化
- **移動平均フィルタ**
  - 適応的窓幅制御
  - 異常値耐性向上
- **予測・補間機能**
  - GNSS信号断時の位置予測
  - スプライン補間による軌道平滑化
- **遅延補償機構**
  - センサー間時刻同期
  - 伝送遅延補正

### Phase 3: 高度化
#### 5. RTK/PPP対応
- **RTCM補正データ処理**
  - リアルタイム補正データ受信・処理
  - ネットワークRTK対応
- **基準局連携機能**
  - 複数基準局データ統合
  - 最適基準局自動選択
- **高精度測位実現**
  - cm級精度の実現
  - 整数値バイアス解決

#### 6. AI/ML統合
- **LSTM/Transformerによる軌道予測**
  - 深層学習による高精度軌道予測
  - 長期予測能力の実現
- **異常パターン学習**
  - 環境固有の異常パターン学習
  - 予防的異常検知
- **環境適応型調整**
  - 都市部/郊外/高速道路別最適化
  - リアルタイム環境認識

## 性能目標

### 精度向上
- **水平精度**: 50cm → 5cm (RTK使用時)
- **垂直精度**: 1m → 10cm
- **姿勢精度**: 1° → 0.1°

### 堅牢性向上
- **信号遮断耐性**: 5秒 → 30秒
- **異常検知率**: 基本レベル → 99.9%
- **初期化時間**: 30秒 → 5秒

### 計算効率
- **リアルタイム性**: 100Hz処理対応
- **メモリ使用量**: 最適化により50%削減
- **CPU負荷**: マルチスレッド処理で負荷分散

## 技術要素

### 新規依存関係
```xml
<depend>sensor_msgs</depend>
<depend>tf2_geometry_msgs</depend>
<depend>eigen3_cmake_module</depend>
<depend>geographic_msgs</depend>
<depend>nav_msgs</depend>
<depend>std_msgs</depend>
```

### 主要アルゴリズム
- **Extended Kalman Filter (EKF)**: 非線形システムの状態推定
- **Unscented Kalman Filter (UKF)**: 高非線形システム対応
- **Particle Filter**: 多峰性分布対応
- **LSTM Neural Network**: 時系列予測

### 新規ファイル構成（予定）
```
my_gnss_poser/
├── src/
│   ├── pose_transformer.cpp (現在: 基本変換)
│   ├── gnss_quality_monitor.cpp (新規)
│   ├── extended_kalman_filter.cpp (新規)
│   ├── rtk_processor.cpp (新規)
│   └── ml_predictor.cpp (新規)
├── include/my_gnss_poser/
│   ├── types.hpp
│   ├── filters.hpp
│   └── utils.hpp
├── config/
│   └── params.yaml
├── package.xml (依存関係更新)
└── CMakeLists.txt (ビルド設定更新)
```

## 実装順序

### Step 1: 環境準備
1. 依存関係追加
2. ビルドシステム更新
3. 基本構造リファクタリング

### Step 2: Phase 1実装
1. GNSS品質監視機能
2. 動的共分散推定
3. 異常値検出・除外

### Step 3: Phase 2実装
1. EKF基本実装
2. IMU融合機能
3. 時系列処理

### Step 4: Phase 3実装
1. RTK対応
2. ML予測機能
3. 性能最適化

## テスト・検証計画

### 単体テスト
- 各モジュールの独立テスト
- 境界値・異常値テスト

### 統合テスト
- センサー統合動作検証
- リアルタイム性能測定

### 実環境テスト
- 様々な環境での精度検証
- 長時間安定性試験

## 開発方針

### コミット管理方針
- **コミットメッセージ制限**: 200文字以内に収める
- **長いメッセージ対策**: 200文字を超えそうな場合は事前に警告し、以下を検討
  - メッセージの簡潔化
  - 作業を複数の小さなコミットに分割
- **目的**: コミット履歴の可読性向上とレビューの効率化

### 定数管理方針
- **定数値の根拠明記**: 全ての定数値にコメントで根拠・出典を記載
- **根拠の種類**:
  - 仕様書・標準規格に基づく値
  - 経験的に調整された値（empirically tuned）
  - 物理的制約に基づく値
  - 既存システムとの互換性維持のための値
- **不明な根拠**: 推定値や仮定値の場合は明示し、後で調整可能であることを記載
- **目的**: コードの保守性向上と設定変更時の影響把握

## 作業再開時の指示

### このファイルの確認
1. 現在の進捗状況を把握
2. 次に実装すべき機能を特定
3. 技術要件・制約事項を確認

### 作業環境準備
```bash
cd /home/ishimitsu/Works/AutomotiveAIChallenge/aichallenge-2025
git checkout improve-my-gnss-poser
```

### 実装開始手順
1. Phase 1の機能から段階的に実装
2. 各機能実装後にテスト・検証
3. 性能測定とベンチマーク比較
4. コミット・プッシュで進捗保存

## 関連ファイルパス
- **パッケージディレクトリ**: `/home/ishimitsu/Works/AutomotiveAIChallenge/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/my_gnss_poser/`
- **launch設定**: `/home/ishimitsu/Works/AutomotiveAIChallenge/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/reference.launch.xml`
- **比較対象**: `/home/ishimitsu/Works/AutomotiveAIChallenge/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/imu_gnss_poser/`

## 進捗状況

### 完了項目 ✅
- **Phase 1 基礎機能**: 完了
  - 環境準備（依存関係追加）
  - 基本構造リファクタリング（PoseTransformer → GnssImuPoser）
  - GNSS品質監視機能実装
  - 動的共分散推定機能実装
  - 基本IMU融合機能実装

- **コード品質改善**: 完了
  - if-else連鎖をswitch文にリファクタリング
  - 全マジックナンバーを名前付き定数に置換（23個の定数）
  - 全定数に根拠説明コメントを追加
  - 開発方針策定（コミット管理・定数管理）

### 現在の状況
- **ブランチ**: `improve-my-gnss-poser`
- **最新コミット**: cb3a5ec "Add documentation for constants and establish development policies"
- **機能レベル**: 既存`imu_gnss_poser`の基本機能を上回る状態

### 次回作業予定
- **Phase 1 残り機能**: 異常値検出とリジェクション
  - Step 5: 異常値検出機能（統計的外れ値検出）
  - Step 6: 位置ジャンプ検出機能
  - Step 7: 時系列一貫性チェック機能
  - Phase 1完了確認とテスト

### 技術的な実装状況
- **新クラス**: `GnssImuPoser`（pose_transformer.cpp）
- **主要機能**:
  - 動的GNSS品質評価（NavSatFixステータス基準）
  - 品質に基づく適応的共分散調整
  - IMU履歴管理とフィルタリング
  - 無効姿勢データのIMU補完
- **設定可能な定数**: 23個（全て根拠コメント付き）

---
**作成日**: 2025-08-11  
**最終更新**: 2025-08-11  
**プロジェクト状況**: Phase 1 基礎機能完了、異常値検出実装待ち  
**次回作業**: Phase 1 異常値検出機能実装開始