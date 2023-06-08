# Calibration
FPP(Fringe　Projection Profilometry)・位相シフト法(Phase Shift Profilometry)において
カメラ内部パラメータ、カメラレンズ歪みパラメータ、LUTを用い、レンズ歪みを含まない計測結果を求める

# 01_Camera_Parameter.py
カメラの内部パラメータ・レンズ歪みパラメータ推定＆保存
(各画像の外部パラメータ推定はついで)

# 02_Camera_Calibration_Picture.cpp
撮影画像を内部キャリブレーション＆内部キャリブレーションした画像を保存
(カメラパラメータ取得済み)

# 03_LUT_generation.cpp
num(=3)回位相シフト縞投影した白ボード撮影画像からLUTを作成＆保存
(カメラパラメータ取得済み)

# 04_3D_measurement.cpp
num(=3)回位相シフト縞投影した計測物体撮影画像から三次元点群作成＆保存
(カメラパラメータ・LUT取得済み)
