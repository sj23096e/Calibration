# カメラの内部パラメータ・レンズ歪みパラメータ推定＆保存 (各画像の外部パラメータ推定はついで)
# 入力：チェスボード10枚以上
# 出力：内部パラメータ・レンズ歪みパラメータ
# 準備：チェスボードの格子数・格子1マスのサイズ
# 使用言語：Python3.9(64-bit)

import numpy as np
import cv2
import glob
from matplotlib import pyplot as plt

# 終了基準[termination criteria]
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# チェスボード設定[prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0))]
objp = np.zeros((8*7,3), np.float32)
objp[:,:2] = np.mgrid[0:8, 0:7].T.reshape(-1,2) * 12.0          ##格子1マスのサイズ[mm]入力(今は12mm)##

# 画像のオブジェクトポイントと画像ポイントを格納する配列[Arrays to store object points and image points from all the images.]
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
# Array to store images with image points
imgs = []

# 関数：推定したパラメータのRMSret、内部パラメータmtx、レンズ歪みパラメータdist表示
def printResult(ret, mtx, dist):
    print("*** calibration result ***")
    print("RMS of reprojection = ", ret, " (pixels)")
    print("Camera matrix = \n", mtx)
    print("Lens distortion = \n", dist)

# 関数：レンズ歪みパラメータdist、内部パラメータmtx保存
def saveIntrinsicParams(mtx, dist):
    f = open("intrinsicParams.dat", "wb")
    for i in range(3):
        for j in range(3):
            f.write(mtx[i][j])
    for i in range(5):
        f.write(dist[0][i])
    f.close()

# 関数：外部パラメータrvec, tvec表示
def printVecs(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    print("roation matrix     = \n", R)
    print("translation vector = \n", tvec)

# main関数
if __name__ == '__main__':
    import re
    def atoi(text):
        return int(text) if text.isdigit() else text

    def natural_keys(text):
        return [ atoi(c) for c in re.split(r'(\d+)', text) ]
   
    # 撮影したチェスボード読み込み
    images = sorted(glob.glob('calibration_images/simulation20230119_2/*.png'), key=natural_keys)   ##画像ファイル相対位置・形式##
    print(images)
    print(len(images))
    if len(images) == 0:
        print("error - no image file")
        exit(-1)
    elif len(images) < 10:
        print("error - not enough number of image files")
        exit(-1)
    
    # チェスボードの格子点検出
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                
        ret, corners = cv2.findChessboardCorners(gray, (8,7), None)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
        else:
            print("failure to detect corners: ", fname)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.drawChessboardCorners(img, (8,7), corners2, ret)
        imgs.append(img)

    # 内部パラメータ、レンズ歪みパラメータ推定
    ret, mtx, dist, rvecs ,tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    printResult(ret, mtx, dist)
    if ret > 1:
        print("failure to calculate matrix")
        exit(-1)
    h, w = img.shape[:2]

    # 歪み補正内部パラメータ推定、保存
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist.ravel(), (w,h), 1, (w,h))
    print("New camera matrix = \n", newcameramtx)
    print("save camera parameters (new intrinsic matrix and distortion vector)")
    saveIntrinsicParams(newcameramtx, dist)

    # 撮影画像の外部パラメータ表示、歪み補正前後の画像表示(ここは確認程度)
    i = 0
    for fname in images:
        print("\n*** image file name = ", fname, " ***")
        printVecs(rvecs[i], tvecs[i])
        img = cv2.imread(fname)
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        dst = cv2.cvtColor(dst, cv2.COLOR_BGR2RGB)
        fig = plt.figure(figsize=(14,6))
        fig.canvas.manager.set_window_title(fname)
        out = cv2.hconcat([imgs[i], dst])
        plt.imshow(out)
        plt.axis('off')
        plt.show()
        i = i + 1

# end of program.
