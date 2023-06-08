/* 撮影画像を内部キャリブレーション＆内部キャリブレーションした画像を保存 */
/* 入力：撮影画像 */
/* 出力：内部キャリブレーション済み撮影画像 */
/* 準備：カメラパラメータ */
/* 使用言語：c++ */

#include <fstream>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include "matplotlib-cpp/matplotlibcpp.h" //<matplotlib-cpp/matplotlibcpp.h>
#include <sstream>
#include <iomanip>
#define _USE_MATH_DEFINES
#include <math.h> //M_PI(円周率)利用のため追加

/* 関数：カメラのレンズ歪みパラメータdist、内部パラメータmtx読み込み */
bool
loadIntrinsicParams(cv::Mat& mtx, cv::Mat& dist)
{
    std::ifstream f("C:\\Users\\owner\\source\\repos\\03_camera_geometrical_calibration_cpp\\3D_measurement\\intrinsicParams.dat", std::ios::in | std::ios::binary);    //  ##intrinsicParams.datの絶対位置##
    if (!f) {
        return false;
    }
    for (int row = 0; row < mtx.rows; row++) {
        for (int col = 0; col < mtx.cols; col++) {
            f.read(reinterpret_cast<char*>(&(mtx.at<double>(row, col))), sizeof(double));
        }
    }
    for (int row = 0; row < dist.rows; row++) {
        for (int col = 0; col < dist.cols; col++) {
            f.read(reinterpret_cast<char*>(&(dist.at<double>(row, col))), sizeof(double));
        }
    }
    f.close();
    return true;
}

/* main関数 */
int
main(int argc, char* argv[])
{
    /* num(ステップ数)、m(読み取り枚数)定義 */
    const int num = 3;  //  ##ステップ数##
    const int m = 3;    //  ##読み取り枚数##

    /* カメラの内部パラメータ読み込み */
    cv::Mat mtx = cv::Mat(3, 3, CV_64FC1), dist = cv::Mat(1, 5, CV_64FC1);
    if (!loadIntrinsicParams(mtx, dist)) {
        std::cerr << "file (intrinsic parameters) open error" << std::endl;
        return -1;
    }

    /* 撮影画像複数読み込み+undistort（レンズ歪み補正）+歪み補正後データ保存("元のファイル名_cal"で) */
        //frame:読み込み画像,undistortedframe:歪み補正後画像
    cv::Mat frame[m];
    cv::Mat undistortedframe[m];
    for (int a = 0; a < num; a++) {
        /* 撮影画像のファイル名読み込みのため */
        std::ostringstream ass;
        ass << std::setfill('0') << std::setw(1) << a;

        /* frame[]に撮影画像読み込み */
        frame[a] = cv::imread("20230209\\" + ass.str() + ".bmp");          //   ##撮影画像の相対位置・形式##
        if (frame[a].empty() == true) {
            printf("frame[%d] is empty.\n", a);
            // 画像の中身が空なら終了する
            return 0;
        }

        /* 撮影画像のレンズ歪み補正、歪み補正画像undistortedframe[]保存 */
        cv::undistort(frame[a], undistortedframe[a], mtx, dist);
        cv::imwrite("20230209\\" + ass.str() + "_cal.png", undistortedframe[a]); // ##歪み補正画像の相対保存位置・形式##
    }
   return 0;
}
// end of program
