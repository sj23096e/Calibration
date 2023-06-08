/* num(=3)回位相シフト縞投影した白ボード撮影画像からLUTを作成＆保存 */
/* 入力：白色ボード撮影画像(計測最前面、最後面各num枚) */
/* 出力：LUT */
/* 準備：カメラパラメータ */
/* 使用言語：c++ */

#include <fstream>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include "matplotlib-cpp/matplotlibcpp.h"//<matplotlib-cpp/matplotlibcpp.h>
#define _USE_MATH_DEFINES
#include <math.h> //M_PI(円周率)利用のため追加

/* 関数：カメラのレンズ歪みパラメータdist、内部パラメータmtx読み込み */
bool
loadIntrinsicParams(cv::Mat& mtx, cv::Mat& dist)
{
    std::ifstream f("C:/Users/owner/source/repos/04_LUT_generation/LUT_generation/intrinsicParams.dat", std::ios::in | std::ios::binary);  //カメラパラメータの絶対位置に変更  ##intrinsicParams.datの絶対位置##
    //↑LUT_generationと同階層に、使うintrinsicParams.datを置く
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

/* 関数：3stepシフト法による相対位相導出 */
void
calculateRphases3(float* in, float* rphase, float* amplitude, float* bias)
{
    *bias = (in[0] + in[1] + in[2]) / 3.0f;
    *amplitude = sqrtf((2.0f * in[0] - in[1] - in[2]) * (2.0f * in[0] - in[1] - in[2])
        + 3.0f * (in[1] - in[2]) * (in[1] - in[2])) / 3.0f;
    *rphase = atan2f(sqrtf(3.0f) * (in[2] - in[1]), 2.0f * in[0] - in[1] - in[2]);
}

/* 関数：縞波形、振幅、バイアス表示 */
void
plotSinusoidalWaves(cv::Mat sinusoidalwaves[], cv::Mat amplitude[], cv::Mat bias[],
    const int num)
{
    int row = sinusoidalwaves[0].rows / 2;
    int cols = sinusoidalwaves[0].cols;
    std::vector<float> u, v[3 + 2]; //##ステップ数(今は3)##
    std::ostringstream ss;

    for (int times = 0; times < 2; times++) {
        for (int col = 0; col < cols; col++) {  //cols=640(CAP_PROP_FRAME_WIDTH)
            u.push_back(col);
            for (int i = 0; i < num; i++) {
                v[i].push_back(sinusoidalwaves[i + num * times].at<float>(row, col));
            }
            v[num].push_back(amplitude[times].at<float>(row, col));
            v[num + 1].push_back(bias[times].at<float>(row, col));
          }
        if (times == 0)
            matplotlibcpp::title("Pixel Values at near end");
        else
            matplotlibcpp::title("Pixel Values at far end");
        matplotlibcpp::xlabel("x (pixel)");
        matplotlibcpp::xlim(0, cols);
        matplotlibcpp::ylim(0, 255);
        for (int i = 0; i < num; i++) {
            std::string title = "pixel values (phase ";
            ss << i;
            title.append(ss.str());
            title.append(")");
            matplotlibcpp::named_plot(title, u, v[i]);
            ss.str("");
        }
        matplotlibcpp::named_plot("amplitudes", u, v[num]);
        matplotlibcpp::named_plot("biases", u, v[num + 1]);
        matplotlibcpp::legend();
        matplotlibcpp::show();        
        u.clear();
        for (int i = 0; i < num + 2; i++) {
            v[i].clear();            
            
        }
    }
}

/* 関数：相対位相、正規化振幅、正規化バイアス表示 */
void
plotRelativePhases(cv::Mat rphase[], cv::Mat amplitude[], cv::Mat bias[])
{
    int row = rphase[0].rows / 2;
    int cols = rphase[0].cols;
    float scaleparam = 3.0f / 255.0f;
    std::vector<float> u(cols), v0(cols), v1(cols), v2(cols), v3(cols), v4(cols), v5(cols);
    for (int col = 0; col < cols; col++) {
        u[col] = col;
        v0[col] = rphase[0].at<float>(row, col);
        v1[col] = scaleparam * amplitude[0].at<float>(row, col);
        v2[col] = scaleparam * bias[0].at<float>(row, col);
        v3[col] = rphase[1].at<float>(row, col);
        v4[col] = scaleparam * amplitude[1].at<float>(row, col);
        v5[col] = scaleparam * bias[1].at<float>(row, col);
    }
    matplotlibcpp::title("Relative Phases");
    matplotlibcpp::xlabel("x (pixel)");
    matplotlibcpp::xlim(0, cols);
    matplotlibcpp::ylim(-M_PI, M_PI);
    matplotlibcpp::named_plot("relative phases (near end)", u, v0);
    matplotlibcpp::named_plot("amplitudes normalized between [0, 3] (near end)", u, v1);
    matplotlibcpp::named_plot("biases normalized between [0, 3] (near end)", u, v2);
    matplotlibcpp::named_plot("relative phases (far end)", u, v3);
    matplotlibcpp::named_plot("amplitudes normalized between [0, 3] (far end)", u, v4);
    matplotlibcpp::named_plot("biases normalized between [0, 3] (far end)", u, v5);
    matplotlibcpp::legend();
    matplotlibcpp::show();
}

/* 関数：閾値処理 */
void
thresholdAmplitudes(cv::Mat rphase[], cv::Mat amplitude[], float threshold)
{
    cv::Mat amp = cv::Mat::zeros(amplitude[0].rows, amplitude[0].cols, CV_32FC1);
    for (int row = 0; row < amp.rows; row++) {
        for (int col = 0; col < amp.cols; col++) {
            if (amplitude[0].at<float>(row, col) < threshold || amplitude[1].at<float>(row, col) < threshold) {
                for (int times = 0; times < 2; times++)
                    rphase[times].at<float>(row, col) = -FLT_MAX;
            }
            else {
                amp.at<float>(row, col) = 255.0f;
                if (rphase[0].at<float>(row, col) > rphase[1].at<float>(row, col)) {
                    rphase[1].at<float>(row, col) += 2.0f * static_cast<float>(M_PI);
                }
            }
        }
    }
    matplotlibcpp::imshow(&(amp.at<float>(0, 0)), amp.rows, amp.cols, 1);
    matplotlibcpp::axis("off");
    matplotlibcpp::show();
}

/* 関数：三次元座標位置推定 */
void
calc3DcoordinateValues(cv::Mat coordv[], cv::Mat& mtx, float distance[])
{
    float fx = static_cast<float>(mtx.at<double>(0, 0));
    float fy = static_cast<float>(mtx.at<double>(1, 1));
    float cx = static_cast<float>(mtx.at<double>(0, 2));
    float cy = static_cast<float>(mtx.at<double>(1, 2));
    for (int times = 0; times < 2; times++) {
        for (int row = 0; row < coordv[0].rows; row++) {
            double Y = (row - cy) * distance[times] / fy;
            for (int col = 0; col < coordv[0].cols; col++) {
                double X = (col - cx) * distance[times] / fx;
                coordv[times].at<cv::Vec3f>(row, col)[0] = X;
                coordv[times].at<cv::Vec3f>(row, col)[1] = Y;
                coordv[times].at<cv::Vec3f>(row, col)[2] = distance[times];
            }
        }
    }
}

/* main関数 */
int
main(int argc, char* argv[])
{
    /* 変数等定義 */    
    const int num = 3;          //##ステップ数##
    const int threshold = 10;   //##閾値##
    cv::Mat frame[num * 2];     //frame:読み込み画像
    cv::Mat undistortedframe[num * 2];//undistortedframe:歪み補正画像

    /* カメラ内部パラメータ読み込み */
    cv::Mat mtx = cv::Mat(3, 3, CV_64FC1), dist = cv::Mat(1, 5, CV_64FC1);
        if (!loadIntrinsicParams(mtx, dist)) {
        std::cerr << "file (./intrinsicParam.dat) open error" << std::endl;
        return -1;
    }

    /* 複数画像読み込み */
        //読み込み画像は"LUT_n(f)_pic/jikkenn(simuration)/2023****_f**/"に入れておく ※これ以下の下層ディレクトリは無理
    for (int a = 0; a < num; a++) {
        std::ostringstream ass;
        ass << std::setfill('0') << std::setw(1) << a;
        frame[a] = cv::imread("LUT_n_pic\\jikkenn\\20230203_f32_exp200\\" + ass.str() + ".bmp");          //    ##最前面位置の撮影画像の相対位置、形式##
        frame[a + num] = cv::imread("LUT_f_pic\\jikkenn\\20230203_f32_exp200\\" + ass.str() + ".bmp");    //    ##最後面位置の撮影画像の相対位置、形式##
        printf("%d", a);
    }
    
    /* 画像のレンズ歪み補正 */
    for (int i = 0; i < num * 2; i++) {
        cv::undistort(frame[i], undistortedframe[i], mtx, dist);
    }

    /* 縞波形、相対位相等計算 */
    cv::Mat sinusoidalwaves[num * 2], amplitude[2], bias[2], rphase[2], coord[2];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < num; j++)
            sinusoidalwaves[j + num * i] = cv::Mat(480, 640, CV_32FC1);
        amplitude[i] = cv::Mat(480, 640, CV_32FC1);
        bias[i] = cv::Mat::zeros(480, 640, CV_32FC1);
        rphase[i] = cv::Mat::zeros(480, 640, CV_32FC1);
        coord[i] = cv::Mat::zeros(480, 640, CV_32FC3);
    }
    float gamma = 1.00f;    //  ##カメラのガンマ値##
    for (int times = 0; times < 2; times++) {
        for (int row = 0; row < undistortedframe[0].rows; row++) {
            for (int col = 0; col < undistortedframe[0].cols; col++) {
                float in[num];
                for (int i = 0; i < num; i++) {
                    in[i] = 0.0f;
                    for (int bgr = 0; bgr < 3; bgr++) {
                        in[i] += 255.0f
                            * powf(static_cast<float>(undistortedframe[i + num * times].at<cv::Vec3b>(row, col)[bgr]) / 255.0f,
                                gamma) / 3.0f;
                    }
                    sinusoidalwaves[i + num * times].at<float>(row, col) = in[i];
                }
                // *** in case of num == 3
                calculateRphases3(in, &rphase[times].at<float>(row, col),
                		  &amplitude[times].at<float>(row, col),
                		  &(bias[times].at<float>(row, col)));
            }
        }
    }

    /* 縞波形、相対位相等表示 */
    plotSinusoidalWaves(sinusoidalwaves, amplitude, bias, num);
    plotRelativePhases(rphase, amplitude, bias);

    std::cout << "created the two LUTs and saved them." << std::endl;

    /* 閾値処理 */
    thresholdAmplitudes(rphase, amplitude, threshold);

    /* 三次元座標位置推定 */
    float distance[2] = { 475.0f, 525.0f };     //最前面位置、最後面位置[mm]     ##カメラ位置##
    calc3DcoordinateValues(coord, mtx, distance);

    /* ファイルポインタ設定(LUT保存用) */
    //std::string filename = "LUTs.xml";
    cv::FileStorage fs("LUTs.xml", cv::FileStorage::WRITE);   //filename挟んでも良し

    /* LUTに相対位相、三次元座標位置格納 */
    fs << "LUTatNearEnd" << rphase[0];
    fs << "CoordinateValuesatNearEnd" << coord[0];
    fs << "LUTatFarEnd" << rphase[1];
    fs << "CoordinateValuesatFarEnd" << coord[1];
    fs.release();

    cv::destroyAllWindows();
    return 0;
}
// end of program
