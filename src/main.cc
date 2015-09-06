#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "basic.h"

using namespace cv;
using namespace std;

VisualSystem g_v;

int main(int argc, char **argv) {
    char c;

    //从视频导入
    if (argc > 1) {
        g_v.capture_ = new VideoCapture(argv[1]);

        if (!g_v.capture_->isOpened()) {
            printf("打开视频失败");
            exit(0);
        }

    //从摄像头打开
    } else {
        g_v.capture_ = new VideoCapture(0);
    }

    cvNamedWindow("videoImage", CV_WINDOW_AUTOSIZE); 
    cvMoveWindow("videoImage", 30, 0);

    cvNamedWindow("roiImage", CV_WINDOW_AUTOSIZE); 
    cvMoveWindow("roiImage", 30, 450);

    cvNamedWindow("retImage", CV_WINDOW_AUTOSIZE); 
    cvMoveWindow("retImage", 30, 650);

    cvNamedWindow("cavas", CV_WINDOW_AUTOSIZE); 
    //cvMoveWindow("cavas",650, 450);
    cvMoveWindow("cavas", 600, 450);

    g_v.Init();

    while(1) {
        g_v.LoopImg();

        imshow("videoImage", g_v.img_src_);
        imshow("roiImage", g_v.img_roi_);
        imshow("retImage", g_v.img_ret_);
        imshow("cavas", g_v.img_cavas_);

        c = waitKey(0);
        c = waitKey(1);
        if ('c' == c) {
            waitKey(0);
        }
    }

    return 0;
}
