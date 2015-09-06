#ifndef VLD_IMG_TRANSFORM_H_
#define VLD_IMG_TRANSFORM_H_

#include <opencv2/core/core.hpp>

using namespace cv;

class Perspective {
    public:
        Mat tsf_ipm, tsf_ipm_inv;
        void Init();
        void IPM(Mat &src, Mat &dst);
};

class GaussianFilter {
    public:
        void Init(int _sigma_x, int _sigma_y);
        void Filter(Mat src, Mat &dst);
    private:
        Mat gaussian_kernel_x_;
        Mat gaussian_kernel_y_;
        int sigma_x_;
        int sigma_y_;
        void onGaussianChange();
};

class TopHat {
    public:
        int morph_size_;
        Mat element_;
        void Init();
        void Transform(Mat &src, Mat &dest);
};

#endif // VLD_IMG_TRANSFORM_H_
