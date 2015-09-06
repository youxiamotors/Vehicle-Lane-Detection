#include "basic.h"

extern VisualSystem g_v;

void Perspective::Init() {
    /// IPM 仿射变换矩阵
    Point2f src[4];
    Point2f dst[4];

    int roi_width = g_v.kImgWidth;

    src[0].x = 0;
    src[0].y = 0;
    src[1].x = g_v.kIPMStartX;
    src[1].y = g_v.kRoiHeight;
    src[2].x = g_v.kIPMEndX;
    src[2].y = g_v.kRoiHeight;
    src[3].x = roi_width;
    src[3].y = 0;

    dst[0].x = 0;
    dst[0].y = 0;
    dst[1].x = 0;
    dst[1].y = g_v.kRoiHeight;
    dst[2].x = roi_width;
    dst[2].y = g_v.kRoiHeight;
    dst[3].x = roi_width;
    dst[3].y = 0;

    tsf_ipm = getPerspectiveTransform(dst, src);
    tsf_ipm_inv = tsf_ipm.inv();
}

void Perspective::IPM(Mat &src, Mat &dst) {
    warpPerspective(src, dst, tsf_ipm, src.size());
}


void GaussianFilter::Init(int _sigma_x, int _sigma_y) {
    sigma_x_ = _sigma_x;
    sigma_y_ = _sigma_y;
    onGaussianChange();
}

void GaussianFilter::onGaussianChange() {
    int i;
    float x;
    float xs[GAUSSIANSIZE] = {0};
    float ys[GAUSSIANSIZE] = {0};
    float sumx = 0, sumy = 0;

    //fprintf(stderr, "\\sigam_{x}: %d, \\sigma_{y}: %d, Gaussian Size: %d\n", sigma_x_, sigma_y_, GAUSSIANSIZE);    
    for (i = 0; i < GAUSSIANSIZE; i++) {
        x = 1.0 * i + 0.5 - GAUSSIANSIZE * 0.5;
        xs[i] = (1.0 / sigma_x_ / sigma_x_) * exp(-1.0 * x * x / 2 / sigma_x_ / sigma_x_) * (1 - x * x / sigma_x_ / sigma_x_);     
        ys[i] = exp(-1.0 * x * x / 2 / sigma_y_ / sigma_y_);

        sumx += xs[i];
        sumy += ys[i];
    }       
    for (i = 0; i < GAUSSIANSIZE; i++) {
        xs[i] /= sumx;
        ys[i] /= sumy;
    }       


    gaussian_kernel_x_ = Mat(1, GAUSSIANSIZE, CV_32F, xs).clone();
    gaussian_kernel_y_ = Mat(1, GAUSSIANSIZE, CV_32F, ys).clone();
}

void GaussianFilter::Filter(Mat src, Mat &dst) {
    sepFilter2D(src, dst, src.depth(), gaussian_kernel_x_, gaussian_kernel_y_);
}

void TopHat::Init() {
    morph_size_ = 3;
    element_ = getStructuringElement( MORPH_RECT, Size( 2*morph_size_ + 1, 2*morph_size_+1 ), Point( morph_size_, morph_size_ ) );
}

void TopHat::Transform(Mat &src, Mat &dest) {
    morphologyEx(src, dest, MORPH_TOPHAT, element_, Point(-1,-1), 4);
}
