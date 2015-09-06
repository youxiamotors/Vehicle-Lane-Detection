/**@ Function : Polyfit for OpenCV 
 **@ Auhtor : chouclee 
 **@ Date : Mar.15/2013 **/


#include <opencv2/opencv.hpp>
#include "equation.h"

using namespace cv;

void Polyfit(const Mat& src_x, const Mat& src_y, Mat& dst, int size) {
    int npoints = src_x.rows;
    Mat x = Mat::zeros(size, npoints, CV_MAKETYPE(DataType<double>::depth, 1));//(size)*npoints

    double* pmat_x = (double*)src_x.data;//重写了矩阵x的计算部分
    double* p_x_data = (double*)x.data;
    int step_x = (int)(x.step/x.elemSize1());
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < npoints; x++) {
            if (y == 0)
                p_x_data[x] = 1;
            else if (y == 1)
                p_x_data[x + step_x] = pmat_x[x];
            else p_x_data[x + y*step_x] = pmat_x[x]* p_x_data[x + (y-1)*step_x];
        }
    }

    Mat x_t, x_inv;
    transpose(x,x_t);//x_t --> npoints*(size)
    Mat temp = x*x_t;//(size)*npoints mul npoints*(size) --> (size)*(size)
    Mat temp2;
    invert (temp,temp2);
    Mat temp3 = temp2*x;//(size)*(size) mul (size)*npoints
    Mat W = temp3*src_y;//(size)*npoints mul npoints*(size)
    W.copyTo(dst);
}
