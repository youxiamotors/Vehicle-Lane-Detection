#ifndef VLD_EQUATION_H_
#define VLD_EQUATION_H_

using namespace cv;

//theta 取 [0, 180)
#define EqCalculateThetaByK(theta, k) do {\
    if (1 == cvIsNaN(k)) {\
        theta = RADIAN_90;\
    } else {\
        theta = atan(k);\
    }\
    if (theta < 0) {theta = fabs(theta);}\
    else {theta = RADIAN_180 - theta;}\
} while(0);

#define EqCalculateLineKBBy2Points(k, b, p1, p2) do {\
    if (fabsf(p1.x - p2.x) < 0.01) {\
        k = NAN;\
        b = p1.x;\
    } else {\
        k = (float)(p2.y - p1.y) / (float)(p2.x - p1.x);\
        b = p2.y - k * p2.x;\
    }\
} while(0);

#define EqCalculateLineKBy2Points(k, p1, p2) do {\
    if (fabsf(p1.x - p2.x) < 0.01) {\
        k = NAN;\
    } else {\
        k = (float)(p2.y - p1.y) / (float)(p2.x - p1.x);\
    }\
} while(0);

#define EqCalculateLineThetaBy2Points(theta, p1, p2) do {\
    if (fabsf(p1.x - p2.x) < 0.01) {\
        theta = NAN;\
    } else {\
        theta = (float)(p2.y - p1.y) / (float)(p2.x - p1.x);\
    }\
    EqCalculateThetaByK(theta, theta);\
} while(0);

#define EqCalculateXByYKB(x, default_x, y, k, b) do {\
    if (cvIsNaN(k)) {\
        x = default_x;\
    } else {\
        x = (y - b) / k;\
    }\
} while(0);

void Polyfit(const Mat& src_x, const Mat& src_y, Mat& dst, int size);

inline void Polyval(const Mat& cst_mat, const Mat& mat_x, Mat& mat_y) {
    int i,s;
    double x;

    CV_Assert(mat_x.channels() == 1);

    s = mat_x.rows;
    for (i = 0; i < s; i++) {
        x = mat_x.ptr<double>(i)[0];

        mat_y.ptr<double>(i)[0] = cst_mat.ptr<double>(0)[0] +
            cst_mat.ptr<double>(1)[0] * x + 
            cst_mat.ptr<double>(2)[0] * pow(x,2) + 
            cst_mat.ptr<double>(3)[0] * pow(x,3);
    }
}

inline void Polyval(const Mat& cst_mat, const Mat& mat_x, vector<Point2f>& dst_points) {
    int i,s;
    double x,y;

    CV_Assert(mat_x.channels() == 1);

    dst_points.clear();

    s = mat_x.rows;
    for (i = 0; i < s; i++) {
        x = mat_x.ptr<double>(i)[0];

        y = cst_mat.ptr<double>(0)[0] +
            cst_mat.ptr<double>(1)[0] * x + 
            cst_mat.ptr<double>(2)[0] * pow(x,2) + 
            cst_mat.ptr<double>(3)[0] * pow(x,3);

        dst_points.push_back(Point2f(x,y));
    }
}

inline double Polyval(const Mat& cst_mat, double x) {
    return cst_mat.ptr<double>(0)[0] +
        cst_mat.ptr<double>(1)[0] * x + 
        cst_mat.ptr<double>(2)[0] * pow(x,2) + 
        cst_mat.ptr<double>(3)[0] * pow(x,3);
}

#endif // VLD_EQUATION_H_
