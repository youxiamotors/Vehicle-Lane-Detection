#include "basic.h"

#define POINTS_SIZE 2
#define EQUATION_CONSTANT_TERMS_SIZE 4

VisualSystem g_v;

int main(int argc, char **argv) {
    int i;
    Mat points_x, points_y, eq_constant_terms;
    vector<Point2f> dst_points;
    Point2f *point_ptr;

    points_x.create(POINTS_SIZE*4, 1, CV_64FC1);
    points_y.create(POINTS_SIZE*4, 1, CV_64FC1);
    eq_constant_terms.create(EQUATION_CONSTANT_TERMS_SIZE, 
            1, CV_MAKETYPE(DataType<double>::depth, 1));

    for (i = 0; i < POINTS_SIZE * 4; i+=4) {
        points_x.ptr<double>(i)[0] = i;
        points_x.ptr<double>(i+1)[0] = i;
        points_x.ptr<double>(i+2)[0] = i;
        points_x.ptr<double>(i+3)[0] = i;
        points_y.ptr<double>(i)[0] = i;
        points_y.ptr<double>(i+1)[0] = i;
        points_y.ptr<double>(i+2)[0] = i;
        points_y.ptr<double>(i+3)[0] = i;
    }

    Polyfit(points_y, points_x, eq_constant_terms, LANE_EQUATION_CONSTANT_TERMS_SIZE);

    LOGI("%f %f %f %f\n", 
            eq_constant_terms.ptr<double>(0)[0],
            eq_constant_terms.ptr<double>(1)[0],
            eq_constant_terms.ptr<double>(2)[0],
            eq_constant_terms.ptr<double>(3)[0]);

    Polyval(eq_constant_terms, points_y, dst_points);

    for (i = 0; i < dst_points.size(); i++) {
        point_ptr = &dst_points.at(i);
        LOGI("%f %f\n", point_ptr->x, point_ptr->y);
    }

    return 0;
}
