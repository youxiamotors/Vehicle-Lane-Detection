#ifndef VLD_BASIC_H_
#define VLD_BASIC_H_

#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * 规定:
 * y轴递减方向为正方向
 */

using namespace cv;

#define RADIAN_1                0.017453292519943295
#define RADIAN_90               1.5707963267948966
#define RADIAN_180              3.141592653589793
#define ADIAN_NEGATIVE_90      -1.5707963267948966

#include "config.h"
#include "util.h"
#include "equation.h"
#include "img_transform.h"

typedef struct LaneLineNode {
    LaneLineNode() {}
    Point2i p1;
    Point2i p2;
    float k,b; //线性方程中y=kx+b 中的k,b
    float theta;
    float score;
    void Clear() {}
} LaneLineNode;

typedef struct DoubleLaneLineNode {
    DoubleLaneLineNode() {this->Clear();}
    LaneLineNode *lln1_ptr;
    LaneLineNode *lln2_ptr;
    LaneLineNode lln;
    int llns_sum;
    float llns_distance;
    void Clear() {lln1_ptr = 0; lln2_ptr = 0; llns_sum = 0; llns_distance = 999999999;}
} DoubleLaneLineNode;

typedef struct LaneLineFlow {
    LaneLineFlow() {this->Clear();}
    float score;
    float theta;
    int top_point_x;
    int bottom_point_y;
    vector<LaneLineNode*> nodes;
    void Clear() {score = 0; nodes.clear();}
} LaneLineFlow;

typedef struct TimeLaneLineFlow {
    TimeLaneLineFlow() {this->Clear();}
    float score;
    float theta;
    int top_point_x;
    int bottom_point_y;
    Pool<LaneLineNode> nodes;
    void Clear() {score = 0; nodes.Clear();}
    void Overlay(const LaneLineFlow *llf_ptr) {
        int i, size;
        LaneLineNode *tllf_lln_ptr;

        this->score = llf_ptr->score;
        this->theta = llf_ptr->theta;
        this->top_point_x = llf_ptr->top_point_x;
        this->bottom_point_y = llf_ptr->bottom_point_y;

        nodes.Clear();

        size = llf_ptr->nodes.size();
        for (i = 0; i < size; i++) {
            tllf_lln_ptr = nodes.new_node_v();
            *tllf_lln_ptr = *llf_ptr->nodes.at(i);
        }
    }
} TimeLaneLineFlow;

enum LaneLineType {
    DOTTED_LINE,
    FULL_LINE
};

typedef struct LaneLine {
    LaneLine() {
        score = 0;
        points_x.create(1, 1, CV_64FC1);
        points_y.create(1, 1, CV_64FC1);
        //目前简单起见，只做单通道的数据拟合
        eq_constant_terms.create(LANE_EQUATION_CONSTANT_TERMS_SIZE, 
                1, CV_MAKETYPE(DataType<double>::depth, 1));
    }

    float score;
    float theta;
    int top_point_x;
    int bottom_point_y;
    Pool<LaneLineNode> nodes;
    Mat eq_constant_terms;
    Mat points_x;
    Mat points_y;
    LaneLineType type;

    void Clear() {score = 0; nodes.Clear();}
    void Overlay(const LaneLineFlow *llf_ptr) {
        int i, size;
        LaneLineNode *ll_lln_ptr;

        this->score = llf_ptr->score;
        this->theta = llf_ptr->theta;
        this->top_point_x = llf_ptr->top_point_x;
        this->bottom_point_y = llf_ptr->bottom_point_y;

        nodes.Clear();

        size = llf_ptr->nodes.size();
        for (i = 0; i < size; i++) {
            ll_lln_ptr = nodes.new_node_v();
            *ll_lln_ptr = *llf_ptr->nodes.at(i);
        }
    }

    void ImproveScore(float _score) {
        if (_score > 0) {
            score += _score;
            if (score >= SCORE_MAX) score = SCORE_MAX;
        }
    }
} LaneLine;

typedef struct Lane {
    Lane() {score = 0;}

    float score;

    LaneLine left_ll;
    LaneLine right_ll;

    void Clear() {
        score = 0;
        left_ll.Clear();
        right_ll.Clear();
    }

    void ImproveScore(float _score) {
        if (_score > 0) {
            score += _score;
            if (score >= 26) score = 26;
        }
    }

    Lane *next, *prev;
} Lane;

class VisualSystem {
    public:
    int kImgWidth, kImgHeight;
    int kRoiHeight, kRoiStartY;
    int kIPMStartX, kIPMEndX;
    int kHotLaneAreaStartX, kHotLaneAreaEndX;

    int lane_width_;
    int lane_width_max_;
    int lane_width_min_;

    float lane_theta_;
    float lane_theta_score_;

    VideoCapture *capture_;
    Mat img_src_;
    Mat img_gray_;
    Mat img_roi_;
    Mat img_ret_;
    Mat img_cavas_;

    Perspective perspective_;
    GaussianFilter gaussian_filter_;
    TopHat tophat_;

    float canny_threshold_;

    void Init();
    void LoopImg();

    vector<Vec4i> houghlines_;

    Pool<LaneLineNode> lanelinenodes_;

    Pool<DoubleLaneLineNode> double_lanelinenodes_;
    void CaptureDoubleLaneLineNodes();

    Pool<LaneLineFlow> lanelineflows_;
    Pool<TimeLaneLineFlow> time_lanelineflows_;
    bool if_estimate_lanelineflows_;
    void CaptureLaneLineFlows();
    bool EstimateLaneLineFlowsCheckIfCompleted(float score_at_lease);
    void EstimateLaneLineFlowsCompleted(float score_at_lease);
    void EstimateLaneLineFlows();

    Pool<Lane> lanes_;
    bool if_estimate_lane_;
    bool if_track_lane_;
    void ChangeLaneOrder();
    void UpdateLaneLineEquation(LaneLine *ll_ptr);
    void UpdateLaneEquation(Lane *lane_ptr);
    void EstimateLane();
    bool LaneLineAppendLaneLineNode(LaneLine *ll_ptr, LaneLineNode *lln_ptr);
    void LaneAppendLaneLineNode(Lane *lane_ptr, LaneLineNode *lln_ptr);
    void TrackLaneNormal();
    //车辆换道过程 道路跟踪
    void TrackLaneChanging();
    void TrackLane();
};

#endif // VLD_BASIC_H_
