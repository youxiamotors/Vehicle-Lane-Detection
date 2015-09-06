#include "basic.h"

void VisualSystem::Init() {
    *capture_ >> img_src_;

    kImgWidth = (int)static_cast<int>(capture_->get(CV_CAP_PROP_FRAME_WIDTH));
    kImgHeight= (int)static_cast<int>(capture_->get(CV_CAP_PROP_FRAME_HEIGHT));
    kRoiStartY = kImgHeight * ROI_Y_RATIO;
    kRoiHeight = kImgHeight * ROI_HEIGHT_RATIO;
    kIPMStartX = kImgWidth * IPM_STARTX_RATIO;
    kIPMEndX = kImgWidth - kIPMStartX;
    kHotLaneAreaStartX = kIPMStartX + 6;
    kHotLaneAreaEndX = kIPMEndX - 6;

    lane_width_ = LANE_WIDTH;
    lane_width_max_ = lane_width_ - 14;
    lane_width_min_ = 14;

    lane_theta_ = 0;
    lane_theta_score_ = 0.0;

    if_estimate_lanelineflows_  = true;
    if_estimate_lane_           = false;
    if_track_lane_              = false;

    cvtColor(img_src_, img_gray_, CV_BGR2GRAY);

    img_roi_ = img_gray_(
            Rect(0, kRoiStartY, kImgWidth, kRoiHeight));

    img_cavas_ = Mat(kRoiHeight, kImgWidth, CV_8UC(1));

    perspective_.Init();
    gaussian_filter_.Init(4, 25);
    tophat_.Init();
}

void VisualSystem::LoopImg() {
    *capture_ >> img_src_;

    cvtColor(img_src_, img_gray_, CV_BGR2GRAY);

    perspective_.IPM(img_roi_, img_roi_);

    blur(img_roi_, img_roi_, Size(3,3));
    /*
    tophat_.Transform(img_roi_, img_roi_);
    threshold(img_roi_, img_roi_, 30, 255, CV_THRESH_BINARY);  
    */
    /*
    threshold(img_roi_, img_roi_, 130, 255, CV_THRESH_BINARY);  
    */

    img_roi_.copyTo(img_ret_);
    Canny(img_ret_, img_ret_, CANNY_THRESHOLD, CANNY_THRESHOLD*3, 3);
    //gaussian_filter_.Filter(img_ret_, img_ret_);
    blur(img_ret_, img_ret_, Size(3,3));

    img_cavas_.setTo(Scalar(0));

    CaptureDoubleLaneLineNodes();
    CaptureLaneLineFlows();

    if (if_estimate_lanelineflows_) {
        EstimateLaneLineFlows();
    }

    if (if_estimate_lane_) {
        EstimateLane();
    }

    if (if_track_lane_) {
        TrackLane();
    }

    int i, j;
    Vec4i l;
    LaneLineNode *lln_ptr;
    PoolNode<LaneLineNode> *lln_itr;

    LaneLineFlow *llf_ptr;
    PoolNode<LaneLineFlow> *llf_itr;

    DoubleLaneLineNode *dlln_ptr;
    PoolNode<DoubleLaneLineNode> *dlln_itr;

    LaneLine *ll_ptr;
    LaneLineNode *ll_lln_ptr;
    PoolNode<LaneLineNode> *ll_lln_itr;

    Lane *lane_ptr;
    PoolNode<Lane> *lane_itr;

    LOGI("%d %d %d\n", if_estimate_lanelineflows_, if_estimate_lane_, if_track_lane_);
    dlln_ptr = (dlln_itr = double_lanelinenodes_.front_)->v;
    for (i = 0; i < double_lanelinenodes_.sum_; i++) {
        continue;
        lln_ptr = &dlln_ptr->lln;
        line(img_ret_,
                Point(lln_ptr->p1.x, lln_ptr->p1.y),
                Point(lln_ptr->p2.x, lln_ptr->p2.y),
                Scalar(255), 1, CV_AA);

        dlln_ptr = (dlln_itr = dlln_itr->next)->v;
    }
    //return;

    lln_ptr = (lln_itr = lanelinenodes_.front_)->v;
    for (i = 0; i < lanelinenodes_.sum_; i++, lln_ptr = (lln_itr = lln_itr->next)->v) {
        line(img_ret_,
                Point(lln_ptr->p1.x, lln_ptr->p1.y),
                Point(lln_ptr->p2.x, lln_ptr->p2.y),
                Scalar(255), 3, CV_AA);
    }
    //return;

    lane_ptr = (lane_itr = lanes_.front_)->v;
    for (i = 0; i < lanes_.sum_; i++, lane_ptr = (lane_itr = lane_itr->next)->v) {
        Polyval(lane_ptr->left_ll.eq_constant_terms, lane_ptr->left_ll.points_y, lane_ptr->left_ll.points_x);
        for (j = 0; j < lane_ptr->left_ll.points_x.rows; j++) {
            circle(img_cavas_, 
                    Point((int)lane_ptr->left_ll.points_x.ptr<double>(j)[0],
                        (int)lane_ptr->left_ll.points_y.ptr<double>(j)[0]),
                    3 , Scalar(255), -1, 8 );
        }

        Polyval(lane_ptr->right_ll.eq_constant_terms, lane_ptr->right_ll.points_y, lane_ptr->right_ll.points_x);
        for (j = 0; j < lane_ptr->right_ll.points_x.rows; j++) {
            circle(img_cavas_, 
                    Point((int)lane_ptr->right_ll.points_x.ptr<double>(j)[0],
                        (int)lane_ptr->right_ll.points_y.ptr<double>(j)[0]),
                    3 , Scalar(255), -1, 8 );
        }
    }
    //return;

    lane_ptr = (lane_itr = lanes_.front_)->v;
    for (i = 0; i < lanes_.sum_; i++, lane_ptr = (lane_itr = lane_itr->next)->v) {
        //if (0 != i) continue;
        ll_ptr = &lane_ptr->left_ll;
        ll_lln_ptr = (ll_lln_itr = ll_ptr->nodes.front_)->v;
        for (j = 0; j < ll_ptr->nodes.sum_; j++, ll_lln_ptr = (ll_lln_itr = ll_lln_itr->next)->v) {
            line(img_cavas_,
                    Point(ll_lln_ptr->p1.x, ll_lln_ptr->p1.y),
                    Point(ll_lln_ptr->p2.x, ll_lln_ptr->p2.y),
                    Scalar(255), 1, CV_AA);
        }

        ll_ptr = &lane_ptr->right_ll;
        ll_lln_ptr = (ll_lln_itr = ll_ptr->nodes.front_)->v;
        for (j = 0; j < ll_ptr->nodes.sum_; j++, ll_lln_ptr = (ll_lln_itr = ll_lln_itr->next)->v) {
            line(img_cavas_,
                    Point(ll_lln_ptr->p1.x, ll_lln_ptr->p1.y),
                    Point(ll_lln_ptr->p2.x, ll_lln_ptr->p2.y),
                    Scalar(255), 1, CV_AA);
        }
    }
    return;

    lane_ptr = (lane_itr = lanes_.front_)->v;
    for (i = 0; i < lanes_.sum_; i++, lane_ptr = (lane_itr = lane_itr->next)->v) {
        //if (0 != i) continue;
        ll_ptr = &lane_ptr->left_ll;
        ll_lln_ptr = (ll_lln_itr = ll_ptr->nodes.front_)->v;
        for (j = 0; j < ll_ptr->nodes.sum_; j++, ll_lln_ptr = (ll_lln_itr = ll_lln_itr->next)->v) {
            line(img_cavas_,
                    Point(ll_lln_ptr->p1.x, ll_lln_ptr->p1.y),
                    Point(ll_lln_ptr->p2.x, ll_lln_ptr->p2.y),
                    Scalar(255), 2, CV_AA);
        }

        ll_ptr = &lane_ptr->right_ll;
        ll_lln_ptr = (ll_lln_itr = ll_ptr->nodes.front_)->v;
        for (j = 0; j < ll_ptr->nodes.sum_; j++, ll_lln_ptr = (ll_lln_itr = ll_lln_itr->next)->v) {
            line(img_cavas_,
                    Point(ll_lln_ptr->p1.x, ll_lln_ptr->p1.y),
                    Point(ll_lln_ptr->p2.x, ll_lln_ptr->p2.y),
                    Scalar(255), 1, CV_AA);
        }
    }
    return;

    llf_ptr = (llf_itr = lanelineflows_.front_)->v;
    for (i = 0; i < lanelineflows_.sum_; i++, llf_ptr = (llf_itr = llf_itr->next)->v) {
        for (j = 0; j < llf_ptr->nodes.size(); j++) {
            lln_ptr = llf_ptr->nodes.at(j);
            line(img_cavas_,
                    Point(lln_ptr->p1.x, lln_ptr->p1.y),
                    Point(lln_ptr->p2.x, lln_ptr->p2.y),
                    Scalar(255), j+1, CV_AA);
        }
    }
    return;


    line(img_ret_,
            Point(kIPMStartX, 30),
            Point(kIPMEndX, 30),
            Scalar(255), 8, CV_AA);
    return;
}
