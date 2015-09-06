#include "basic.h"

void VisualSystem::EstimateLane() {
    int i, j, index;
    int distance1, distance2;

    Lane *lane_ptr;
    PoolNode<Lane> *lane_itr;
    Lane *new_lane_ptr;

    LaneLineFlow *llf_ptr;
    PoolNode<LaneLineFlow> *llf_itr;

    LaneLine *ll_ptr;
    LaneLineNode *ll_lln_ptr;
    PoolNode<LaneLineNode> *ll_lln_itr;

    lanes_.Clear();

    //初始化 Lane left_ll
    PoolIteratorBegin(lanelineflows_, llf_ptr, llf_itr);
    for (i = 0; i < lanelineflows_.sum_; i++, PoolIteratorLoop(llf_ptr, llf_itr)) {
        index = -1;

        PoolIteratorBegin(lanes_, lane_ptr, lane_itr);
        for (j = 0; j < lanes_.sum_; j++, PoolIteratorLoop(lane_ptr, lane_itr)) {
            if (abs(llf_ptr->top_point_x - lane_ptr->left_ll.top_point_x) < 10) {
                if (llf_ptr->nodes.size() > lane_ptr->left_ll.nodes.sum_) {
                    lane_ptr->left_ll.Overlay(llf_ptr);
                }
                index = j;
            }
        }

        if (-1 == index) {
            new_lane_ptr = lanes_.new_node_v();
            new_lane_ptr->left_ll.Overlay(llf_ptr);
        }
    }

    ChangeLaneOrder();

    //生成 Lane right_ll
    PoolIteratorBegin(lanes_, lane_ptr, lane_itr);
    for (i = 0; i < lanes_.sum_; i++, PoolIteratorLoop(lane_ptr, lane_itr)) {

        PoolIteratorBegin(lanelineflows_, llf_ptr, llf_itr);
        for (j = 0; j < lanelineflows_.sum_; j++, PoolIteratorLoop(llf_ptr, llf_itr)) {
            if (llf_ptr->top_point_x <= lane_ptr->left_ll.top_point_x) {
                continue;
            }

            distance1 = llf_ptr->top_point_x - lane_ptr->left_ll.top_point_x;
            distance2 = distance1 / lane_width_;
            distance1 = distance1 % lane_width_;

            if (distance2 > 1) {
                continue;
            }

            if (0 == distance2 && distance1 < lane_width_max_) {
                continue;
            }

            if (1 == distance2 && distance1 > lane_width_min_) {
                continue;
            }

            if (0 != lane_ptr->right_ll.score &&
                    llf_ptr->nodes.size() < lane_ptr->right_ll.nodes.sum_) {
                continue;
            }

            lane_ptr->right_ll.Overlay(llf_ptr);
            break;
        }
    }

    //去除无效 Lane，并重新初始化得分
    PoolIteratorBegin(lanes_, lane_ptr, lane_itr);
    for (i = 0; i < lanes_.sum_; i++) {
        if (0 == lane_ptr->left_ll.score ||
                0 == lane_ptr->right_ll.score) {
            PoolDelEntryInLoop(lanes_, lane_ptr, lane_itr, i);
            continue;
        }

        lane_ptr->score = 10;
        lane_ptr->left_ll.score = 10;
        lane_ptr->right_ll.score = 10;

        ll_ptr = &lane_ptr->left_ll;
        ll_lln_ptr = (ll_lln_itr = ll_ptr->nodes.front_)->v;
        for (j = 0; j < ll_ptr->nodes.sum_; j++, PoolIteratorLoop(ll_lln_ptr, ll_lln_itr)) {
            ll_lln_ptr->score = 10;
        }

        ll_ptr = &lane_ptr->right_ll;
        ll_lln_ptr = (ll_lln_itr = ll_ptr->nodes.front_)->v;
        for (j = 0; j < ll_ptr->nodes.sum_; j++, PoolIteratorLoop(ll_lln_ptr, ll_lln_itr)) {
            ll_lln_ptr->score = 10;
        }

        UpdateLaneEquation(lane_ptr);

        PoolIteratorLoop(lane_ptr, lane_itr);
    }


    if_estimate_lanelineflows_  = false;
    if_estimate_lane_           = false;
    if_track_lane_              = true;
}
