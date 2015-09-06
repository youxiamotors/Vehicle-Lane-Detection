#include "basic.h"

void VisualSystem::ChangeLaneOrder() {
    int i, j;
    Lane *lane_ptr, *lane2_ptr;
    PoolNode<Lane> *lane_itr, *lane2_itr;

    PoolIteratorBegin(lanes_, lane_ptr, lane_itr);
    for (i = 0; i < lanes_.sum_-1; i++, PoolIteratorLoop(lane_ptr, lane_itr)) {

        lane2_ptr = (lane2_itr = lane_itr->next)->v;
        for(j = i+1; j < lanes_.sum_; j++, PoolIteratorLoop(lane2_ptr, lane2_itr)) {
            if (lane_ptr->left_ll.top_point_x >
                    lane2_ptr->left_ll.top_point_x) {
                lanes_.exchange_node(&lane_itr, &lane2_itr);
                lane_ptr = lane_itr->v;
                lane2_ptr = lane2_itr->v;
            }
        }
    }
}

void VisualSystem::UpdateLaneLineEquation(LaneLine *ll_ptr) {
    if (0 == ll_ptr->nodes.sum_) {
        return;
    }

    int i, size, x, y;
    LaneLineNode *lln_ptr;
    PoolNode<LaneLineNode> *lln_itr;

    size = ll_ptr->nodes.sum_ * 6;
    ll_ptr->points_x.resize(size);
    ll_ptr->points_y.resize(size);
    PoolIteratorBegin(ll_ptr->nodes, lln_ptr, lln_itr);
    for (i = 0; i < size; i+=6, PoolIteratorLoop(lln_ptr, lln_itr)) {
        ll_ptr->points_x.ptr<double>(i)[0] = lln_ptr->p1.x;
        ll_ptr->points_y.ptr<double>(i)[0] = lln_ptr->p1.y;

        ll_ptr->points_x.ptr<double>(i+1)[0] = lln_ptr->p2.x;
        ll_ptr->points_y.ptr<double>(i+1)[0] = lln_ptr->p2.y;

        y = lln_ptr->p2.y + (lln_ptr->p1.y - lln_ptr->p2.y) * 0.2;
        EqCalculateXByYKB(x, lln_ptr->p2.x, y, lln_ptr->k, lln_ptr->b);
        ll_ptr->points_x.ptr<double>(i+2)[0] = x;
        ll_ptr->points_y.ptr<double>(i+2)[0] = y;

        y = lln_ptr->p2.y + (lln_ptr->p1.y - lln_ptr->p2.y) * 0.4;
        EqCalculateXByYKB(x, lln_ptr->p2.x, y, lln_ptr->k, lln_ptr->b);
        ll_ptr->points_x.ptr<double>(i+3)[0] = x;
        ll_ptr->points_y.ptr<double>(i+3)[0] = y;

        y = lln_ptr->p2.y + (lln_ptr->p1.y - lln_ptr->p2.y) * 0.6;
        EqCalculateXByYKB(x, lln_ptr->p2.x, y, lln_ptr->k, lln_ptr->b);
        ll_ptr->points_x.ptr<double>(i+4)[0] = x;
        ll_ptr->points_y.ptr<double>(i+4)[0] = y;

        y = lln_ptr->p2.y + (lln_ptr->p1.y - lln_ptr->p2.y) * 0.8;
        EqCalculateXByYKB(x, lln_ptr->p2.x, y, lln_ptr->k, lln_ptr->b);
        ll_ptr->points_x.ptr<double>(i+5)[0] = x;
        ll_ptr->points_y.ptr<double>(i+5)[0] = y;
    }

    Polyfit(ll_ptr->points_y, ll_ptr->points_x, 
            ll_ptr->eq_constant_terms, LANE_EQUATION_CONSTANT_TERMS_SIZE);
}

void VisualSystem::UpdateLaneEquation(Lane *lane_ptr) {
    UpdateLaneLineEquation(&lane_ptr->left_ll);
    UpdateLaneLineEquation(&lane_ptr->right_ll);
}
