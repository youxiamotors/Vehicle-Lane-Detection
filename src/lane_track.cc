#include "basic.h"

bool VisualSystem::LaneLineAppendLaneLineNode(LaneLine *ll_ptr, LaneLineNode *lln_ptr) {
    int i, j, space_distance1, space_distance2;
    float theta, angle_distance, score;
    Point2i p1, p2;

    LaneLineNode *ll_lln_ptr;
    PoolNode<LaneLineNode> *ll_lln_itr;

    LaneLineNode *target_lln_ptr;
    PoolNode<LaneLineNode> *target_lln_itr;

    LaneLineNode *new_lln_ptr;
    PoolNode<LaneLineNode> *new_lln_itr;

    p1.y = lln_ptr->p1.y;
    p1.x = Polyval(ll_ptr->eq_constant_terms, p1.y);
    p2.y = lln_ptr->p2.y;
    p2.x = Polyval(ll_ptr->eq_constant_terms, p2.y);

    EqCalculateLineThetaBy2Points(theta, p1, p2);

    if (!AngleIfCodirectional(lln_ptr->theta, theta)) {
        return false;
    }

    space_distance1 = abs(p1.x - lln_ptr->p1.x);
    space_distance2 = abs(p2.x - lln_ptr->p2.x);
    if (space_distance1 > lane_width_min_ || space_distance2 > lane_width_min_) {
        return false;
    }

    AngleCalculateDistance(angle_distance, lln_ptr->theta, theta);
    score = (angle_distance*60 + (space_distance1 + space_distance2)) * 1.0;
    if (0 == score) {
        score = SCORE_MAX;
    } else {
        score = 600 / (angle_distance*400 + (space_distance1 + space_distance2));
        if (score > SCORE_MAX) score = SCORE_MAX;
    }

    PoolIteratorBegin(ll_ptr->nodes, ll_lln_ptr, ll_lln_itr);
    for (i = 0; i < ll_ptr->nodes.sum_; i++, PoolIteratorLoop(ll_lln_ptr, ll_lln_itr)) {
        if (abs(lln_ptr->p1.y - ll_lln_ptr->p1.y) < 20 ||
                abs(lln_ptr->p2.y - ll_lln_ptr->p2.y) < 20 || 
                (lln_ptr->p1.y > ll_lln_ptr->p1.y && lln_ptr->p2.y < ll_lln_ptr->p2.y)) {
            if (score > ll_lln_ptr->score) {
                *ll_lln_ptr = *lln_ptr;
                ll_lln_ptr->score = score;
                ll_ptr->ImproveScore(6);
            }
            return true;
        }
    }

    PoolIteratorBegin(ll_ptr->nodes, ll_lln_ptr, ll_lln_itr);
    for (i = 0; i < ll_ptr->nodes.sum_; i++, PoolIteratorLoop(ll_lln_ptr, ll_lln_itr)) {
        if (lln_ptr->p2.y < ll_lln_ptr->p2.y) {
            target_lln_ptr = (target_lln_itr = ll_lln_itr)->v;
            for (j = i; j < ll_ptr->nodes.sum_; j++, PoolIteratorLoop(ll_lln_ptr, ll_lln_itr)) {
                if (lln_ptr->p2.y > ll_lln_ptr->p2.y) {
                    break;
                }
                target_lln_ptr = (target_lln_itr = ll_lln_itr)->v;
            }

            new_lln_ptr = (new_lln_itr = ll_ptr->nodes.new_node())->v;
            *new_lln_ptr = *lln_ptr;
            new_lln_ptr->score = score;
            ll_ptr->ImproveScore(6);
            ll_ptr->nodes.insert_after(new_lln_itr, target_lln_itr);
            return true;
        }

        if (lln_ptr->p1.y > ll_lln_ptr->p1.y) {
            target_lln_ptr = (target_lln_itr = ll_lln_itr)->v;
            for (j = i; j < ll_ptr->nodes.sum_; j++, PoolIteratorLoop(ll_lln_ptr, ll_lln_itr)) {
                if (lln_ptr->p1.y < ll_lln_ptr->p1.y) {
                    break;
                }
                target_lln_ptr = (target_lln_itr = ll_lln_itr)->v;
            }

            new_lln_ptr = (new_lln_itr = ll_ptr->nodes.new_node())->v;
            *new_lln_ptr = *lln_ptr;
            new_lln_ptr->score = score;
            ll_ptr->ImproveScore(6);
            ll_ptr->nodes.insert_before(new_lln_itr, target_lln_itr);
            return true;
        }
    }

    return false;
}

void VisualSystem::LaneAppendLaneLineNode(Lane *lane_ptr, LaneLineNode *lln_ptr) {
    if (LaneLineAppendLaneLineNode(&lane_ptr->left_ll, lln_ptr)) {
        lane_ptr->ImproveScore(6);
    }

    if (LaneLineAppendLaneLineNode(&lane_ptr->right_ll, lln_ptr)) {
        lane_ptr->ImproveScore(6);
    }
}

void VisualSystem::TrackLaneNormal() {
    int i, j;

    Lane *lane_ptr;
    PoolNode<Lane> *lane_itr;

    DoubleLaneLineNode *dlln_ptr;
    PoolNode<DoubleLaneLineNode> *dlln_itr;

    LaneLineNode *lln_ptr;
    PoolNode<LaneLineNode> *lln_itr;

    PoolIteratorBegin(lanes_, lane_ptr, lane_itr);
    for (i = 0; i < lanes_.sum_; i++, PoolIteratorLoop(lane_ptr, lane_itr)) {
        //从 double_lanelinenodes_ 中获取 LaneLineNode 
        PoolIteratorBegin(double_lanelinenodes_, dlln_ptr, dlln_itr);
        for (j = 0; j < double_lanelinenodes_.sum_; j++, PoolIteratorLoop(dlln_ptr, dlln_itr)) {
            LaneAppendLaneLineNode(lane_ptr, &dlln_ptr->lln);
        }

        //从 lanelinenodes_ 中获取 LaneLineNode
        PoolIteratorBegin(lanelinenodes_, lln_ptr, lln_itr);
        for (j = 0; j < lanelinenodes_.sum_; j++, PoolIteratorLoop(lln_ptr, lln_itr)) {
            LaneAppendLaneLineNode(lane_ptr, lln_ptr);
        }
    }
}

void VisualSystem::TrackLaneChanging() {
}

void VisualSystem::TrackLane() {
    int i, j;

    LaneLine *ll_ptr;
    LaneLineNode *ll_lln_ptr;
    PoolNode<LaneLineNode> *ll_lln_itr;

    Lane *lane_ptr;
    PoolNode<Lane> *lane_itr;

    //减少得分
    PoolIteratorBegin(lanes_, lane_ptr, lane_itr);
    for (i = 0; i < lanes_.sum_; i++, PoolIteratorLoop(lane_ptr, lane_itr)) {
        lane_ptr->score -= 6;
        
        ll_ptr = &lane_ptr->left_ll;
        ll_lln_ptr = (ll_lln_itr = ll_ptr->nodes.front_)->v;
        for (j = 0; j < ll_ptr->nodes.sum_; j++, PoolIteratorLoop(ll_lln_ptr, ll_lln_itr)) {
            ll_lln_ptr->score -= 6;
        }
        
        ll_ptr = &lane_ptr->right_ll;
        ll_lln_ptr = (ll_lln_itr = ll_ptr->nodes.front_)->v;
        for (j = 0; j < ll_ptr->nodes.sum_; j++, PoolIteratorLoop(ll_lln_ptr, ll_lln_itr)) {
            ll_lln_ptr->score -= 6;
        }
    }

    TrackLaneNormal();

    //去掉无效 Lane, 无效 LaneLineNode
    PoolIteratorBegin(lanes_, lane_ptr, lane_itr);
    for (i = 0; i < lanes_.sum_; i++) {
        if (lane_ptr->score <= 0) {
            PoolDelEntryInLoop(lanes_, lane_ptr, lane_itr, i);
            continue;
        }
 
        ll_ptr = &lane_ptr->left_ll;
        ll_lln_ptr = (ll_lln_itr = ll_ptr->nodes.front_)->v;
        for (j = 0; j < ll_ptr->nodes.sum_; j++) {
            if (ll_lln_ptr->score <= 0) {
                PoolDelEntryInLoop(ll_ptr->nodes, ll_lln_ptr, ll_lln_itr, j);
                continue;
            }
            PoolIteratorLoop(ll_lln_ptr, ll_lln_itr);
        }
  
        ll_ptr = &lane_ptr->right_ll;
        ll_lln_itr = ll_ptr->nodes.front_;
        ll_lln_ptr = ll_lln_itr->v;
        for (j = 0; j < ll_ptr->nodes.sum_; j++) {
            if (ll_lln_ptr->score <= 0) {
                PoolDelEntryInLoop(ll_ptr->nodes, ll_lln_ptr, ll_lln_itr, j);
                continue;
            }
            PoolIteratorLoop(ll_lln_ptr, ll_lln_itr);
        }

        UpdateLaneEquation(lane_ptr);

        PoolIteratorLoop(lane_ptr, lane_itr);
    }

    //找不到道路了
    if (0 == lanes_.sum_) {
        if_estimate_lanelineflows_  = true;
        if_estimate_lane_           = false;
        if_track_lane_              = false;
        return;
    }

}
