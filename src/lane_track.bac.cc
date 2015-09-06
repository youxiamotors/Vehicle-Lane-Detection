#include "basic.h"

void VisualSystem::TrackLaneNormal() {
    int i, j, k;
    bool if_conflict;
    float theta;
    float angle_distance;
    float score;
    int space_distance1, space_distance2;
    Point2i p1, p2;

    Lane *lane_ptr;
    PoolNode<Lane> *lane_itr;

    DoubleLaneLineNode *dlln_ptr;
    PoolNode<DoubleLaneLineNode> *dlln_itr;

    LaneLineNode *lln_ptr;
    PoolNode<LaneLineNode> *lln_itr;

    TimeLaneLineFlow *tllf_ptr;
    LaneLineNode *tllf_lln_ptr;
    PoolNode<LaneLineNode> *tllf_lln_itr;
    PoolNode<LaneLineNode> *new_tllf_lln_itr;

    //根据 double_lanelinenodes_ 调整 lanes_
    PoolIteratorBegin(double_lanelinenodes_, dlln_ptr, dlln_itr);
    for (i = 0; i < double_lanelinenodes_.sum_; i++, PoolIteratorLoop(dlln_ptr, dlln_itr)) {
        lln_ptr = &dlln_ptr->lln;

        PoolIteratorBegin(lanes_, lane_ptr, lane_itr);
        for (j = 0; j < lanes_.sum_; j++, PoolIteratorLoop(lane_ptr, lane_itr)) {

//LEFT_CANDIDATE_TLLFS
            p1.y = lln_ptr->p1.y;
            p1.x = Polyval(lane_ptr->left_eq_constant_terms, p1.y);
            p2.y = lln_ptr->p2.y;
            p2.x = Polyval(lane_ptr->left_eq_constant_terms, p2.y);

            EqCalculateLineThetaBy2Points(theta, p1, p2);

            AngleCalculateDistance(angle_distance, lln_ptr->theta, theta);
            if (angle_distance > ANGLE_CODIRECTIONAL_AT_LEASE) {
                goto RIGHT_CANDIDATE_TLLFS;
            }

            space_distance1 = abs(p1.x - lln_ptr->p1.x);
            space_distance2 = abs(p2.x - lln_ptr->p2.x);
            if (space_distance1 > lane_width_min_ || space_distance2 > lane_width_min_) {
                goto RIGHT_CANDIDATE_TLLFS;
            }

            score = 1 / (angle_distance*60 + (space_distance1 + space_distance2));

            tllf_ptr = &lane_ptr->left_candidate_tllfs;
            tllf_ptr = &lane_ptr->left_tllfs;
            PoolIteratorBegin(tllf_ptr->nodes, tllf_lln_ptr, tllf_lln_itr);
            if_conflict = false;
            for (k = 0; k < tllf_ptr->nodes.sum_; k++, PoolIteratorLoop(tllf_lln_ptr, tllf_lln_itr)) {
                if (abs(lln_ptr->p1.y - tllf_lln_ptr->p1.y) < 20 ||
                        abs(lln_ptr->p1.y - tllf_lln_ptr->p1.y) < 20) {
                    if_conflict = true;
                    if (score > tllf_lln_ptr->score) {
                        lane_ptr->ImproveScore(6, tllf_ptr, 6, tllf_lln_ptr, 6);
                        *tllf_lln_ptr = *lln_ptr;

                        new_tllf_lln_itr = tllf_ptr->nodes.new_node();
                        *new_tllf_lln_itr->v = *lln_ptr;
                        new_tllf_lln_itr->v->score = score;
                    } 
                    break;
                }
            }

            if (tllf_lln_itr == tllf_ptr->nodes.last_->next) {
                tllf_lln_ptr = (tllf_lln_itr = tllf_lln_itr->prev)->v;
            }

            if (false == if_conflict) {
                lane_ptr->ImproveScore(6, tllf_ptr, 6, tllf_lln_ptr, 6);
                new_tllf_lln_itr = tllf_ptr->nodes.new_node();
                *new_tllf_lln_itr->v = *lln_ptr;
                new_tllf_lln_itr->v->score = score;
                if (lln_ptr->p2.y - tllf_lln_ptr->p2.y < 0) {
                    tllf_ptr->nodes.insert_after(new_tllf_lln_itr, tllf_lln_itr);
                } else if(lln_ptr->p1.y - tllf_lln_ptr->p1.y > 0) {
                    tllf_ptr->nodes.insert_after(new_tllf_lln_itr, 0);
                }
                break;
            }

RIGHT_CANDIDATE_TLLFS:
            p1.y = lln_ptr->p1.y;
            p1.x = Polyval(lane_ptr->right_eq_constant_terms, p1.y);
            p2.y = lln_ptr->p2.y;
            p2.x = Polyval(lane_ptr->right_eq_constant_terms, p2.y);

            EqCalculateLineThetaBy2Points(theta, p1, p2);

            AngleCalculateDistance(angle_distance, lln_ptr->theta, theta);
            if (angle_distance > ANGLE_CODIRECTIONAL_AT_LEASE) {
                continue;
            }

            space_distance1 = abs(p1.x - lln_ptr->p1.x);
            space_distance2 = abs(p2.x - lln_ptr->p2.x);
            if (space_distance1 > lane_width_min_ || space_distance2 > lane_width_min_) {
                continue;
            }

            score = 1 / (angle_distance*60 + (space_distance1 + space_distance2));

            tllf_ptr = &lane_ptr->right_candidate_tllfs;
            tllf_ptr = &lane_ptr->right_tllfs;
            PoolIteratorBegin(tllf_ptr->nodes, tllf_lln_ptr, tllf_lln_itr);
            if_conflict = false;
            for (k = 0; k < tllf_ptr->nodes.sum_; k++, PoolIteratorLoop(tllf_lln_ptr, tllf_lln_itr)) {
                if (abs(lln_ptr->p1.y - tllf_lln_ptr->p1.y) < 20 ||
                        abs(lln_ptr->p1.y - tllf_lln_ptr->p1.y) < 20) {
                    if_conflict = true;
                    if (score > tllf_lln_ptr->score) {
                        lane_ptr->ImproveScore(6, tllf_ptr, 6, tllf_lln_ptr, 6);
                        *tllf_lln_ptr = *lln_ptr;

                        new_tllf_lln_itr = tllf_ptr->nodes.new_node();
                        *new_tllf_lln_itr->v = *lln_ptr;
                        new_tllf_lln_itr->v->score = score;
                    } 
                    break;
                }
            }

            if (tllf_lln_itr == tllf_ptr->nodes.last_->next) {
                tllf_lln_ptr = (tllf_lln_itr = tllf_lln_itr->prev)->v;
            }

            if (false == if_conflict) {
                lane_ptr->ImproveScore(6, tllf_ptr, 6, tllf_lln_ptr, 6);
                new_tllf_lln_itr = tllf_ptr->nodes.new_node();
                *new_tllf_lln_itr->v = *lln_ptr;
                new_tllf_lln_itr->v->score = score;
                if (lln_ptr->p2.y - tllf_lln_ptr->p2.y < 0) {
                    tllf_ptr->nodes.insert_after(new_tllf_lln_itr, tllf_lln_itr);
                } else if(lln_ptr->p1.y - tllf_lln_ptr->p1.y > 0) {
                    tllf_ptr->nodes.insert_after(new_tllf_lln_itr, 0);
                }
                break;
            }
        }
    }
    return;

    //根据 lanelineflows_ 调整 lanes_
    PoolIteratorBegin(lanelinenodes_, lln_ptr, lln_itr);
    for (i = 0; i < lanelinenodes_.sum_; i++, PoolIteratorLoop(lln_ptr, lln_itr)) {

        PoolIteratorBegin(lanes_, lane_ptr, lane_itr);
        for (j = 0; j < lanes_.sum_; j++, PoolIteratorLoop(lane_ptr, lane_itr)) {

//LEFT_CANDIDATE_TLLFS_LLN:
            AngleCalculateDistance(angle_distance, lln_ptr->theta, lane_theta_);
            if (angle_distance > ANGLE_CODIRECTIONAL_AT_LEASE) {
                goto RIGHT_CANDIDATE_TLLFS_LLN;
            }

            p1.y = lln_ptr->p1.y;
            p1.x = Polyval(lane_ptr->left_eq_constant_terms, p1.y);
            p2.y = lln_ptr->p2.y;
            p2.x = Polyval(lane_ptr->left_eq_constant_terms, p2.y);

            AngleCalculateDistance(angle_distance, lln_ptr->theta, lane_theta_);
            if (angle_distance > ANGLE_PARALLEL_AT_LEASE) {
                goto RIGHT_CANDIDATE_TLLFS_LLN;
            }

            space_distance1 = abs(p1.x - lln_ptr->p1.x);
            space_distance2 = abs(p2.x - lln_ptr->p2.x);
            if (space_distance1 > 6 || space_distance2 > 6) {
                goto RIGHT_CANDIDATE_TLLFS_LLN;
            }

            score = 1 / (angle_distance*60 + (space_distance1 + space_distance2));

            tllf_ptr = &lane_ptr->left_candidate_tllfs;
            tllf_ptr = &lane_ptr->left_tllfs;
            PoolIteratorBegin(tllf_ptr->nodes, tllf_lln_ptr, tllf_lln_itr);
            if_conflict = false;
            for (k = 0; k < tllf_ptr->nodes.sum_; k++, PoolIteratorLoop(tllf_lln_ptr, tllf_lln_itr)) {
                if (abs(lln_ptr->p1.y - tllf_lln_ptr->p1.y) < 20 ||
                        abs(lln_ptr->p1.y - tllf_lln_ptr->p1.y) < 20) {
                    if_conflict = true;
                    if (score > tllf_lln_ptr->score) {
                        lane_ptr->ImproveScore(6, tllf_ptr, 6, tllf_lln_ptr, 6);
                        *tllf_lln_ptr = *lln_ptr;

                        new_tllf_lln_itr = tllf_ptr->nodes.new_node();
                        *new_tllf_lln_itr->v = *lln_ptr;
                        new_tllf_lln_itr->v->score = score;
                    } 
                    break;
                }
            }

            if (tllf_lln_itr == tllf_ptr->nodes.last_->next) {
                tllf_lln_ptr = (tllf_lln_itr = tllf_lln_itr->prev)->v;
            }

            if (false == if_conflict) {
                lane_ptr->ImproveScore(6, tllf_ptr, 6, tllf_lln_ptr, 6);
                new_tllf_lln_itr = tllf_ptr->nodes.new_node();
                *new_tllf_lln_itr->v = *lln_ptr;
                new_tllf_lln_itr->v->score = score;
                if (lln_ptr->p2.y - tllf_lln_ptr->p2.y < 0) {
                    tllf_ptr->nodes.insert_after(new_tllf_lln_itr, tllf_lln_itr);
                } else if(lln_ptr->p1.y - tllf_lln_ptr->p1.y > 0) {
                    tllf_ptr->nodes.insert_after(new_tllf_lln_itr, 0);
                }
                break;
            }

RIGHT_CANDIDATE_TLLFS_LLN:
            AngleCalculateDistance(angle_distance, lln_ptr->theta, lane_theta_);
            if (angle_distance > ANGLE_CODIRECTIONAL_AT_LEASE) {
                continue;
            }

            p1.y = lln_ptr->p1.y;
            p1.x = Polyval(lane_ptr->right_eq_constant_terms, p1.y);
            p2.y = lln_ptr->p2.y;
            p2.x = Polyval(lane_ptr->right_eq_constant_terms, p2.y);

            AngleCalculateDistance(angle_distance, lln_ptr->theta, lane_theta_);
            if (angle_distance > ANGLE_PARALLEL_AT_LEASE) {
                continue;
            }

            space_distance1 = abs(p1.x - lln_ptr->p1.x);
            space_distance2 = abs(p2.x - lln_ptr->p2.x);
            if (space_distance1 > 6 || space_distance2 > 6) {
                continue;
            }

            score = 1 / (angle_distance*60 + (space_distance1 + space_distance2));

            tllf_ptr = &lane_ptr->right_candidate_tllfs;
            tllf_ptr = &lane_ptr->right_tllfs;
            PoolIteratorBegin(tllf_ptr->nodes, tllf_lln_ptr, tllf_lln_itr);
            if_conflict = false;
            for (k = 0; k < tllf_ptr->nodes.sum_; k++, PoolIteratorLoop(tllf_lln_ptr, tllf_lln_itr)) {
                if (abs(lln_ptr->p1.y - tllf_lln_ptr->p1.y) < 20 ||
                        abs(lln_ptr->p1.y - tllf_lln_ptr->p1.y) < 20) {
                    if_conflict = true;
                    if (score > tllf_lln_ptr->score) {
                        lane_ptr->ImproveScore(6, tllf_ptr, 6, tllf_lln_ptr, 6);
                        *tllf_lln_ptr = *lln_ptr;

                        new_tllf_lln_itr = tllf_ptr->nodes.new_node();
                        *new_tllf_lln_itr->v = *lln_ptr;
                        new_tllf_lln_itr->v->score = score;
                    } 
                    break;
                }
            }

            if (tllf_lln_itr == tllf_ptr->nodes.last_->next) {
                tllf_lln_ptr = (tllf_lln_itr = tllf_lln_itr->prev)->v;
            }

            if (false == if_conflict) {
                lane_ptr->ImproveScore(6, tllf_ptr, 6, tllf_lln_ptr, 6);
                new_tllf_lln_itr = tllf_ptr->nodes.new_node();
                *new_tllf_lln_itr->v = *lln_ptr;
                new_tllf_lln_itr->v->score = score;
                if (lln_ptr->p2.y - tllf_lln_ptr->p2.y < 0) {
                    tllf_ptr->nodes.insert_after(new_tllf_lln_itr, tllf_lln_itr);
                } else if(lln_ptr->p1.y - tllf_lln_ptr->p1.y > 0) {
                    tllf_ptr->nodes.insert_after(new_tllf_lln_itr, 0);
                }
                break;
            }
        }
    }
}

void VisualSystem::TrackLaneChanging() {
}

void VisualSystem::TrackLane() {
    int i, j;

    TimeLaneLineFlow *tllf_ptr;
    LaneLineNode *tllf_lln_ptr;
    PoolNode<LaneLineNode> *tllf_lln_itr;

    Lane *lane_ptr;
    PoolNode<Lane> *lane_itr;

    //减少得分
    PoolIteratorBegin(lanes_, lane_ptr, lane_itr);
    for (i = 0; i < lanes_.sum_; i++, PoolIteratorLoop(lane_ptr, lane_itr)) {
        lane_ptr->score -= 6;
        
        tllf_ptr = &lane_ptr->left_tllfs;
        tllf_lln_ptr = (tllf_lln_itr = tllf_ptr->nodes.front_)->v;
        for (j = 0; j < tllf_ptr->nodes.sum_; j++, PoolIteratorLoop(tllf_lln_ptr, tllf_lln_itr)) {
            tllf_lln_ptr->score -= 6;
        }
        lane_ptr->left_candidate_tllfs.Clear();
        
        tllf_ptr = &lane_ptr->right_tllfs;
        tllf_lln_ptr = (tllf_lln_itr = tllf_ptr->nodes.front_)->v;
        for (j = 0; j < tllf_ptr->nodes.sum_; j++, PoolIteratorLoop(tllf_lln_ptr, tllf_lln_itr)) {
            tllf_lln_ptr->score -= 6;
        }
        lane_ptr->right_candidate_tllfs.Clear();
    }

    TrackLaneNormal();

    //去掉无效 Lane, 无效 LaneLineNode
    PoolIteratorBegin(lanes_, lane_ptr, lane_itr);
    for (i = 0; i < lanes_.sum_; i++) {
        if (lane_ptr->score <= 0) {
            PoolDelEntryInLoop(lanes_, lane_ptr, lane_itr, i);
            continue;
        }
 
        tllf_ptr = &lane_ptr->left_tllfs;
        tllf_lln_ptr = (tllf_lln_itr = tllf_ptr->nodes.front_)->v;
        for (j = 0; j < tllf_ptr->nodes.sum_; j++) {
            if (tllf_lln_ptr->score <= 0) {
                PoolDelEntryInLoop(tllf_ptr->nodes, tllf_lln_ptr, tllf_lln_itr, j);
                continue;
            }
            PoolIteratorLoop(tllf_lln_ptr, tllf_lln_itr);
        }
  
        tllf_ptr = &lane_ptr->right_tllfs;
        tllf_lln_itr = tllf_ptr->nodes.front_;
        tllf_lln_ptr = tllf_lln_itr->v;
        for (j = 0; j < tllf_ptr->nodes.sum_; j++) {
            if (tllf_lln_ptr->score <= 0) {
                PoolDelEntryInLoop(tllf_ptr->nodes, tllf_lln_ptr, tllf_lln_itr, j);
                continue;
            }
            PoolIteratorLoop(tllf_lln_ptr, tllf_lln_itr);
        }

        PoolIteratorLoop(lane_ptr, lane_itr);
    }

    //找不到道路了
    if (0 == lanes_.sum_) {
    }

    UpdateLaneEquation();
}
