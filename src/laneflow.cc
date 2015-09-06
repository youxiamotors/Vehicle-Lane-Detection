#include "basic.h"

bool VisualSystem::EstimateLaneLineFlowsCheckIfCompleted(float score_at_lease) {
    int i, index;
    float angle_distance;

    LaneLineFlow *llf_ptr = 0, *llf2_ptr = 0;
    PoolNode<LaneLineFlow> *llf_itr = 0, *llf2_itr = 0;

    TimeLaneLineFlow *tllf_ptr = 0; 
    PoolNode<TimeLaneLineFlow> *tllf_itr = 0;

    index = -1;
    PoolIteratorBegin(lanelineflows_, llf_ptr, llf_itr);
    for (i = 0; i < lanelineflows_.sum_; i++, PoolIteratorLoop(llf_ptr, llf_itr)) {
        if (llf_ptr->score < score_at_lease) {
            continue;
        }
        index = i;
        break;
    }

    if (-1 == index) return false;

    llf2_ptr = (llf2_itr = llf_itr->next)->v;
    for (i++; i < lanelineflows_.sum_; i++, PoolIteratorLoop(llf2_ptr, llf2_itr)) {
        if (llf2_ptr->score < score_at_lease) {
            continue;
        }

        AngleCalculateDistance(angle_distance, 
                llf_ptr->theta,
                llf2_ptr->theta);
        if (angle_distance > ANGLE_PARALLEL_AT_LEASE) {
            return false;        
        }
    }

    if (0 == llf2_ptr) return false;

    PoolIteratorBegin(time_lanelineflows_, tllf_ptr, tllf_itr);
    for (i = 0; i < time_lanelineflows_.sum_; i++, PoolIteratorLoop(tllf_ptr, tllf_itr)) {
        if (tllf_ptr->score < score_at_lease) {
            continue;
        }

        AngleCalculateDistance(angle_distance, 
                llf_ptr->theta,
                tllf_ptr->theta);
        if (angle_distance > ANGLE_PARALLEL_AT_LEASE) {
            return false;        
        }
    }

    index = 0;
    lane_theta_ = 0;
    PoolIteratorBegin(lanelineflows_, llf_ptr, llf_itr);
    for (i = 0; i < lanelineflows_.sum_; i++, PoolIteratorLoop(llf_ptr, llf_itr)) {
        if (llf_ptr->score < score_at_lease) {
            continue;
        }

        index++;
        lane_theta_ += llf_ptr->theta;
    }
    lane_theta_ = lane_theta_ / index;
    lane_theta_score_ = 1.0;

    return true;
}

void VisualSystem::EstimateLaneLineFlowsCompleted(float score_at_lease) {
    int i;
    int space_distance;
    float angle_distance;

    LaneLineFlow *llf_ptr, *llf2_ptr;
    PoolNode<LaneLineFlow> *llf_itr;
    PoolNode<LaneLineFlow> *llf2_itr;

    PoolIteratorBegin(lanelineflows_, llf_ptr, llf_itr);
    for (i = 0; i < lanelineflows_.sum_; i++, PoolIteratorLoop(llf_ptr, llf_itr)) {
        if (llf_ptr->score > score_at_lease) {
            break;
        }
    }

    //增加 或 减少 LaneLineFlow 
    PoolIteratorBegin(lanelineflows_, llf2_ptr, llf2_itr);
    for (i = 0; i < lanelineflows_.sum_; i++, PoolIteratorLoop(llf2_ptr, llf2_itr)) {

        if (!AngleIfCodirectional(llf2_ptr->theta, lane_theta_)) {
            llf2_ptr->score = 0;
            continue;
        }


        if (llf2_ptr->score >= score_at_lease) {
            continue;
        }

        AngleCalculateDistance(angle_distance, llf2_ptr->theta, lane_theta_);
        if (angle_distance > ANGLE_PARALLEL_AT_LEASE) {
            continue;
        }

        space_distance = abs(llf2_ptr->top_point_x - llf_ptr->top_point_x) % lane_width_;
        if (space_distance > lane_width_min_ && space_distance < lane_width_max_) {
            continue;
        }

        if (0 == (llf2_ptr->top_point_x - llf_ptr->top_point_x) / lane_width_ && 
                space_distance < lane_width_-8) {
            continue;
        }

        llf2_ptr->score = score_at_lease;
    }

    PoolIteratorBegin(lanelineflows_, llf2_ptr, llf2_itr);
    for (i = 0; i < lanelineflows_.sum_; i++) {
        if (llf2_ptr->score < score_at_lease) {
            PoolDelEntryInLoop(lanelineflows_, llf2_ptr, llf2_itr, i);
            continue;
        }
        PoolIteratorLoop(llf2_ptr, llf2_itr);
    }

    time_lanelineflows_.Clear();
    if_estimate_lanelineflows_  = false;
    if_estimate_lane_           = true;
    if_track_lane_              = false;
}

void VisualSystem::CaptureLaneLineFlows() {
    int i, j, k, size3;
    int x;
    int space_distance;
    float angle_distance;
    float distance, distance_extreme;

    DoubleLaneLineNode *dlln_ptr;
    PoolNode<DoubleLaneLineNode>  *dlln_itr;

    LaneLineNode *lln_ptr;
    LaneLineFlow *llf_ptr, *target_llf_ptr;
    PoolNode<LaneLineFlow> *llf_itr;

    LaneLineFlow *new_llf_ptr;
    LaneLineNode *llf_lln_ptr;

    lanelineflows_.Clear();

    //生成 lanelineflows_
    PoolIteratorBegin(double_lanelinenodes_, dlln_ptr, dlln_itr);
    for (i = 0; i < double_lanelinenodes_.sum_; i++, PoolIteratorLoop(dlln_ptr, dlln_itr)) {
        lln_ptr = &dlln_ptr->lln;

        target_llf_ptr = 0;
        distance_extreme = 999999999;
        PoolIteratorBegin(lanelineflows_, llf_ptr, llf_itr);
        for (j = 0; j < lanelineflows_.sum_; j++, PoolIteratorLoop(llf_ptr, llf_itr)) {
            llf_lln_ptr = llf_ptr->nodes.back();

            if (lln_ptr->p2.y >= llf_lln_ptr->p2.y) {
                continue;
            }

            if (lln_ptr->p1.y >= llf_lln_ptr->p1.y) {
                continue;
            }

            if (!AngleIfCodirectional(lln_ptr->theta, llf_lln_ptr->theta)) {
                continue;
            }

            EqCalculateXByYKB(x, llf_lln_ptr->p2.x, lln_ptr->p1.y, llf_lln_ptr->k, llf_lln_ptr->b);
            space_distance = abs(x - lln_ptr->p1.x);
            if (space_distance > 10) {
                continue;
            }

            AngleCalculateDistance(angle_distance, lln_ptr->theta, llf_lln_ptr->theta);
            distance = angle_distance*1800 + space_distance*0.01;

            if (distance > distance_extreme) {
                continue;
            }

            distance_extreme = distance;
            target_llf_ptr = llf_ptr;
        }

        if (0 != target_llf_ptr) {
            target_llf_ptr->nodes.push_back(lln_ptr);
            target_llf_ptr->bottom_point_y = lln_ptr->p2.y;
            continue;
        }

        new_llf_ptr = lanelineflows_.new_node_v();
        new_llf_ptr->nodes.push_back(lln_ptr);
        EqCalculateXByYKB(new_llf_ptr->top_point_x, lln_ptr->p1.x, kRoiHeight, lln_ptr->k, lln_ptr->b);
        new_llf_ptr->bottom_point_y = lln_ptr->p2.y;
        new_llf_ptr->score = 0;
    }

    PoolIteratorBegin(lanelineflows_, llf_ptr, llf_itr);
    for (j = 0; j < lanelineflows_.sum_; j++, PoolIteratorLoop(llf_ptr, llf_itr)) {
        angle_distance = 0;
        size3 = llf_ptr->nodes.size();
        for (k = 0; k < size3; k++) {
            llf_lln_ptr = llf_ptr->nodes.at(k);

            angle_distance += llf_lln_ptr->theta;
        }
        llf_ptr->theta = angle_distance / size3;
    }
}

void VisualSystem::EstimateLaneLineFlows() {
    CaptureLaneLineFlows();

    int i, j, index;
    int space_distance;
    float angle_distance, scoreincrement;

    LaneLineFlow *llf_ptr, *llf2_ptr;
    PoolNode<LaneLineFlow> *llf_itr, *llf2_itr;

    LaneLineNode *llf_lln_ptr;
    TimeLaneLineFlow *tllf_ptr;
    PoolNode<TimeLaneLineFlow> *tllf_itr;

    LaneLineNode *tllf_lln_ptr;
    TimeLaneLineFlow *new_tllf_ptr;

    //根据车道线平行 计算得分
    PoolIteratorBegin(lanelineflows_, llf_ptr, llf_itr);
    for (i = 0; i < lanelineflows_.sum_; i++, PoolIteratorLoop(llf_ptr, llf_itr)) {

        llf2_ptr = (llf2_itr = llf_itr->next)->v;
        for (j = i+1; j < lanelineflows_.sum_; j++, PoolIteratorLoop(llf2_ptr, llf2_itr)) {
            space_distance = abs(llf_ptr->top_point_x - llf2_ptr->top_point_x) % lane_width_;
            if (space_distance > 8 && space_distance < lane_width_-8) {
                continue;
            }

            if (abs(llf_ptr->top_point_x - llf2_ptr->top_point_x) / lane_width_ > 1) {
                continue;
            }

            AngleCalculateDistance(angle_distance, 
                    llf_ptr->nodes.front()->theta, llf2_ptr->nodes.front()->theta);
            if (angle_distance > 0.08)  {
                continue;
            }

            llf_ptr->score++;
            llf2_ptr->score++;
        }
    }

    if (EstimateLaneLineFlowsCheckIfCompleted(2)) {
        EstimateLaneLineFlowsCompleted(2);
        return;
    }

    //根据车道线在车辆正前方 计算得分
    PoolIteratorBegin(lanelineflows_, llf_ptr, llf_itr);
    for (i = 0; i < lanelineflows_.sum_; i++, PoolIteratorLoop(llf_ptr, llf_itr)) {
        llf_lln_ptr = llf_ptr->nodes.front();

        scoreincrement = 0;

        if (llf_lln_ptr->p1.x < kIPMStartX) {
            scoreincrement = (llf_lln_ptr->p1.x - kIPMStartX) * 4;
        } else if (llf_lln_ptr->p1.x > kIPMEndX) {
            scoreincrement = (kIPMEndX - llf_lln_ptr->p1.x) * 4;
        } else {
            space_distance = kRoiHeight - llf_lln_ptr->p1.y;
            if (space_distance > 60) {
                scoreincrement =  -1 * space_distance * 4;

            } else {
                scoreincrement += 4;

                if (0 == space_distance) {
                    scoreincrement += 12;
                } else {
                    scoreincrement += 12 - 6 * (1 - llf_lln_ptr->p1.y / kRoiHeight);
                }

                if (llf_lln_ptr->p1.x > kHotLaneAreaStartX && llf_lln_ptr->p1.x < kHotLaneAreaEndX) {
                    scoreincrement += 6;
                }
            }
        }

        llf_ptr->score += scoreincrement;
    }

    if (EstimateLaneLineFlowsCheckIfCompleted(20)) {
        EstimateLaneLineFlowsCompleted(20);
        return;
    }

    //根据车道线历史检测，积累得分
    PoolIteratorBegin(time_lanelineflows_, tllf_ptr, tllf_itr);
    for (i = 0; i < time_lanelineflows_.sum_; i++) {
        tllf_ptr->score -= 8;
        if (tllf_ptr->score <= 0) {
            PoolDelEntryInLoop(time_lanelineflows_, tllf_ptr, tllf_itr, i);
            continue;
        }

        PoolIteratorLoop(tllf_ptr, tllf_itr);
    }

    PoolIteratorBegin(lanelineflows_, llf_ptr, llf_itr);
    for (i = 0; i < lanelineflows_.sum_; i++, PoolIteratorLoop(llf_ptr, llf_itr)) {
        llf_lln_ptr = llf_ptr->nodes.front();

        index = -1;
        PoolIteratorBegin(time_lanelineflows_, tllf_ptr, tllf_itr);
        tllf_lln_ptr = tllf_ptr->nodes.front_->v;
        for (j = 0; j < time_lanelineflows_.sum_; j++, PoolIteratorLoop(tllf_ptr, tllf_itr)) {
            AngleCalculateDistance(angle_distance, llf_ptr->theta, tllf_ptr->theta);
            if (angle_distance > ANGLE_PARALLEL_AT_LEASE) {
                continue;
            }

            if (abs(tllf_lln_ptr->p1.x - llf_lln_ptr->p1.x) < 4 &&
                    abs(tllf_lln_ptr->p1.y - llf_lln_ptr->p1.y) < 4) {
                tllf_ptr->score += 8;

            } else if(abs(tllf_ptr->top_point_x - llf_ptr->top_point_x) < 4) {
                tllf_ptr->score += 6;

            } else {
                continue;
            }

            if (tllf_ptr->score > 60) tllf_ptr->score = 60;

            llf_ptr->score += tllf_ptr->score;

            index = j;
        }

        if (-1 == index) {
            new_tllf_ptr = time_lanelineflows_.new_node_v();
            new_tllf_ptr->Overlay(llf_ptr);
            new_tllf_ptr->score = llf_ptr->score / 1.2;
        }
    }

    if (EstimateLaneLineFlowsCheckIfCompleted(40)) {
        EstimateLaneLineFlowsCompleted(40);
        return;
    }
}
