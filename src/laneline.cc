#include "basic.h"

void VisualSystem::CaptureDoubleLaneLineNodes() {
    int i, j, _int, _x, size;
    float angle_distance, space_distance, distance;
    Vec4i l;

    DoubleLaneLineNode *dlln_ptr, *dlln2_ptr;
    PoolNode<DoubleLaneLineNode> *dlln_itr, *dlln2_itr;
    DoubleLaneLineNode *new_dlln_ptr;

    LaneLineNode lln;
    LaneLineNode *new_lln_ptr;
    LaneLineNode *dlln_lln_ptr;

    houghlines_.clear();
    HoughLinesP(img_ret_, houghlines_, 1, RADIAN_1, 20, 10, 3);

    double_lanelinenodes_.Clear();
    lanelinenodes_.Clear();

    //生成DoubleLaneLineNode
    size = houghlines_.size();
    for(i = 0; i < size; i++ ) {
        l = houghlines_[i];

        //保证线条y坐标递减
        if (l[3] > l[1]) {
            _int = l[2]; l[2] = l[0]; l[0] = _int;
            _int = l[3]; l[3] = l[1]; l[1] = _int;
        //保证y坐标一样时，线条x坐标递增增加
        } else if (l[1] == l[3]) {
            if (l[2] < l[0]) {
                _int = l[2]; l[2] = l[0]; l[0] = _int;
                _int = l[3]; l[3] = l[1]; l[1] = _int;
            }
        }

        lln.p1.x = l[0]; lln.p1.y = l[1];
        lln.p2.x = l[2]; lln.p2.y = l[3];

        EqCalculateLineKBBy2Points(lln.k, lln.b, lln.p1, lln.p2);
        //过滤斜率太小的
        if (fabsf(lln.k) < 0.05) {
            continue;
        }

        //过滤碎片
        if (lln.p1.y - lln.p2.y < LANELINE_HEIGHT_MIN) {
            continue;
        }

        new_lln_ptr = lanelinenodes_.new_node_v();
        *new_lln_ptr = lln;

        EqCalculateThetaByK(new_lln_ptr->theta, new_lln_ptr->k);

        dlln2_ptr = 0;
        PoolIteratorBegin(double_lanelinenodes_, dlln_ptr, dlln_itr);
        for(j = 0; j < double_lanelinenodes_.sum_; j++, PoolIteratorLoop(dlln_ptr, dlln_itr)) {
            dlln_lln_ptr = dlln_ptr->lln1_ptr;

            AngleCalculateDistance(angle_distance, dlln_lln_ptr->theta, new_lln_ptr->theta);
            if (angle_distance > ANGLE_SAME_AT_LEASE) {
                continue;
            }

            EqCalculateXByYKB(_x, new_lln_ptr->p1.x, dlln_lln_ptr->p1.y,
                    new_lln_ptr->k, new_lln_ptr->b);
            if (abs(_x - dlln_lln_ptr->p1.x) > LANELINE_WIDTH_MAX) {
                continue;
            }

            if (abs(dlln_lln_ptr->p1.x - new_lln_ptr->p1.x) < LANELINE_WIDTH_MIN ||
                    abs(dlln_lln_ptr->p2.x - new_lln_ptr->p2.x) < LANELINE_WIDTH_MIN) {
                continue;
            }

            _int = 20;
            if (abs(dlln_lln_ptr->p1.y - new_lln_ptr->p1.y) > _int &&
                    abs(dlln_lln_ptr->p2.y - new_lln_ptr->p2.y) > _int) {
                continue;
            }

            space_distance = abs(dlln_lln_ptr->p1.x - new_lln_ptr->p1.x) +
                abs(dlln_lln_ptr->p1.y - new_lln_ptr->p1.y) +
                abs(dlln_lln_ptr->p2.x - new_lln_ptr->p2.x) +
                abs(dlln_lln_ptr->p2.y - new_lln_ptr->p2.y);


            distance = angle_distance*1800 + space_distance*0.01;

            if (distance > dlln_ptr->llns_distance) {
                continue;
            }

            dlln2_ptr = dlln_ptr;
        }

        if (0 != dlln2_ptr) {
            dlln2_ptr->lln2_ptr = new_lln_ptr;
            dlln2_ptr->llns_sum = 2;
            dlln2_ptr->llns_distance = distance;
        } else {
            new_dlln_ptr = double_lanelinenodes_.new_node_v();
            new_dlln_ptr->lln1_ptr = new_lln_ptr;
            new_dlln_ptr->llns_sum = 1;
        }
    }


    PoolIteratorBegin(double_lanelinenodes_, dlln_ptr, dlln_itr);
    for(i = 0; i < double_lanelinenodes_.sum_; i++) {
        //去掉无效 DoubleLaneLineNode
        if (dlln_ptr->llns_sum < 2) {
            PoolDelEntryInLoop(double_lanelinenodes_, dlln_ptr, dlln_itr, i);
            continue;
        }

        //求解 DoubleLaneLineNode 的中线
        if (dlln_ptr->lln1_ptr->p1.y > dlln_ptr->lln2_ptr->p1.y) {
            EqCalculateXByYKB(_x, dlln_ptr->lln2_ptr->p1.x, 
                    dlln_ptr->lln1_ptr->p1.y, 
                    dlln_ptr->lln2_ptr->k, 
                    dlln_ptr->lln2_ptr->b);
            dlln_ptr->lln.p1.x = (_x + dlln_ptr->lln1_ptr->p1.x) / 2;
            dlln_ptr->lln.p1.y = dlln_ptr->lln1_ptr->p1.y;
        } else {
            EqCalculateXByYKB(_x, dlln_ptr->lln1_ptr->p1.x, 
                    dlln_ptr->lln2_ptr->p1.y, 
                    dlln_ptr->lln1_ptr->k, 
                    dlln_ptr->lln1_ptr->b);
            dlln_ptr->lln.p1.x = (_x + dlln_ptr->lln2_ptr->p1.x) / 2;
            dlln_ptr->lln.p1.y = dlln_ptr->lln2_ptr->p1.y;
        }

        if (dlln_ptr->lln1_ptr->p2.y < dlln_ptr->lln2_ptr->p2.y) {
            EqCalculateXByYKB(_x, dlln_ptr->lln2_ptr->p2.x, 
                    dlln_ptr->lln1_ptr->p2.y, 
                    dlln_ptr->lln2_ptr->k, 
                    dlln_ptr->lln2_ptr->b);
            dlln_ptr->lln.p2.x = (_x + dlln_ptr->lln1_ptr->p2.x) / 2;
            dlln_ptr->lln.p2.y = dlln_ptr->lln1_ptr->p2.y;
        } else {
            EqCalculateXByYKB(_x, dlln_ptr->lln1_ptr->p2.x, 
                    dlln_ptr->lln2_ptr->p2.y, 
                    dlln_ptr->lln1_ptr->k, 
                    dlln_ptr->lln1_ptr->b);
            dlln_ptr->lln.p2.x = (_x + dlln_ptr->lln2_ptr->p2.x) / 2;
            dlln_ptr->lln.p2.y = dlln_ptr->lln2_ptr->p2.y;
        }

        EqCalculateLineKBBy2Points(
                dlln_ptr->lln.k,
                dlln_ptr->lln.b,
                dlln_ptr->lln.p1,
                dlln_ptr->lln.p2);
        EqCalculateThetaByK(dlln_ptr->lln.theta, dlln_ptr->lln.k);

        PoolIteratorLoop(dlln_ptr, dlln_itr);
    }

    //对 double_lanelinenodes_ 按正方向排序
    PoolIteratorBegin(double_lanelinenodes_, dlln_ptr, dlln_itr);
    for(i = 0; i < double_lanelinenodes_.sum_-1; i++, PoolIteratorLoop(dlln_ptr, dlln_itr)) {

        dlln2_ptr = (dlln2_itr = dlln_itr->next)->v;
        for(j = i+1; j < double_lanelinenodes_.sum_; j++, PoolIteratorLoop(dlln2_ptr, dlln2_itr)) {
            if (dlln_ptr->lln.p1.y < dlln2_ptr->lln.p1.y) {
                double_lanelinenodes_.exchange_node(&dlln_itr, &dlln2_itr);
                dlln_ptr = dlln_itr->v;
                dlln2_ptr = dlln2_itr->v;
            }
        }
    }
}
