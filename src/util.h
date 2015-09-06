#ifndef VLD_UTIL_H_
#define VLD_UTIL_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <locale.h>

/* Anti-warning macro... */
#define NOTUSED(V) ((void) V)
#define NOWARN(V) ((void) V)

#define LOG_ERROR 1
#define LOG_WARNING 2
#define LOG_NOTICE 3
#define LOG_INFO 4
#define LOG_DEBUG 6

#define LOG(level, FMT, ...) do {\
    char timestr[64];\
    struct timeval tv;\
    gettimeofday(&tv,NULL);\
    strftime(timestr,sizeof(timestr),"%d %b %H:%M:%S.",localtime(&tv.tv_sec));\
    fputc('[', stderr); fputs(timestr, stderr); fputs("]", stderr); \
    fprintf(stderr, "(%s:%d) ", __FILE__, __LINE__); fprintf(stderr, FMT, ##__VA_ARGS__); \
    fflush(stderr);\
} while(0);

#define LOGD(FMT, ...) LOG(LOG_DEBUG, FMT, ##__VA_ARGS__)
#define LOGW(FMT, ...) LOG(LOG_WARNING, FMT, ##__VA_ARGS__)
#define LOGE(FMT, ...) LOG(LOG_ERROR, FMT, ##__VA_ARGS__)
#define LOGN(FMT, ...) LOG(LOG_NOTICE, FMT, ##__VA_ARGS__)
#define LOGI(FMT, ...) LOG(LOG_INFO, FMT, ##__VA_ARGS__)

#ifdef IS_DEBUG
#include <assert.h>
#define Assert(condition) assert(condition);
#else
#define Assert(condition) {}
#endif

//theta 取 [0, 180)
//计算道路夹角
#define AngleCalculateDistance(ret, theta1, theta2) do {\
    ret = fabsf(theta1 - theta2);\
    if (ret > RADIAN_90) ret = RADIAN_180 - ret;\
} while(0);

//判断是否同方向
inline bool AngleIfCodirectional(float theta1, float theta2) {
    float angle_distance;
    if (theta1 > ANGLE_NEAR_RADIAN_90_MIN && theta1 < ANGLE_NEAR_RADIAN_90_MAX &&
            theta2 > ANGLE_NEAR_RADIAN_90_MIN && theta2 < ANGLE_NEAR_RADIAN_90_MAX) {
        return true;
    }

    if ((theta1 > ANGLE_NEAR_RADIAN_90_MAX && theta2 > ANGLE_NEAR_RADIAN_90_MAX) || 
            (theta1 < ANGLE_NEAR_RADIAN_90_MIN && theta2 < ANGLE_NEAR_RADIAN_90_MIN)) {
        AngleCalculateDistance(angle_distance, theta1, theta2);
        if (angle_distance > ANGLE_CODIRECTIONAL_AT_LEASE) {
            return false;
        } else {
            return true;
        } 
    }

    return false;
}

#define Exchange(a, b) do {\
    a = b ^ a;\
    b = b ^ a;\
    a = b ^ a;\
} while(0);

#define PoolDelEntryInLoop(pool, ptr, itr, index) ptr = (itr = itr->next)->v;\
    pool.del_node(index);\
    index--;

#define PoolIteratorBegin(pool, ptr, itr) ptr = (itr = pool.front_)->v

#define PoolIteratorLoop(ptr, itr) ptr = (itr = itr->next)->v

template<class T> 
class PoolNode {
    public:
        PoolNode *next, *prev;
        T *v;
};

template<class T> 
class Pool {
    public:
        int sum_; //存储对象总数
        int size_;//存储对象空间大小
        PoolNode<T> *front_;
        PoolNode<T> *last_; //front_[sum_-1]
        PoolNode<T> *end_;  //front_[size-1]

        Pool() {
            PoolNode<T> *itr;
            front_ = NewPoolNode();

            itr = front_->next = NewPoolNode();
            itr->prev = front_;

            itr->next = NewPoolNode();
            itr->next->prev = itr;
            itr = itr->next;

            itr->next = NewPoolNode();
            itr->next->prev = itr;
            itr = itr->next;
            
            end_ = itr;

            last_ = 0;
            sum_ = 0;
            size_ = 4;
        }

        PoolNode<T>* NewPoolNode() {
            PoolNode<T> *itr = zero_node(new PoolNode<T>());
            itr->v = new T();
            itr->v->Clear();
            return itr;
        }

        PoolNode<T>* zero_node(PoolNode<T> *itr) {
            itr->next = 0;
            itr->prev = 0;
            return itr;
        }

        PoolNode<T>* new_node() {
            sum_++;

            if (sum_ >= size_-2) {
                int i;
                PoolNode<T> *itr;

                size_ += 16;
                itr = end_;
                for (i = 0; i < 16; i++) {
                    itr->next = NewPoolNode();
                    itr->next->prev = itr;
                    itr = itr->next;
                }
                end_ = itr;
            }

            if (0 == last_) {
                last_ = front_;
            } else {
                last_ = last_->next;
            }

            return last_;
        }

        T* new_node_v() {
            return new_node()->v;
        }

        void del_node(int index) {
            if (sum_ <= 0) return;

            sum_--;
            int last_index = sum_;

            if (0 == sum_) {
                front_->v->Clear();
                last_ = front_;
                return;
            }

            if (last_index == index) {
                last_->v->Clear();
                last_ = last_->prev;
                return;
            }

            int i = 0;
            PoolNode<T> *itr1, *itr2;

            Assert(front_ != last_);

            if (0 == index) {
                itr2 = front_;
                front_ = itr2->next;
                front_->prev = 0;

            } else {
                itr1 = itr2 = front_;
                for (i = 0; i < index-1; i++) {
                    itr1 = itr1->next;
                }
                itr2 = itr1->next;
                itr1->next = itr2->next;
                itr2->next->prev = itr1;
            }

            itr2->v->Clear();
            itr2->next = 0;
            end_->next = itr2;
            itr2->prev = end_;
            end_ = itr2;
        }

        PoolNode<T>* get_node(int index) {
            int i;
            PoolNode<T> *itr = front_;
            for (i = 0; i < index; i++) {
                itr = itr->next;
            }
            return itr;
        }

        T* get_node_v(int index) {
            return get_node(index)->v;
        }
        
        void insert_after(PoolNode<T> *node, PoolNode<T> *target_node) {
            if (node == target_node) return;
            if (node == target_node->next) return;

            if (front_ == node) {
                front_ = node->next;
            }

            if (last_ == node) {
                last_ = node->prev;
            }

            if (last_ == target_node) {
                last_ = node;
            }

            if (0 != node->prev) node->prev->next = node->next;
            if (0 != node->next) node->next->prev = node->prev;

            node->prev = target_node;
            node->next = target_node->next;
            if (0 != target_node->next) {
                target_node->next->prev = node;
            }
            target_node->next = node;
        }
        
        void insert_before(PoolNode<T> *node, PoolNode<T> *target_node) {
            if (node == target_node) return;
            if (node == target_node->prev) return;

            if (front_ == node) {
                front_ = node->next;
            }

            if (last_ == node) {
                last_ = node->prev;
            }

            if (front_ == target_node) {
                front_ = node;
            }

            if (0 != node->prev) node->prev->next = node->next;
            if (0 != node->next) node->next->prev = node->prev;

            node->next = target_node;
            node->prev = target_node->prev;
            if (0 != target_node->prev) {
                target_node->prev->next = node;
            }
            target_node->prev = node;
        }

        void exchange_node(PoolNode<T> **itr1, PoolNode<T> **itr2) {
            PoolNode<T> *tmp_itr;

            if ((*itr1)->next == *itr2) {
                if (0 != (*itr1)->prev) (*itr1)->prev->next = *itr2;
                if (0 != (*itr2)->next) (*itr2)->next->prev = *itr1;

                tmp_itr = (*itr1)->prev;
                (*itr1)->prev = (*itr2);
                (*itr2)->prev = tmp_itr;

                tmp_itr = (*itr2)->next;
                (*itr2)->next = (*itr1);
                (*itr1)->next = tmp_itr;

            } else if((*itr2)->next == *itr1) {
                if (0 != (*itr2)->prev) (*itr2)->prev->next = *itr1;
                if (0 != (*itr1)->next) (*itr1)->next->prev = *itr2;

                tmp_itr = (*itr2)->prev;
                (*itr2)->prev = (*itr1);
                (*itr1)->prev = tmp_itr;

                tmp_itr = (*itr1)->next;
                (*itr1)->next = (*itr2);
                (*itr2)->next = tmp_itr;

            } else {
                if (0 != (*itr1)->prev) (*itr1)->prev->next = *itr2;
                if (0 != (*itr1)->next) (*itr1)->next->prev = *itr2;
                if (0 != (*itr2)->prev) (*itr2)->prev->next = *itr1;
                if (0 != (*itr2)->next) (*itr2)->next->prev = *itr1;

                tmp_itr = (*itr2)->prev;
                (*itr2)->prev= (*itr1)->prev;
                (*itr1)->prev = tmp_itr;

                tmp_itr = (*itr2)->next;
                (*itr2)->next = (*itr1)->next;
                (*itr1)->next= tmp_itr;
            }

            if (*itr1 == front_) {
                front_ = *itr2;
            } else if (*itr2 == front_) {
                front_ = *itr1;
            }

            tmp_itr = (*itr1);
            (*itr1) = (*itr2);
            (*itr2) = tmp_itr;
        }

        void Clear() {
            int i;
            PoolNode<T> *itr = front_;
            for (i = 0; i < sum_; i++) {
                itr->v->Clear();
                itr = itr->next;
            }

            sum_ = 0;
            last_ = 0;
        }

        void Release() {
            int i;

            sum_ = 0;
            PoolNode<T> *itr1 = front_, *itr2;
            for (i = 0; i < size_; i++) {
                itr2 = itr1->next;
                delete itr1;
                itr1 = itr2;
            }
            front_ = 0;
            end_ = 0;
            size_= 0;
        }

        ~Pool() {
            this->Release();
        }
};

#endif
