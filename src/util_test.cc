#include "basic.h"
#include "assert.h"

typedef struct DN {
    void Clear() {}
} DN;

typedef struct D {
    Pool<DN> nodes;
    void Clear() {}
} D;

VisualSystem g_v;
Pool<D> ds;

void printDs() {
    int i;
    D *d_ptr;
    PoolNode<D> *d_itr;

    LOGI("%d\n", ds.sum_);
    PoolIteratorBegin(ds, d_ptr, d_itr);
    for (i = 0; i < ds.sum_; i++, PoolIteratorLoop(d_ptr, d_itr)) {
        printf("%p\n", d_itr);
    }
    printf("\n");
}

void test_basic() {
    int i;

    D *d_ptr;
    PoolNode<D> *d_itr;
    D *d1_ptr;
    PoolNode<D> *d1_itr;
    D *d2_ptr;
    PoolNode<D> *d2_itr;

    for (i = 0; i < 6; i++) {
        d_ptr = ds.new_node_v();
    }

    d_itr = ds.get_node(0);

    d1_ptr = (d1_itr = ds.get_node(0))->v;
    d2_ptr = (d2_itr = ds.get_node(1))->v;
    ds.exchange_node(&d1_itr, &d2_itr);

    assert(d_itr == ds.get_node(1));
    assert(d_itr->next == ds.get_node(2));
    assert(d_itr->prev == ds.get_node(0));
    assert(ds.get_node(ds.sum_-1) == ds.last_);

    d1_itr = ds.last_;

    ds.del_node(0);

    d_itr = ds.new_node();

    ds.insert_after(d_itr, d1_itr);
    assert(d1_itr->next == d_itr);
    assert(d_itr->prev == d1_itr);
    assert(ds.get_node(ds.sum_-1) == ds.last_);

    ds.del_node(0);
    ds.del_node(ds.sum_-1);
    assert(ds.get_node(ds.sum_-1) == ds.last_);


    d_itr = ds.new_node();

    ds.insert_before(d_itr, d1_itr);
    assert(d1_itr->prev == d_itr);
    assert(d_itr->next == d1_itr);
    assert(ds.get_node(ds.sum_-1) == ds.last_);
}

void test_child() {
    D *d_ptr;
    DN *d_dt_ptr;
    int i;

    for (i = 0; i < 20; i++) {
        d_ptr = ds.new_node_v();
    }

    for (i = 0; i < 20; i++) {
        d_dt_ptr = d_ptr->nodes.new_node_v();
    }
}

int main(int argc, char **argv) {
    //test_child();
    test_basic();
    return 0;
}
