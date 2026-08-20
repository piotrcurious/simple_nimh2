
#include <iostream>
#include <cassert>

static int g_lock_depth = 0;
static int g_max_lock_depth = 0;

void test_lock() {
    g_lock_depth++;
    if (g_lock_depth > g_max_lock_depth) {
        g_max_lock_depth = g_lock_depth;
    }
}

void test_unlock() {
    g_lock_depth--;
    assert(g_lock_depth >= 0);
}

#define WEB_LOCK() test_lock()
#define WEB_UNLOCK() test_unlock()

#include "main.cpp"
