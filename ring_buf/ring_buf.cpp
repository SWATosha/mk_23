#include "ring_buf.hpp"

Ring_buffer::Ring_buffer():wr_idx{0}, rd_idx{0}, size{0} {}

uint8_t Ring_buffer::get () {
    uint8_t tmp = arr[rd_idx];
    rd_idx++;
    rd_idx %= CAPACITY;
    size--;
    return tmp;
}

void Ring_buffer::put (uint8_t val) {
    arr[wr_idx++] = val;
    wr_idx %= CAPACITY;
    size++;
}

bool Ring_buffer::empty () {
    return (size == 0);
}

bool Ring_buffer::not_full () {
    return (size <= CAPACITY);
}