#include "circular_buffer.hpp"


int main(int argc, char** argv) {

    CircularBuffer<int, 5> buffer;
    
    for (size_t i = 0; i < 25; i++) {
        buffer.push(i);
        buffer.printBuffer();
    }
    
    return 0;
}