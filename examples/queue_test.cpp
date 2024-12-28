#include "queue.hpp"


int main(int argc, char** argv) {

    Queue<int> queue;

    for (int i = 0; i < 15; i++) {
        queue.enqueue(i);
    }

    for (int i = 0; i < 5; i++) {
        queue.dequeue();
    }
    
    queue.print();

    return 0;
}