#include "priority_queue.hpp"

int main(int argc, char* argv[]) {
    PriorityQueue<int> pq;

    pq.push(5);
    pq.push(2);
    pq.push(8);
    pq.push(1);

    while (!pq.empty()) {
        std::cout << pq.pop() << " ";
    }
    std::cout << "\n";

    return 0;
}