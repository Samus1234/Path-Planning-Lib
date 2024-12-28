#include "stack.hpp"

int main(int argc, char** argv) {

    Stack<int> stack;

    for (int i = 0; i < 15; i++) {
        stack.push(i);
    }

    for (int i = 0; i < 5; i++) {
        stack.pop();
    }

    stack.print();    

    return 0;
}