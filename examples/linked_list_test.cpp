#include "doubly_linked_list.hpp"

int main(int argc, char** argv) {

    std::cout << "\n";

    DoublyLinkedList<int> list;

    for (int i = 0; i < 15; i++) {
        list.insertAtFront(i);
    }

    std::cout << "Original List" << std::endl;

    list.printForward();

    std::cout << "\n";

    list.printReverse();

    std::cout << "\n";

    for (int i = 0; i < 5; i++) {
        list.deleteAtFront();
    }

    std::cout << "After front deletion" << std::endl;

    list.printForward();

    std::cout << "\n";

    list.printReverse();

    std::cout << "\n";

    for (int i = 0; i < 5; i++) {
        list.deleteAtEnd();
    }

    std::cout << "After end deletion" << std::endl;

    list.printForward();

    std::cout << "\n";

    list.printReverse();

    std::cout << "\n";

    for (int i = 0; i < 5; i++) {
        list.insertAtEnd(i);
    }

    std::cout << "After end insertion" << std::endl;

    list.printForward();

    std::cout << "\n";

    list.printReverse();

    std::cout << "\n";

    std::cout << "Front of list: " << list.peekFront() << "\n";;
    std::cout << "End of list: " << list.peekEnd() << std::endl;

    std::cout << "\n";

    // Insert at position 2 (middle)
    Node<int>* node = list.head_->next_->next_;
    list.insertAt(99, node);
    std::cout << "After insertion at position 2:" << std::endl;
    list.printForward();
    std::cout << "\n";

    // Delete at position 2
    node = list.head_->next_->next_;
    list.deleteAt(node);
    std::cout << "After deletion at position 2:" << std::endl;
    list.printForward();
    std::cout << "\n";

    std::cout <<
        "----------------------------------- Algorithms Test -----------------------------------------"
        << std::endl;
    
    auto it = std::find(list.begin(), list.end(), 4);

    std::cout << "Element is: " << *it << std::endl;

    return 0;
}