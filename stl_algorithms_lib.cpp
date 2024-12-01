#include <iostream>
// Threading
#include <thread>
#include <mutex>
#include <semaphore>
// Containers
#include <queue>
#include <vector>
// Smart pointer
#include <memory>
// Functions
#include <functional>

template <typename T>
struct Node {
    Node() = default;
    Node(T input_data) : data_(input_data) {}
    T data_{NULL};
    Node* prev_{nullptr};
    Node* next_{nullptr};
};

template <typename T>
class List {
public:
    List() = default;
    ~List() {
        // Delete all memory
        Node<T>* next_node = head_;
        while (next_node) {
            Node<T>* delete_node = next_node;
            next_node = next_node->next_;
            delete delete_node;
        } 
    }

    void insertEnd(const T& data) {
        Node<T>* new_node = new Node<T>(data);
        // Create the first element in the list
        if (head_ == NULL) {
            head_ = new_node;
            // Set the tail pointer
            tail_ = new_node;
            return;
        }
        new_node->next_ = head_;
        head_->prev_ = new_node;
        head_ = new_node;
    }

    bool isEmpty() {
        return head_ == NULL;
    }

    void printForward() {
        // Check for empty list
        if (isEmpty()) {
            std::cout << "List Empty" << std::endl;
            return;
        }
        Node<T>* next_node = head_;
        while (next_node) {
            std::cout << next_node->data_ << ", ";
            next_node = next_node->next_;
        }
        std::cout << "\n";
    }

    void printBackward() {
        // Check for empty list
        if (isEmpty()) {
            std::cout << "List Empty" << std::endl;
            return;
        }
        Node<T>* prev_node = tail_;
        while (prev_node) {
            std::cout << prev_node->data_ << ", ";
            prev_node = prev_node->prev_;
        }
        std::cout << "\n";
    }

private:
    Node<T>* head_{NULL};
    Node<T>* tail_{NULL};
};


int main(int argc, char** argv) {

    List<int> list;

    list.insertEnd(2);

    list.insertEnd(5);

    list.insertEnd(3);

    list.insertEnd(6);

    list.insertEnd(9);

    list.insertEnd(8);

    list.printForward();

    list.printBackward();

    return 0;
}