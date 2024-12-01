#include <iostream>

template<typename T>
struct Node {
    Node() = default;
    ~Node() = default;

    Node(const T& data) : data_(data) {}
    Node(T&& data) : data_(std::move(data)) {}

    T data_{};
    Node* below_{nullptr};
};

template<typename T>
class Stack {
    using MyNode = Node<T>;
public:
    Stack() = default;

    ~Stack() {
        while (!isEmpty()) {
            pop();
        }
    }

    Stack(const Stack& other) = delete;

    Stack(Stack&& other) = delete;

    bool isEmpty() const {
        return top_ == nullptr;
    }

    size_t size() const {
        return size_;
    }

    const T& peek() const {
        if (isEmpty()) {
            throw std::runtime_error("Cannot peek: Stack is empty.");
        }
        return top_->data_;
    }

    void push(const T& data) {
        Node<T>* new_node = new Node<T>(data);
        if(isEmpty()) {
            top_ = new_node;
            size_++;
            return;
        }
        new_node->below_ = top_;
        top_ = new_node;
        size_++;
    }

    void push(T&& data) {
        Node<T>* new_node = new Node<T>(std::move(data));
        if(isEmpty()) {
            top_ = new_node;
            size_++;
            return;
        }
        new_node->below_ = top_;
        top_ = new_node;
        size_++;
    }

    T pop() {
        if (isEmpty()) {
            throw std::runtime_error("Cannot pop: Stack is empty.");
        }
        T data = top_->data_;
        Node<T>* temp_node = top_;
        top_ = top_->below_;
        delete temp_node;
        size_--;
        return data;
    }

    void print() const {
        Node<T>* temp_node = top_;
        std::cout << "Top |";
        while (temp_node) { // Stop when temp_node is nullptr
            std::cout << temp_node->data_ << "|";
            temp_node = temp_node->below_; // Move to the next node
        }
        std::cout << " End" << std::endl;
    }


public:
    size_t size_{0};
    Node<T>* top_{nullptr};
};

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