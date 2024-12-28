#ifndef _STACK_H_
#define _STACK_H_

#include <iostream>

template<typename T>
struct StackNode {
    StackNode() = default;
    ~StackNode() = default;

    StackNode(const T& data) : data_(data) {}
    StackNode(T&& data) : data_(std::move(data)) {}

    T data_{};
    StackNode* below_{nullptr};
};

template<typename T>
class Stack {
    using Node = StackNode<T>;
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
        Node* new_node = new Node(data);
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
        Node* new_node = new Node(std::move(data));
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
        Node* temp_node = top_;
        top_ = top_->below_;
        delete temp_node;
        size_--;
        return data;
    }

    void print() const {
        Node* temp_node = top_;
        std::cout << "Top |";
        while (temp_node) { // Stop when temp_node is nullptr
            std::cout << temp_node->data_ << "|";
            temp_node = temp_node->below_; // Move to the next node
        }
        std::cout << " End" << std::endl;
    }


public:
    size_t size_{0};
    Node* top_{nullptr};
};

#endif /* _STACK_H_ */