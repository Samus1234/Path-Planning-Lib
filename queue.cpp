#include <iostream>

template<typename T>
struct Node {
    Node() = default;
    ~Node() = default;

    Node(const T& data) : data_(data) {}
    Node(T&& data) : data_(std::move(data)) {}

    T data_{};
    Node* next_{nullptr};
};

template<typename T>
class Queue {
    using Node = Node<T>;
public:
    Queue() = default;

    ~Queue() {
        // while (!isEmpty()){
        //     dequeue();
        // }
    }

    Queue(const Queue& other) = delete;

    Queue(Queue&& other) = delete;

    bool isEmpty() const {
        return back_ == nullptr;
    }

    size_t size() const {
        return size_;
    }

    const T& peek() const {
        if (isEmpty()) {
            throw std::runtime_error("Cannot Peek : Queue is empty");
        }
        return front_->data_; 
    }

    void enqueue(const T& data) {
        Node* new_node = new Node(data);
        if (isEmpty()) {
            front_ = new_node;
            back_ = new_node;
            size_++;
            return;
        }
        back_->next_ = new_node;
        back_ = new_node;
        size_++;
    }

    void enqueue(T&& data) {
        Node* new_node = new Node(std::move(data));
        if (isEmpty()) {
            front_ = new_node;
            back_ = new_node;
            size_++;
            return;
        }
        back_->next_ = new_node;
        back_ = new_node;
        size_++;
    }

    T dequeue() {
        if (isEmpty()) {
            throw std::runtime_error("Cannot dequeue: Queue is empty.");
        }
        T data = front_->data_;
        Node* temp_node = front_;
        front_ = front_->next_;

        delete temp_node;
        size_--;

        if (isEmpty()) {
            back_ = nullptr;
        }
        return data;
    }

    void print() {
        Node* temp_node = front_;
        std::cout << "Back ";
        while (temp_node) {
            std::cout << temp_node->data_ << " -> ";
            temp_node = temp_node->next_;
        }
        std::cout << "Front" << std::endl;
    }

public:
    size_t size_{0};
    Node* back_{nullptr};
    Node* front_{nullptr};

};

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