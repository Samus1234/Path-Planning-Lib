#include <iostream>
#include <vector>

template <typename T>
class PriorityQueue {
public:
    PriorityQueue() = default;
    ~PriorityQueue() = default;

    PriorityQueue(const std::vector<T>& q) : q_(q) {
        for (int i = parent(size() - 1); i >= 0; --i) {
            downHeapify(i);
        }
    }

    const T& peek() const {
        return q_.at(0);
    }

    size_t size() const {
        return q_.size();
    }

    bool empty() const {
        return q_.empty();
    }

    void upHeapify(size_t index) {
        if (index >= size() || index < 0) {
            throw std::out_of_range("Invalid Index");
        }
        while (index > 0) {
            size_t parent_index = parent(index);
            if (q_[index] < q_[parent_index]) {
                std::swap(q_[index], q_[parent_index]);
                index = parent_index;
            } else {
                break;
            }
        }
    }

    void downHeapify(size_t index) {
        if (index >= size()) {
            throw std::out_of_range("Invalid Index");
        }
        while (index < size()) {
            size_t left = leftChild(index);
            size_t right = rightChild(index);
            size_t min_child_index = index;

            if (left < size() && q_[left] < q_[min_child_index]) {
                min_child_index = left;
            }
            if (right < size() && q_[right] < q_[min_child_index]) {
                min_child_index = right;
            }

            if (min_child_index != index) {
                std::swap(q_[index], q_[min_child_index]);
                index = min_child_index;
            } else {
                break;
            }
        }
    }

    void push(const T& data) {
        q_.push_back(data);
        upHeapify(size()-1);
    }

    void push(T&& data) {
        q_.emplace_back(std::move(data));
        upHeapify(size()-1);
    }

    T pop() {
        if (empty()) {
            throw std::out_of_range("Heap is empty");
        }
        T front = q_[0];
        std::swap(q_[0], q_.back());
        q_.pop_back();
        if (!empty()) {
            downHeapify(0);
        }
        return front;
    }

    void printHeap() const {
        for (const auto& elem : q_) {
            std::cout << elem << " ";
        }
        std::cout << "\n";
    }
    
private:

    size_t parent(size_t index) {
        return (index - 1) / 2;
    }

    size_t leftChild(size_t index) {
        return index*2 + 1;
    }

    size_t rightChild(size_t index) {
        return index*2 + 2;
    }

    std::vector<T> q_;
};



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