#ifndef _CIRCULAR_BUFFER_H_
#define _CIRCULAR_BUFFER_H_

#include <iostream>
#include <cstring>

template <typename T, size_t N>
class CircularBuffer {
public:
    CircularBuffer() {
        std::fill(std::begin(data_), std::end(data_), T());
    }

    ~CircularBuffer() = default;

    void push(const T& data) {
        if (!full()) {
            data_[index_] = data;
            index_++;
        } else {
            for (size_t i = 0; i < N; i++){
                size_t j = (N-1) - i;
                if (j > 0) {
                    data_[j] = data_[j-1];
                }
            }
            data_[0] = data;
        }
    }

    bool full() { return index_ == N; }

    void printBuffer() {
        for (size_t i = 0; i < index_; i++) {
            std::cout << data_[i] << " -> ";
        }
        std::cout << "end" << std::endl;
    }

private:
    T data_[N];
    size_t index_{0};
};

#endif /* _CIRCULAR_BUFFER_H_ */