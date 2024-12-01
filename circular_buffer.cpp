#include <iostream>
#include <cstring>

template <typename T, size_t N>
class CircularBuffer {
public:
    CircularBuffer() {
        memset(data_, (T) NULL, N);
    }

    ~CircularBuffer() = default;

    void push(const T& data) {
        if (!full()) {
            data_[index_] = data;
            index_++;
        } else {
            for (size_t i = 0; i < N; i++){
                size_t j = (N-1) - i;
                data_[j] = data_[j-1];
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

int main(int argc, char** argv) {

    CircularBuffer<int, 5> buffer;
    
    for (size_t i = 0; i < 25; i++) {
        buffer.push(i);
        buffer.printBuffer();
    }
    

    return 0;
}