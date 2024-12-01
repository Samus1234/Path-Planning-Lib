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

template<typename T>
class SafeContainer
{
public:
    SafeContainer() = default;
    ~SafeContainer() = default;

    void enterContainer(T input) {
        std::scoped_lock lock(this->mtx_);
        q_.push_back(input);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const T& frontOfContainer() const {
        std::scoped_lock lock(this->mtx_);
        return q_.front();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    T& popFrontContainer() {
        std::scoped_lock lock(this->mtx_);
        return q_.pop();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void incrementElement(int index) {
        q_[index]++;
    }

    void printContainer() {
        printf("{");
        for (auto& val : q_) {
            printf("%d, ", val);
        }
        printf("}\n");
    }

private:
    std::vector<T> q_;
    std::mutex mtx_;
};


/// @brief Main function
/// @param argc number of arguments
/// @param argv array of string arguments
/// @return integer indicating success/failure
int main(int argc, char** argv) {

    auto container = std::make_shared<SafeContainer<int>>();

    std::function<void(int)> enter_container =
        [container] (int input) { return container->enterContainer(input); };

    std::vector<std::thread> threads;

    for (size_t i = 0; i < 10; i++) {
        threads.push_back(std::thread(enter_container, (int)(i + 1)));
    }

    for (auto& thread : threads) {
        thread.join();
    }
    
    container->printContainer();

    return 0;
}