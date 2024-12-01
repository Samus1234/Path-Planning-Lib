#include <iostream>

int arraySum(int arr[], int n) {
    if (n == 0) {
        return 0;
    }
    return arr[n - 1] + arraySum(arr, n - 1);
}

int main(int argc, char** argv) {

    int arr[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    std::cout << arraySum(arr, 10) << std::endl;

    return 0;
}