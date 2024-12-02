mkdir -p build

clang++ -Wall -O2 -std=c++20 -I/usr/include/python3.10 -lpython3.10 -o build/$1 $1.cpp

./build/$1