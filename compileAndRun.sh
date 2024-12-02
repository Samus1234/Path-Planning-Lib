filename=$1

executable_name="${filename%.*}"

mkdir -p build

clang++ -Wall -O2 -std=c++20 -I/usr/include/python3.10 -lpython3.10 -o build/${executable_name} ${filename}

./build/${executable_name}
