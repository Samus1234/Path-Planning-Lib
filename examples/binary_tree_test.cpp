#include "binary_tree.hpp"

int main(int argc, char** argv) {

    BinaryTree<int, 0> tree;

    tree.insert(10);
    tree.insert(5);
    tree.insert(15);
    tree.insert(3);
    tree.insert(7);

    tree.print();

    std::cout << "\n";

    std::cout << "Tree height: " << tree.getHeight() << "\n";

    std::cout << "\n";

    auto it = tree.begin();

    for (it = tree.begin(); it != tree.end(); it++) {
        std::cout << *it << " " << std::endl;
    }

    return 0;
}