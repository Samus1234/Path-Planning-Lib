#include <iostream>
#include <stack>
#include <queue>

template<typename T>
struct Node{
public:
    Node() = default;
    ~Node() = default;

    Node(const T& data) : data_(data) {}
    Node(T&& data) : data_(std::move(data)) {}

public:
    T data_{};
    Node* parent_{nullptr};
    Node* left_child_{nullptr};
    Node* right_child_{nullptr};
};

template<typename BinaryTree, uint8_t TRAVERSAL>
class BinaryTreeIterator {
public:
private:
};

template<typename T>
class BinaryTree {
public:
    using NodeType = Node<T>;
public:
    BinaryTree() = default;

    ~BinaryTree() {
        clearNode(root_);
    }

    size_t getHeight() const {
        return calculateHeight(root_);
    }

    void clear() {
        clearNode(root_);
    }

    void insert(const T& data) {
        root_ = insertNode(root_, data);
    }

    void insert(T&& data) {
        root_ = insertNode(root_, std::move(data));
    }

    void search(const T& data) const {
        printf("Node was %s in tree\n", searchNode(root_, data) ? "found" : "not found");
    }

    void search(T&& data) const {
        printf("Node was %s in tree\n", searchNode(root_, std::move(data)) ? "found" : "not found");
    }

    void deleteNode(const T& data) {
        NodeType* node = searchNode(root_, data);
        if (node == nullptr) {
            std::cout << "Node was not found in tree\n";
            return;
        }
        if((node->left_child_ == nullptr) && (node->right_child_ == nullptr)) {
            delete node;
        } else if ((node->left_child_ == nullptr) && (node->right_child_)) {
            node = node->left_child_;
            delete node->left_child_;
        } else if ((node->right_child_ == nullptr) && (node->left_child_)) {
            node = node->right_child_;
            delete node->right_child_;
        } else {

        }
        
    }

    void deleteNode(T&& data) {
        NodeType* deleteNode = searchNode(root_, std::move(data));
        if (deleteNode == nullptr) {
            std::cout << "Node was not found in tree\n";
            return;
        }
        clearNode(deleteNode);
    }

    void print() const {
        std::cout << "In order traversal" << ": ";
        inOrderTraversal(root_);
        std::cout << "\n";
        std::cout << "Pre order traversal" << ": ";
        preOrderTraversalStack(root_);
        std::cout << "\n";
        std::cout << "Post order traversal" << ": ";
        postOrderTraversal(root_);
        std::cout << "\n";
        std::cout << "Level order traversal" << ": ";
        levelOrderTraversal(root_);
        std::cout << "" << std::endl;
    }

private:

    NodeType* insertNode(NodeType* node, const T& data) {
        if (node == nullptr) {
            NodeType* new_node = new Node(data);
            return new_node;
        }
        if (node->data_ == data) {
            return node;
        } else if (node->data > data) {
            node->left_child_ = insertNode(node->left_child_, data);
            node->left_child_->parent_ = node;
        } else {
            node->right_child_ = insertNode(node->right_child_, data);
            node->right_child_->parent_ = node;
        }
        return node;
    }

    NodeType* insertNode(NodeType* node, T&& data) {
        if (node == nullptr) {
            NodeType* new_node = new Node(std::move(data));
            return new_node;
        }
        if (node->data_ == data) {
            return node;
        } else if (node->data_ > data) {
            node->left_child_ = insertNode(node->left_child_, std::move(data));
            node->left_child_->parent_ = node;
        } else {
            node->right_child_ = insertNode(node->right_child_, std::move(data));
            node->right_child_->parent_ = node;
        }
        return node;
    }

    NodeType* searchNode(NodeType* node, const T& data) const {
        if (node == nullptr) {
            return nullptr;
        }
        std::stack<NodeType*> node_stack;
        node_stack.push(node);

        while (!node_stack.empty()) {
            NodeType* current_node = node_stack.top();
            node_stack.pop();
            if (current_node->data_ == data) {
                return current_node;
            }
            if (current_node->left_child_) {
                node_stack.push(current_node->left_child_);
            }
            if (current_node->right_child_) {
                node_stack.push(current_node->right_child_);
            }            
        }
        return nullptr;       
    }

    NodeType* searchNode(NodeType* node, T&& data) const {
        if (node == nullptr) {
            return nullptr;
        }
        std::stack<NodeType*> node_stack;
        node_stack.push(node);



        while (!node_stack.empty()) {
            NodeType* current_node = node_stack.top();
            node_stack.pop();
            if (current_node->data_ == data) {
                return current_node;
            }
            if (current_node->left_child_) {
                node_stack.push(current_node->left_child_);
            }
            if (current_node->right_child_) {
                node_stack.push(current_node->right_child_);
            }            
        }
        return nullptr; 
    }

    size_t calculateHeight(NodeType* node) {
        if (node == nullptr) {
            return 0;
        }
        return 1 + std::max(
            calculateHeight(node->left_child_),
            calculateHeight(node->right_child_)
        );
    }

    void clearNode(NodeType* node) {
        if (node == nullptr) {
            return;
        }
        clearNode(node->left_child_);
        clearNode(node->right_child_);
        delete node;
    }

    void inOrderTraversal(NodeType* node) const {
        if (node == nullptr) {
            return;
        }
        inOrderTraversal(node->left_child_);
        std::cout << node->data_ << " ";
        inOrderTraversal(node->right_child_);
    }

    void preOrderTraversal(NodeType* node) const {
        if (node == nullptr) {
            return;
        }
        std::cout << node->data_ << " ";
        preOrderTraversal(node->left_child_);
        preOrderTraversal(node->right_child_);
    }

    void postOrderTraversal(NodeType* node) const {
        if (node == nullptr) {
            return;
        }
        postOrderTraversal(node->left_child_);
        postOrderTraversal(node->right_child_);
        std::cout << node->data_ << " ";
    }

    void inOrderTraversalStack(NodeType* node) const {
        if (node == nullptr) {
            return;
        }
        std::stack<NodeType*> node_stack;
        node_stack.push(node);

    }

    void preOrderTraversalStack(NodeType* node) const {
        if (node == nullptr) {
            return;
        }
        std::stack<NodeType*> node_stack;
        node_stack.push(node);

        while(!node_stack.empty()) {
            NodeType* current_node = node_stack.top();
            node_stack.pop();
            if (current_node->right_child_) {
                node_stack.push(current_node->right_child_);
            }
            if (current_node->left_child_) {
                node_stack.push(current_node->left_child_);
            }
            std::cout << current_node->data_ << " ";
        }
    }

    void postOrderTraversalStack(NodeType* node) const {
        if (node == nullptr) {
            return;
        }
        std::stack<NodeType*> node_stack; 
    }

    void levelOrderTraversal(NodeType* node) const {
        if (node == nullptr) {
            return;
        }
        std::queue<NodeType*> node_queue;
        node_queue.push(node);

        while(!node_queue.empty()) {
            NodeType* current_node = node_queue.front();
            node_queue.pop();
            if (current_node->left_child_) {
                node_queue.push(current_node->left_child_);
            }
            if (current_node->right_child_) {
                node_queue.push(current_node->right_child_);
            }
            std::cout << current_node->data_ << " ";
        }
    }
public:
    NodeType* root_{nullptr};
};

int main(int argc, char** argv) {

    BinaryTree<int> tree;

    tree.insert(10);
    tree.insert(5);
    tree.insert(15);
    tree.insert(3);
    tree.insert(7);

    tree.print();

    // std::cout << "\n";

    // tree.deleteNode(4);
    // tree.deleteNode(5);

    // std::cout << "\n";

    // tree.print();

    return 0;
}