#ifndef _BINARY_TREE_H_
#define _BINARY_TREE_H_

#include <iostream>
#include <queue>
#include <stack>

template<typename BinaryTree, uint8_t TRAVERSAL>
class BinaryTreeIterator {
    using ValueType = typename BinaryTree::ValueType;
    using NodeType = typename BinaryTree::NodeType;
public:
    // Iterator traits
    using iterator_category = std::bidirectional_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = ValueType;
    using pointer = ValueType*;
    using reference = ValueType&;

    BinaryTreeIterator() = default;
    ~BinaryTreeIterator() = default;

    BinaryTreeIterator(NodeType* m_ptr) : m_ptr_(m_ptr) {
        switch (TRAVERSAL) {
        case BinaryTree::IN_ORDER:
            stack_.push(m_ptr_);
            break;
        case BinaryTree::PRE_ORDER:
            stack_.push(m_ptr_);
            break;
        case BinaryTree::POST_ORDER:
            stack_.push(m_ptr_);
            break;
        case BinaryTree::LEVEL_ORDER:
            queue_.push(m_ptr_);
            break;
        default:
            throw std::runtime_error("Not a valid traversal option");
            break;
        }
    }

    BinaryTreeIterator& operator ++ () {
        switch (TRAVERSAL) {
        case BinaryTree::IN_ORDER:
            while (!stack_.empty() || m_ptr_ != nullptr) {
                if (m_ptr_ != nullptr) {
                    stack_.push(m_ptr_);
                    m_ptr_ = m_ptr_->left_child_;
                } else {
                    m_ptr_ = stack_.top();
                    stack_.pop();
                    m_ptr_ = m_ptr_->right_child_;
                }
            }
            break;

        case BinaryTree::PRE_ORDER:
            if (!stack_.empty()) {
                m_ptr_ = stack_.top();
                stack_.pop();
                if (m_ptr_->right_child_) {
                    stack_.push(m_ptr_->right_child_);
                }
                if (m_ptr_->left_child_) {
                    stack_.push(m_ptr_->left_child_);
                }
            } else {
                m_ptr_ = nullptr;
            }
            break;

        case BinaryTree::POST_ORDER:
            if (!stack_.empty()) {
                m_ptr_ = stack_.top();
                stack_.pop();
                if (m_ptr_->right_child_) {
                    stack_.push(m_ptr_->right_child_);
                }
                if (m_ptr_->left_child_) {
                    stack_.push(m_ptr_->left_child_);
                }
            } else {
                m_ptr_ = nullptr;
            }
            break;

        case BinaryTree::LEVEL_ORDER:
            if (!stack_.empty()) {
                m_ptr_ = stack_.top();
                stack_.pop();
                if (m_ptr_->right_child_) {
                    stack_.push(m_ptr_->right_child_);
                }
                if (m_ptr_->left_child_) {
                    stack_.push(m_ptr_->left_child_);
                }
            } else {
                m_ptr_ = nullptr;
            }
            break;
        
        default:
            throw std::runtime_error("Invalid traversal mode");
            break;
        }
        return *this;
    }

    BinaryTreeIterator operator ++ (int) {
        BinaryTreeIterator it = *this;
        ++(*this);
        return it;
    }

    BinaryTreeIterator& operator -- () {

    }

    BinaryTreeIterator operator -- (int) {
        BinaryTreeIterator it = *this;
        --(*this);
        return it;
    }

    reference operator * () {
        return m_ptr_->data_;
    }

    pointer operator -> () {
        return &(m_ptr_->data_);
    }

    bool operator == (const BinaryTreeIterator& other) const {
        return m_ptr_ == other.m_ptr_;
    }

    bool operator != (const BinaryTreeIterator& other) const {
        return m_ptr_ != other.m_ptr_;
    }

private:
    NodeType* m_ptr_{nullptr};
    std::stack<NodeType*> stack_;
    std::queue<NodeType*> queue_;
};

template<typename T>
struct Node {
    Node() = default;
    ~Node() = default;

    Node(const T& data) : data_(data) {}
    Node(T&& data) : data_(std::move(data)) {}

    T data_{};
    Node* parent_{nullptr};
    Node* left_child_{nullptr};
    Node* right_child_{nullptr};
};


template<typename T, uint8_t TRAVERSAL>
class BinaryTree {
public:
    static constexpr uint8_t IN_ORDER = 0;
    static constexpr uint8_t PRE_ORDER = 1;
    static constexpr uint8_t POST_ORDER = 2;
    static constexpr uint8_t LEVEL_ORDER = 3;
    using ValueType = T;
    using NodeType = Node<T>;
    using Iterator = BinaryTreeIterator<BinaryTree, TRAVERSAL>;
public:
    BinaryTree() = default;

    ~BinaryTree() {
        clear(root_);
    }

    size_t getHeight() const {
        return calculateHeight(root_);
    }

    void insert(const T& data) {
        root_ = insertNode(root_, data);
    }

    void insert(T&& data) {
        root_ = insertNode(root_, std::move(data));
    }

    void print() const {
        std::cout << "Pre-order traversal:\n";
        preOrderTraversal(root_);
        std::cout << "\n";

        std::cout << "Pre-order traversal stack:\n";
        preOrderTraversalStack(root_);
        std::cout << "\n";

        std::cout << "Post-order traversal:\n";
        postOrderTraversal(root_);
        std::cout << "\n";

        std::cout << "In-order traversal:\n";
        inOrderTraversal(root_);
        std::cout << "" << std::endl;

        std::cout << "Level-order traversal:\n";
        levelOrderTraversal(root_);
        std::cout << "" << std::endl;
    }

    Iterator begin() {
        return Iterator(root_);
    }

    Iterator end() {
        return Iterator(nullptr);
    }

private:
    NodeType* insertNode(NodeType* node, const T& data) {

        if (node == nullptr) {
            NodeType* new_node = new NodeType(data);
            return new_node;
        }
        if (node->data_ == data) {
            return node;
        } else if (node->data_ < data) {
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
            NodeType* new_node = new NodeType(std::move(data));
            return new_node;
        }
        if (node->data_ == data) {
            return node;
        } else if (node->data_ < data) {
            node->right_child_ = insertNode(node->right_child_, std::move(data));
            node->right_child_->parent_ = node;
        } else {
            node->left_child_ = insertNode(node->left_child_, std::move(data));
            node->left_child_->parent_ = node;
        }
        
        return node;
    }

    void clear(NodeType* node) {
        if (node == nullptr) {
            return;
        }
        clear(node->left_child_);
        clear(node->right_child_);
    }

    size_t calculateHeight(NodeType* node) const {
        if (node == nullptr) {
            return 0;
        }

        return 1 + std::max(
            calculateHeight(node->left_child_),
            calculateHeight(node->right_child_)
        );
        
    }

    void preOrderTraversalStack(NodeType* node) const {
        if (node == nullptr) {
            return;
        }

        std::stack<NodeType*> stack;
        stack.push(node);
        while(!stack.empty()) {
            auto current = stack.top();
            stack.pop();
            std::cout << current->data_ << " ";
            if(current->right_child_) {
                stack.push(current->right_child_);
            }
            if(current->left_child_) {
                stack.push(current->left_child_);
            }
        }
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

    void inOrderTraversal(NodeType* node) const {
        if (node == nullptr) {
            return;
        }
        inOrderTraversal(node->left_child_);
        std::cout << node->data_ << " ";
        inOrderTraversal(node->right_child_);
    }

    void levelOrderTraversal(NodeType* node) const {
        if (node == nullptr) {
            return;
        }

        std::queue<NodeType*> q;
        q.push(node);
        while (!q.empty()) {
            auto current = q.front();
            q.pop();
            std::cout << current->data_ << " ";
            if (current->left_child_) {
                q.push(current->left_child_);
            }
            if (current->right_child_) {
                q.push(current->right_child_);
            }
        }
    }

    NodeType* root_{nullptr};
};


#endif /* _BINARY_TREE_H */