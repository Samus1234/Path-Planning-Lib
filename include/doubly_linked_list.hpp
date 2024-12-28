#ifndef _LINKED_LIST_H_
#define _LINKED_LIST_H_

#include <iostream>
#include <algorithm>

template<typename DoublyLinkedList>
class DoublyLinkedListIterator {
    using ValueType = typename DoublyLinkedList::ValueType;
    using NodeType = typename DoublyLinkedList::NodeType;
public:
    // Iterator traits
    using iterator_category = std::bidirectional_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = ValueType;
    using pointer = ValueType*;
    using reference = ValueType&;

    DoublyLinkedListIterator() = default;
    DoublyLinkedListIterator(NodeType* m_ptr) : m_ptr_(m_ptr) {}
    ~DoublyLinkedListIterator() = default;

    DoublyLinkedListIterator& operator++() {
        m_ptr_ = m_ptr_->next_;
        return *this;
    }

    DoublyLinkedListIterator operator++(int) {
        DoublyLinkedListIterator iterator = *this;
        ++(*this);
        return iterator;
    }

    DoublyLinkedListIterator& operator--() {
        m_ptr_ = m_ptr_->prev_;
        return *this;
    }

    DoublyLinkedListIterator operator--(int) {
        DoublyLinkedListIterator iterator = *this;
        --(*this);
        return iterator;
    }

    reference operator*() const {
        return m_ptr_->data_;
    }

    pointer operator->() const {
        return &(m_ptr_->data_);
    }

    bool operator==(const DoublyLinkedListIterator& other) const {
        return m_ptr_ == other.m_ptr_;
    }

    bool operator!=(const DoublyLinkedListIterator& other) const {
        return m_ptr_ != other.m_ptr_;
    }

private:
    NodeType* m_ptr_{nullptr};
};


/// A generic node structure with data and a pointer
/// to the next node
/// @tparam T typename
template <typename T>
struct Node {
    T data_{}; ///< Data
    Node* next_{nullptr}; ///< Pointer to next node
    Node* prev_{nullptr}; ///< Pointer to previous node
    /// Default Constructor
    Node() = default;
    /// Initialization constructor
    /// @param data input data
    Node(const T& data) : data_(data) {}
    /// Move semantic initialization constructor
    /// @param data input data
    Node(T&& data) : data_(std::move(data)) {}
    /// Default destructor
    ~Node() = default;
};

/// Implementation of a doubly-linked list
/// @tparam T typename
template <typename T>
class DoublyLinkedList {
public:
    // Declare some type aliases
    using ValueType = T;
    using NodeType = Node<T>;
    using Iterator = DoublyLinkedListIterator<DoublyLinkedList<T>>;
    /// Default constructor
    DoublyLinkedList() = default;
    /// Destructor
    ~DoublyLinkedList() {
        clearList();
    }

    bool isEmpty() const {
        return head_ == nullptr;
    }

    const T& peekFront() const {
        if (isEmpty()) {
            throw std::runtime_error("Cannot Peek: List is empty");
        }
        return head_->data_;
    }

    const T& peekEnd() const {
        if (isEmpty()) {
            throw std::runtime_error("Cannot Peek: List is empty");
        }
        return tail_->data_;
    }

    void clearList() {
        Node<T>* temp = head_;
        while (temp) {
            Node<T>* delete_node = temp;
            temp = temp->next_;
            delete delete_node;
        }
        head_ = nullptr;
        tail_ = nullptr;
        size_ = 0;
    }

    size_t size() const {
        return size_;
    }

    /// Print every element of the list starting from head
    void printForward() const {
        std::cout << "nullptr -> ";
        for (auto it = begin(); it != end(); it++) {
            std::cout << *it << " -> ";
        }
        std::cout << "nullptr" << std::endl;
    }

    /// Print every element of the list starting from tail
    void printReverse() const {
        std::cout << "nullptr <- ";
        for (auto it = reverse_begin(); it != reverse_end(); it--) {
            std::cout << *it << " <- ";
        }
        std::cout << "nullptr" << std::endl;
    }

    void insertAtFront(const T& data) {
        Node<T>* new_node = new Node<T>(data);
        if (isEmpty()) {
            head_ = new_node;
            tail_ = new_node;
            size_++;
            return;
        }
        new_node->next_ = head_;
        head_->prev_ = new_node;
        head_ = new_node;
        size_++;
    }

    void insertAtFront(T&& data) {
        Node<T>* new_node = new Node<T>(std::move(data));
        if (isEmpty()) {
            head_ = new_node;
            tail_ = new_node;
            size_++;
            return;
        }
        new_node->next_ = head_;
        head_->prev_ = new_node;
        head_ = new_node;
        size_++;
    }

    void insertAtEnd(const T& data) {
        Node<T>* new_node = new Node<T>(data);
        if (isEmpty()) {
            head_ = new_node;
            tail_ = new_node;
            size_++;
            return;
        }
        new_node->prev_ = tail_;
        tail_->next_ = new_node;
        tail_ = new_node;
        size_++;
    }

    void insertAtEnd(T&& data) {
        Node<T>* new_node = new Node<T>(std::move(data));
        if (isEmpty()) {
            head_ = new_node;
            tail_ = new_node;
            size_++;
            return;
        }
        new_node->prev_ = tail_;
        tail_->next_ = new_node;
        tail_ = new_node;
        size_++;
    }

    void deleteAtFront() {
        if (isEmpty()) {
            tail_ = nullptr;
            size_ = 0;
            throw std::runtime_error("List is already empty!");
        }
        if (head_ == tail_) { // Only one element in the list
            delete head_;
            head_ = tail_ = nullptr;
            size_ = 0;
            return;
        }
        Node<T>* temp_node = head_;
        head_ = head_->next_;
        head_->prev_ = nullptr;
        delete temp_node;
        size_--;
    }

    void deleteAtEnd() {
        if (isEmpty()) {
            tail_ = nullptr;
            size_ = 0;
            throw std::runtime_error("List is already empty!");
        }
        if (head_ == tail_) { // Only one element in the list
            delete head_;
            head_ = tail_ = nullptr;
            size_ = 0;
            return;
        }
        Node<T>* temp_node = tail_;
        tail_ = tail_->prev_;
        tail_->next_ = nullptr;
        delete temp_node;
        size_--;
    }

    void insertAt(const T& data, Node<T>* node) {
        if (!node) {
            throw std::invalid_argument("Cannot insert at nullptr position.");
        }
        Node<T>* new_node = new Node<T>(data);
        new_node->next_ = node;
        new_node->prev_ = node->prev_;
        if (node->prev_) {
            node->prev_->next_ = new_node;
        } else {
            head_ = new_node;
        }
        size_++;
    }

    void insertAt(T&& data, Node<T>* node) {
        if (!node) {
            throw std::invalid_argument("Cannot insert at nullptr position.");
        }
        Node<T>* new_node = new Node<T>(std::move(data));
        new_node->next_ = node;
        new_node->prev_ = node->prev_;
        if (node->prev_) {
            node->prev_->next_ = new_node;
        } else {
            head_ = new_node;
        }
        size_++;
    }

    void deleteAt(Node<T>* node) {
        if (!node) {
            throw std::invalid_argument("Cannot delete a nullptr node.");
        }
        if (node->prev_) { // If node is not the first element
            node->prev_->next_ = node->next_;
        } else { // If node is the first element
            head_ = node->next_;
        }
        if (node->next_) { // If node is not the last element
            node->next_->prev_ = node->prev_;
        } else { // If node is the last element
            tail_ = node->prev_;
        }
        delete node;
        size_--;
    }

    Iterator begin() const {
        return Iterator(head_);
    }

    Iterator end() const {
        return Iterator(nullptr);
    }

    Iterator reverse_begin() const {
        return Iterator(tail_);
    }

    Iterator reverse_end() const {
        return Iterator(nullptr);
    }

public:
    size_t size_{0}; ///< Size of the list
    Node<T>* head_{nullptr}; ///< Head of the list
    Node<T>* tail_{nullptr}; ///< Tail of the list
};

#endif /* _LINKED_LIST_H_ */