#include <iostream>

/// A generic node structure with data and a pointer
/// to the next node
/// @tparam T typename
template <typename T>
struct Node {
    T data_{}; ///< Data
    Node* next_{nullptr}; ///< Pointer to next node

    Node() = default;
    Node(const T& data) : data_(data) {}
    Node(T&& data) : data_(std::move(data)) {}
    ~Node() = default;
};

/// Implementation of a singly-linked list
/// @tparam T typename
template <typename T>
class LinkedList {
public:
    /// Default constructor
    LinkedList() = default;
    /// Destructor to delete all heap data
    ~LinkedList() {
        Node<T>* temp = head_;
        while (temp) {
            Node<T>* delete_node = temp;
            temp = temp->next_;
            delete delete_node;
        }
        head_ = nullptr;
        tail_ = nullptr;
    }
    /// Check to see if the list is empty
    /// @return a true if the list is empty
    bool isEmpty() const {
        return head_ == nullptr;
    }
    /// Standard implementation
    void insertAtFront(T& data) {
        Node<T>* new_node = new Node<T>(data);
        // Check to see if list is empty
        if (isEmpty()) {
            head_ = new_node;
            tail_ = new_node;
            return;
        }
        // Move the current head to the next pointer of new_node
        new_node->next_ = head_;
        // Set the new head to new node
        head_ = new_node;
    }
    /// Move semantic implementation
    void insertAtFront(T&& data) {
        Node<T>* new_node = new Node<T>(std::move(data));
        // Check to see if list is empty
        if (isEmpty()) {
            head_ = new_node;
            tail_ = new_node;
            return;
        }
        // Move the current head to the next pointer of new_node
        new_node->next_ = head_;
        // Set the new head to new node
        head_ = new_node;
    }
    /// Insert at the end of the list
    void insertAtEnd(T& data) {
        Node<T>* new_node = new Node<T>(data);
        // Check to see if list is empty
        if (isEmpty()) {
            head_ = new_node;
            tail_ = new_node;
            return;
        }
        tail_->next_ = new_node;
        tail_ = new_node;
    }
    /// Insert at the end of the list
    void insertAtEnd(T&& data) {
        Node<T>* new_node = new Node<T>(std::move(data));
        // Check to see if list is empty
        if (isEmpty()) {
            head_ = new_node;
            tail_ = new_node;
            return;
        }
        tail_->next_ = new_node;
        tail_ = new_node;
    }
    /// Delete element at the front of list
    void deleteAtFront() {
        if (isEmpty()) {
            std::cout << "List already empty" << std::endl;
            tail_ = nullptr;
            return;
        }

        Node<T>* delete_node = head_;
        head_ = delete_node->next_;
        delete delete_node;
        
    }
    /// Get the data of the head node
    /// @return the value of the head node
    const T& peekFront() const {
        if (isEmpty()) {
            throw std::runtime_error("Cannot peek: List is empty");
        }
        return head_->data_;
    }
    /// Get the data of the head node
    /// @return the value of the head node
    const T& peekEnd() const {
        if (isEmpty()) {
            throw std::runtime_error("Cannot peek: List is empty");
        }
        return tail_->data_;
    }
    /// Clears list
    void clear() {
        while (!isEmpty()) {
            deleteAtFront();
        }
    }
    /// Print every element of the list
    void printList() const {
        Node<T>* temp = head_;
        while (temp) {
            std::cout << temp->data_ << " -> ";
            temp = temp->next_;
        }
        std::cout << "nullptr" << std::endl;
    }

private:
    Node<T>* head_{nullptr}; ///< Head of the list
    Node<T>* tail_{nullptr}; ///< Tail of the list
};



int main(int argc, char** argv) {

    std::cout << "\n";

    LinkedList<int> list;

    for (int i = 0; i < 10; i++) {
        list.insertAtFront(i);
    }

    for (int i = 0; i < 4; i++) {
        list.deleteAtFront();
    }

    for (int i = 0; i < 8; i++) {
        list.insertAtEnd(i);
    }

    list.printList();

    std::cout << "\n";

    std::cout << "Front of list: " << list.peekFront() << "\n";;
    std::cout << "End of list: " << list.peekEnd() << std::endl;
    

    return 0;
}