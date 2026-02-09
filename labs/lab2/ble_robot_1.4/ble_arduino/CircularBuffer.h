#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H


template<class T, int Size>
struct CircularBuffer {
    constexpr static int Mask = Size - 1;

    T data[Size] = { 0 };
    int head = 0;
    int tail = 0;

    void push(const T& value) {
        data[head] = value;
        head = (head + 1) & Mask;
        if (head == tail) {
            tail = (tail + 1) & Mask;
        }
    }

    bool pop(T& value) {
        if (head == tail) {
            return false;
        } else {
            value = data[tail];
            tail = (tail + 1) & Mask;
            return true;
        }
    }

    bool pop() {
        if (head == tail) {
            return false;
        } else {
            tail = (tail + 1) & Mask;
            return true;
        }
    }

    const bool is_empty() const {
        return head == tail;
    }

    const T& top() const {
        return data[tail];
    }

    const T& bottom() const {
        return data[(head - 1) & Mask];
    }

    const int size() const {
        return (head - tail) & Mask;
    }
};



#endif // ROBOT_COMMAND_H
