#pragma once

template <class T, int Size> struct CircularBuffer {
    constexpr static int Mask = Size - 1;

    T data[Size] = {T(0)};
    int head = 0;
    int tail = 0;

    void push(const T &value) {
        data[head & Mask] = value;
        head = (head + 1) & Mask;
        if (head == tail) {
            tail = (tail + 1) & Mask;
        }
    }

    bool pop(T &value) {
        if (head == tail) {
            return false;
        } else {
            value = data[tail & Mask];
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

    void clear() {
        head = 0;
        tail = 0;
    }

    const bool is_empty() const {
        return head == tail;
    }

    const T &top() const {
        return data[tail & Mask];
    }

    const T &bottom() const {
        return data[(head - 1) & Mask];
    }

    // Index from head: 0 = most recent, 1 = second most recent, etc.
    const T &operator[](int i) const {
        return data[(head - 1 - i) & Mask];
    }

    const int size() const {
        return (head - tail) & Mask;
    }

    struct Iterator {
        const T *data;
        int pos;

        const T &operator*() const {
            return data[pos & Mask];
        }
        Iterator &operator++() {
            pos = (pos + 1) & Mask;
            return *this;
        }
        Iterator &operator--() {
            pos = (pos - 1) & Mask;
            return *this;
        }
        bool operator!=(const Iterator &o) const {
            return pos != o.pos;
        }
    };

    Iterator begin() const {
        return {data, tail};
    }
    Iterator end() const {
        return {data, head};
    }
};
