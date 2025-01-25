package org.firstinspires.ftc.teamcode.library;

public class CircularBuffer<T> {
    protected final T[] buffer;    // Array to store elements
    protected int head = 0;        // Points to the oldest element
    protected int tail = 0;        // Points to the next available slot
    protected int size = 0;        // The current number of elements
    protected final int capacity;  // The maximum size of the buffer

    // Constructor to initialize the buffer with a specific capacity
    public CircularBuffer(int capacity) {
        if (capacity <= 0) {
            throw new IllegalArgumentException("Capacity must be greater than 0");
        }
        this.capacity = capacity;
        this.buffer = (T[]) new Object[capacity];  // Type-safe cast
    }

    // Adds an element to the buffer
    public void add(T element) {
        if (size == capacity) {
            // If the buffer is full, overwrite the oldest element (circular behavior)
            head = (head + 1) % capacity;  // Move head forward to discard the oldest element
        } else {
            size++;
        }
        buffer[tail] = element;  // Place the new element at the tail
        tail = (tail + 1) % capacity;  // Move the tail forward
    }

    // Removes and returns the oldest element from the buffer
    public T remove() {
        if (size == 0) {
            throw new IllegalStateException("Buffer is empty");
        }
        T value = buffer[head];
        head = (head + 1) % capacity;  // Move the head forward
        size--;
        return value;
    }

    // Returns the oldest element without removing it
    public T peek() {
        if (size == 0) {
            throw new IllegalStateException("Buffer is empty");
        }
        return buffer[head];
    }

    // Returns the current size of the buffer
    public int size() {
        return size;
    }

    // Checks if the buffer is empty
    public boolean isEmpty() {
        return size == 0;
    }

    // Checks if the buffer is full
    public boolean isFull() {
        return size == capacity;
    }

    // Clears the buffer
    public void clear() {
        head = 0;
        tail = 0;
        size = 0;
    }

    // Returns the full array of elements in correct order (from head to tail)
    public T[] getFullArray() {
        T[] fullArray = (T[]) new Object[size];
        int currentIndex = head;
        for (int i = 0; i < size; i++) {
            fullArray[i] = buffer[currentIndex];
            currentIndex = (currentIndex + 1) % capacity;
        }
        return fullArray;
    }

    // Returns a string representation of the buffer (for debugging)
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("[");
        for (int i = 0; i < size; i++) {
            sb.append(buffer[(head + i) % capacity]);
            if (i < size - 1) sb.append(", ");
        }
        sb.append("]");
        return sb.toString();
    }
}