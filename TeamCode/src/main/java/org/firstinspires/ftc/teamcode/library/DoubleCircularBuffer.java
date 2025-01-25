package org.firstinspires.ftc.teamcode.library;

import android.util.Log;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

public class DoubleCircularBuffer {
    protected Double[] buffer;    // Array to store elements
    protected int head = 0;        // Points to the oldest element
    protected int tail = 0;        // Points to the next available slot
    protected int size = 0;        // The current number of elements
    protected final int capacity;  // The maximum size of the buffer
    protected final int stdDevAllowed = 2;
    protected final int BEARING_THRESHOLD = 30;
    private double average = 0;
    private boolean removeOutliers = false;
    private boolean isBearing = false;

    public double getAverage() {
        return this.average;
    }

    // Constructor to initialize the buffer with a specific capacity
    public DoubleCircularBuffer(int capacity, boolean removeOutliers, boolean isBearing) {
        this.removeOutliers = removeOutliers;
        this.isBearing = isBearing;
        if (capacity <= 0) {
            throw new IllegalArgumentException("Capacity must be greater than 0");
        }
        this.capacity = capacity;
        this.buffer = new Double[capacity];  // Type-safe cast
    }

    public void addAndCalculate(Double element, int precision) {
        this.add(element);
        if (this.size > 0) {
            if (this.removeOutliers) {
                if (this.isBearing) {
                    this.removeBearingOutliers();
                } else {
                    this.removeNormalOutliers();
                }
            }
            this.calculateAverage(precision);
        }
    }

    // Adds an element to the buffer
    public void add(Double element) {
        if (size == capacity) {
            // If the buffer is full, overwrite the oldest element (circular behavior)
            head = (head + 1) % capacity;  // Move head forward to discard the oldest element
        } else {
            size++;
        }
        buffer[tail] = element;  // Place the new element at the tail
        tail = (tail + 1) % capacity;  // Move the tail forward
    }

    private void removeNormalOutliers() {
        double sum = 0;
        int count = 0;
        for (Double num : this.buffer) {
            if (num != null) {
                sum += num;
                count += 1;
            }
        }
        if (count > 0) {
            double mean = sum / count;
            double sumSquaredDifferences = 0;
            int sumSquaredCount = 0;
            for (int i = 0; i < this.size; i++) {
                if (this.buffer[i] != null) {
                    sumSquaredDifferences += Math.pow(this.buffer[i] - mean, 2);
                    sumSquaredCount += 1;
                }
            }
            double stdDev = Math.sqrt(sumSquaredDifferences / sumSquaredCount);

            ArrayList<Double> cleanedBuffer = new ArrayList<>();
            for (int i = 0; i < this.size; i++) {
                if (this.buffer[i] != null && Math.abs(this.buffer[i] - mean) <= stdDevAllowed * stdDev) {
                    cleanedBuffer.add(this.buffer[i]);
                } else {
                    cleanedBuffer.add(null);
                }
            }
            this.buffer = cleanedBuffer.toArray(new Double[this.capacity]);
        }
    }

    private void removeBearingOutliers() {
        double xSum = 0.0;
        double ySum = 0.0;

        int count = 0;
        for (int i = 0; i < this.size; i++) {
            if (this.buffer[i] != null) {
                double radians = Math.toRadians(this.buffer[i]);
                xSum += Math.cos(radians);
                ySum += Math.sin(radians);
                count += 1;
            }
        }

        if (count > 0) {
            double xMean = xSum / count;
            double yMean = ySum / count;
            double meanRadians = Math.atan2(yMean, xMean);
            double meanBearing = Math.toDegrees(meanRadians);
            if (meanBearing < 0) {
                meanBearing += 360;
            }

            ArrayList<Double> filteredBearings = new ArrayList<>();
            for (int i = 0; i < this.size; i++) {
                if (this.buffer[i] != null) {
                    double angularDistance = Math.abs(this.buffer[i] - meanBearing);
                    if (angularDistance > 180) {
                        angularDistance = 360 - angularDistance;
                    }

                    if (angularDistance < BEARING_THRESHOLD) {
                        filteredBearings.add(this.buffer[i]);
                    } else {
                        filteredBearings.add(null);
                    }
                } else {
                    filteredBearings.add(null);
                }
            }

            this.buffer = filteredBearings.toArray(new Double[this.capacity]);
        }
    }

    // Removes and returns the oldest element from the buffer
    public Double remove() {
        if (size == 0) {
            throw new IllegalStateException("Buffer is empty");
        }
        Double value = buffer[head];
        head = (head + 1) % capacity;  // Move the head forward
        size--;
        return value;
    }

    // Returns the oldest element without removing it
    public Double peek() {
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
    public Double[] getFullArray() {
        Double[] fullArray = (Double[]) new Object[size];
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

    private void calculateAverage(int precision) {
        if (isBearing) {
            calculateBearingAverage(precision);
        } else {
            calculateNormalAverage(precision);
        }
    }

    private void calculateBearingAverage(int precision) {
        double xSum = 0.0;
        double ySum = 0.0;

        if (this.size > 0) {
            int bearingCount = 0;
            for (Double bearing : this.buffer) {
                if (bearing != null) {
                    double radians = Math.toRadians(bearing);
                    xSum += Math.cos(radians);
                    ySum += Math.sin(radians);
                    bearingCount += 1;
                }
            }
            if (bearingCount > 0 ) {
                double xAvg = xSum / bearingCount;
                double yAvg = ySum / bearingCount;

                double avgRadians = Math.atan2(yAvg, xAvg);
                double avgDegrees = Math.toDegrees(avgRadians);

                if (avgDegrees < 0) {
                    avgDegrees += 360;
                }
                BigDecimal bigDecAvg = BigDecimal.valueOf(avgDegrees);
                this.average = bigDecAvg.setScale(precision, BigDecimal.ROUND_HALF_UP).doubleValue();
            }
        } else {
            this.average = 0;
        }
    }

    private void calculateNormalAverage(int precision) {
        double sum = 0;
        int count = 0;
        for (Double num : this.buffer) {
            if (num != null) {
                sum += num;
                count += 1;
            }
        }
        if (count > 0) {
            BigDecimal bigDecAvg = BigDecimal.valueOf(sum / count);
            this.average = bigDecAvg.setScale(precision, BigDecimal.ROUND_HALF_UP).doubleValue();
        } else {
            this.average = 0;
        }
    }
}