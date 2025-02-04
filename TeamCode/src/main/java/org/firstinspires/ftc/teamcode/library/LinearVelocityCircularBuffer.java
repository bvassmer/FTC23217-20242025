package org.firstinspires.ftc.teamcode.library;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LinearVelocityCircularBuffer extends CircularBuffer {
    protected final LinearVelocity[] buffer;    // Array to store elements
    public LinearVelocityCircularBuffer(int capacity) {
        super(capacity);
        this.buffer = new LinearVelocity[capacity];  // Type-safe cast
    }

    public void addAndCalculate(LinearVelocity element) {
       this.add(element);
       if (this.size > 0) {
       }
    }

    public LinearVelocity getLatestLinearVelocity(AngleUnit angleUnit) {
        return this.newestPeek();
    }

    // Returns the oldest element without removing it
    public LinearVelocity oldestPeek() {
        if (size == 0) {
            throw new IllegalStateException("Buffer is empty");
        }
        return buffer[head];
    }
    public LinearVelocity newestPeek() {
        if (size == 0) {
            throw new IllegalStateException("Buffer is empty");
        }
        return buffer[size - 1];
    }
    public LinearVelocity[] getFullArray() {
        LinearVelocity[] fullArray = new LinearVelocity[size];
        int currentIndex = head;
        for (int i = 0; i < size; i++) {
            fullArray[i] = buffer[currentIndex];
            currentIndex = (currentIndex + 1) % capacity;
        }
        return fullArray;
    }

    public String toString() {
        LinearVelocity latest = this.newestPeek();
        return "Speed:" + latest.speed + " Bearing:" + latest.bearing;
    }

}
