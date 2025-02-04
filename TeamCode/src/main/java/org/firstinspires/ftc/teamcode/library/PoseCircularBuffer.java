package org.firstinspires.ftc.teamcode.library;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.opencv.core.Mat;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PoseCircularBuffer extends CircularBuffer {
    protected final Pose2D[] buffer;    // Array to store elements
    private List<Double> speeds;
    private double averageSpeed;
    private double averageX;
    private double averageY;
    private double averageHeading;
    private double stdDevX = 100;
    private double stdDevY = 100;
    private double stdDevHeading = 100;
    private AngleUnit angleUnit;
    private DistanceUnit distanceUnit;
    private int precision;
    public PoseCircularBuffer(int capacity, int precision, DistanceUnit distanceUnit, AngleUnit angleUnit) {
        super(capacity);
        this.distanceUnit = distanceUnit;
        this.angleUnit = angleUnit;
        this.precision = precision;
        this.buffer = new Pose2D[capacity];
    }

    public double getAverageSpeed() {
        return this.averageSpeed;
    }
    public double getAverageX() {
        return this.averageX;
    }
    public double getAverageY() {
        return this.averageY;
    }
    public double getAverageHeading() {
        return this.averageHeading;
    }
    public double getStdDevX() {
        return this.stdDevX;
    }
    public double getStdDevY() {
        return this.stdDevY;
    }
    public double getStdDevHeading() {
        return this.stdDevHeading;
    }
    public List<Double> getSpeeds() {
        return this.speeds;
    }

    // Adds an element to the buffer
    public void add(Pose2D element) {
        if (size == capacity) {
            // If the buffer is full, overwrite the oldest element (circular behavior)
            head = (head + 1) % capacity;  // Move head forward to discard the oldest element
        } else {
            size++;
        }
        buffer[tail] = element;  // Place the new element at the tail
        tail = (tail + 1) % capacity;  // Move the tail forward
    }

    public void addAndCalculate(Pose2D element) {
       this.add(element);
       if (this.size > 0) {
           this.calculateAverageX();
           this.calculateAverageY();
           this.calculateAverageHeading(this.precision, this.angleUnit);
           this.calculateSpeed();
           this.calculateAverageSpeed();
           this.calculateStdDev();
       }
    }

    public double getLatestX() {
        return this.newestPeek().getX(this.distanceUnit);
    }
    public double getLatestY() {
        return this.newestPeek().getY(this.distanceUnit);
    }
    public double getLatestHeading() {
        double rawHeading = this.newestPeek().getHeading(this.angleUnit);
        return (rawHeading + 360) % 360; // normalize
    }

    public Double[] getXCoordinates() {
        Pose2D currentPoses[] = this.getFullArray();
        return Arrays.stream(currentPoses)
            .map(cb -> cb.getX(this.distanceUnit))
            .toArray(Double[]::new);
    }
    public Double[] getYCoordinates() {
        Pose2D currentPoses[] = this.getFullArray();
        return Arrays.stream(currentPoses)
                .map(cb -> cb.getY(this.distanceUnit))
                .toArray(Double[]::new);
    }

    public Double[] getHeadings() {
        Pose2D currentPoses[] = this.getFullArray();
        return Arrays.stream(currentPoses)
                .map(cb -> cb.getHeading(this.angleUnit))
                .toArray(Double[]::new);
    }
    private void calculateAverageX() {
        if (this.size > 0) {
            Double[] x = getXCoordinates();
            Double sumX = 0.0;
            for (int i = 0; i < x.length; i++) {
                sumX += x[i];
            }
            averageX = sumX / this.size;
        }
    }
    private void calculateAverageY() {
        if (this.size > 0) {
            Double[] y = getYCoordinates();
            Double sumY = 0.0;
            for (int i = 0; i < y.length; i++) {
                sumY += y[i];
            }
            averageY = sumY / this.size;
        }
    }

    private void calculateAverageHeading(int precision, AngleUnit angleUnit) {
        double sumSin = 0.0;
        double sumCos = 0.0;

        if (this.size > 0) {
            int bearingCount = 0;
            Double[] allHeadings = this.getHeadings();
            for (Double bearing : allHeadings) {
                if (bearing != null) {
                    double radians = Math.toRadians(bearing);
                    sumSin += Math.sin(radians);
                    sumCos += Math.cos(radians);
                    bearingCount += 1;
                }
            }
            if (bearingCount > 0 ) {
                double radianAvg = Math.atan2(sumSin / bearingCount, sumCos / bearingCount);
                double degreesUncleanAvg = Math.toDegrees(radianAvg);
                this.averageHeading = (degreesUncleanAvg + 360) % 360;
            } else {
                this.averageHeading = 0;
            }
        } else {
            this.averageHeading = 0;
        }
    }

    private void calculateStdDev() {
        if (this.size == capacity) {
            double squaredSumX = 0.0;
            for (double numX : getXCoordinates()) {
                squaredSumX += Math.pow(numX - this.averageX, 2);
            }
            this.stdDevX = Math.sqrt(squaredSumX / this.size);

            double squaredSumY = 0.0;
            for (double numY : getYCoordinates()) {
                squaredSumY += Math.pow(numY - this.averageY, 2);
            }
            this.stdDevY = Math.sqrt(squaredSumY / this.size);

            double sumSin = 0.0;
            double sumCos = 0.0;
            for (double numHeading : getHeadings()) {
                sumSin += Math.sin(Math.toRadians(numHeading));
                sumCos += Math.cos(Math.toRadians(numHeading));
            }
            double R = Math.sqrt(sumSin * sumSin + sumCos * sumCos) / this.size;
            this.stdDevHeading = Math.sqrt(-2 * Math.log(R));
        }
    }


    public void calculateSpeed() {
        Double[] x = getXCoordinates();
        Double[] y = getYCoordinates();

        if (x.length != y.length || x.length < 2) {
            // Check if the arrays are of the same length and have more than 1 point
        } else {
            List<Double> speeds = new ArrayList<>();  // We will calculate speed between successive points

            // Iterate over the array to calculate speed between consecutive points
            for (int i = 0; i < x.length - 1; i++) {
                // Calculate the distance between points (x[i], y[i]) and (x[i+1], y[i+1])
                Double distance = Math.sqrt(Math.pow(x[i + 1] - x[i], 2) + Math.pow(y[i + 1] - y[i], 2));

                // Assuming time interval is 1 between successive points, so speed = distance
                speeds.add(distance);
            }

            this.speeds = speeds;  // Return an array of calculated speeds
        }

    }

    private void calculateAverageSpeed() {
        if (this.speeds != null && !this.speeds.isEmpty()) {
            this.averageSpeed = this.speeds.stream()
                    .mapToDouble(Double::doubleValue)
                    .average().getAsDouble();
        }
    }


    // Returns the oldest element without removing it
    public Pose2D oldestPeek() {
        if (size == 0) {
            throw new IllegalStateException("Buffer is empty");
        }
        return buffer[head];
    }
    public Pose2D newestPeek() {
        if (size == 0) {
            throw new IllegalStateException("Buffer is empty");
        }
        return buffer[size - 1];
    }
    public Pose2D[] getFullArray() {
        Pose2D[] fullArray = new Pose2D[size];
        int currentIndex = head;
        for (int i = 0; i < size; i++) {
            fullArray[i] = buffer[currentIndex];
            currentIndex = (currentIndex + 1) % capacity;
        }
        return fullArray;
    }

}
