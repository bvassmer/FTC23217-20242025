package org.firstinspires.ftc.teamcode.library;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PoseCircularBuffer extends CircularBuffer {
    protected final Pose2D[] buffer;    // Array to store elements
    private List<Double> speeds;
    private double averageSpeed;
    private double averageBearing;
    private AngleUnit angleUnit;
    private DistanceUnit distanceUnit;
    private int precision;
    public PoseCircularBuffer(int capacity, int precision, DistanceUnit distanceUnit, AngleUnit angleUnit) {
        super(capacity);
        this.distanceUnit = distanceUnit;
        this.angleUnit = angleUnit;
        this.precision = precision;
        this.buffer = (Pose2D[]) new Object[capacity];  // Type-safe cast
    }

    public double getAverageSpeed() {
        return this.averageSpeed;
    }
    public double getAverageBearing() {
        return this.averageBearing;
    }
    public List<Double> getSpeeds() {
        return this.speeds;
    }

    public void addAndCalculate(Pose2D element) {
       this.add(element);
       if (this.size > 0) {
           this.calculateSpeed();
           this.calculateAverageSpeed();
           this.calculateAverageBearing(this.precision, this.angleUnit);
       }
    }

    public double getLatestX() {
        return this.newestPeek().getX(this.distanceUnit);
    }
    public double getLatestY() {
        return this.newestPeek().getY(this.distanceUnit);
    }
    public double getLatestHeading() {
        return this.newestPeek().getHeading(this.angleUnit);
    }

    private Double[] getXCoordinates() {
        Pose2D currentPoses[] = this.getFullArray();
        return Arrays.stream(currentPoses)
            .map(cb -> cb.getX(this.distanceUnit))
            .toArray(Double[]::new);
    }
    private Double[] getYCoordinates() {
        Pose2D currentPoses[] = this.getFullArray();
        return Arrays.stream(currentPoses)
                .map(cb -> cb.getY(this.distanceUnit))
                .toArray(Double[]::new);
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
        if (this.speeds != null && this.speeds.size() > 0) {
            this.averageSpeed = this.speeds.stream()
                    .mapToDouble(Double::doubleValue)
                    .average().getAsDouble();
        }
    }

    private void calculateAverageBearing(int precision, AngleUnit angleUnit) {
        double xSum = 0.0;
        double ySum = 0.0;

        if (this.size > 0) {
            Double[] bearings = new Double[size];
            int currentIndex = head;

            for (int i = 0; i < size; i++) {
                bearings[i] = buffer[currentIndex].getHeading(angleUnit);
                currentIndex = (currentIndex + 1) % capacity;
            }
            int bearingCount = 0;

            for (Double bearing : bearings) {
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
                this.averageBearing = bigDecAvg.setScale(precision, BigDecimal.ROUND_HALF_UP).doubleValue();
            }
        } else {
            this.averageBearing = 0;
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
            fullArray[i] = (Pose2D) buffer[currentIndex];
            currentIndex = (currentIndex + 1) % capacity;
        }
        return fullArray;
    }

}
