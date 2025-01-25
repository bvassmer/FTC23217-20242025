package org.firstinspires.ftc.teamcode.library;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PoseCircularBuffer extends CircularBuffer {
    private List<Double> speeds;
    private double averageSpeed;
    private double averageBearing;
    public PoseCircularBuffer(int capacity) {
        super(capacity);
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
           this.calculateAverageBearing();
       }
    }

    private Double[] getXCoordinates() {
        Pose2D currentPoses[] = this.getFullArray();
        return Arrays.stream(currentPoses)
            .map(cb -> cb.getX(DistanceUnit.INCH))
            .toArray(Double[]::new);
    }
    private Double[] getYCoordinates() {
        Pose2D currentPoses[] = this.getFullArray();
        return Arrays.stream(currentPoses)
                .map(cb -> cb.getY(DistanceUnit.INCH))
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

    private void calculateAverageBearing() {
        if (this.size() > 0) {
            Pose2D currentPoses[] = this.getFullArray();
            this.averageBearing = Arrays.stream(currentPoses)
                    .map(cb -> cb.getHeading(AngleUnit.DEGREES))
                    .mapToDouble(Double::doubleValue)
                    .average().getAsDouble();
        }
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
