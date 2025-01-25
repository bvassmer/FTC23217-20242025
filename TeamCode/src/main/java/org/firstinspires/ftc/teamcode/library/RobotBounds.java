package org.firstinspires.ftc.teamcode.library;

public class RobotBounds {
    Coordinate upperRight = new Coordinate(0, 0);
    Coordinate upperLeft = new Coordinate(0, 0);
    Coordinate lowerLeft = new Coordinate(0, 0);
    Coordinate lowerRight = new Coordinate(0, 0);
    public RobotBounds(Coordinate upperRight, Coordinate upperLeft, Coordinate lowerLeft, Coordinate lowerRight) {
        this.upperRight = upperRight;
        this.upperLeft = upperLeft;
        this.lowerLeft = lowerLeft;
        this.lowerRight = lowerRight;
    }

    public String toString() {
        return "UR:" + upperRight.toString() + " UL:" + upperLeft.toString() + " LL:" + lowerLeft.toString() + " LR:" + lowerRight.toString();
    }
}
