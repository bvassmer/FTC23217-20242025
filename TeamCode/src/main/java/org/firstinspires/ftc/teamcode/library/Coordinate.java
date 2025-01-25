package org.firstinspires.ftc.teamcode.library;

public class Coordinate {
    public final double x;
    public final double y;
    public Coordinate(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public String toString() {
        return "x:" + String.format("%.1f", x) + " y:" + String.format("%.1f", y);
    }
}
