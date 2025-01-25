package org.firstinspires.ftc.teamcode.library;

public class LinearVelocity {
    public double speed = 0.0;
    public double bearing = 0.0;
    public LinearVelocity(double speed, double bearing) {
        this.speed = speed;
        this.bearing = bearing;
    }

    public String toString() {
        return "Speed:" + speed + " Bearing:" + bearing;
    }
}
