package org.firstinspires.ftc.teamcode.library;

public class AngularVelocity {
    public double speed = 0.0; // degrees / sec
    public AngularVelocity(double speed) {
        this.speed = speed;
    }

    public String toString() {
        return "Speed:" + speed;
    }
}
