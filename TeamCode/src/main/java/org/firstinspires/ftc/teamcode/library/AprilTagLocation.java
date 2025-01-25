package org.firstinspires.ftc.teamcode.library;

public class AprilTagLocation {
    public final int id;
    public final double x;
    public final double y;
    public final double angleCorrection;
    public AprilTagLocation(int id, double x, double y, double angleCorrection) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.angleCorrection = angleCorrection;
    }

    public String toString() {
        return "id:" + id + " x:" + String.format("%.1f", x) + " y:" + String.format("%.1f", y) + " ac: " + String.format("%.1f", angleCorrection);
    }

}
