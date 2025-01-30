package org.firstinspires.ftc.teamcode.library;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotState {
    public boolean isMoving = false;
    // currentPose is the current center of the robot.
    public PoseCircularBuffer robotPoses = new PoseCircularBuffer(8, 0, DistanceUnit.INCH, AngleUnit.DEGREES);
    public PoseCircularBuffer cameraPoses = new PoseCircularBuffer(8, 0, DistanceUnit.INCH, AngleUnit.DEGREES);
    public DoubleCircularBuffer bearingCB = new DoubleCircularBuffer(8, true, true);
    public DoubleCircularBuffer slideTofCB = new DoubleCircularBuffer(8, true, false);
    public LinearVelocityCircularBuffer cameraLinearVelocityCB = new LinearVelocityCircularBuffer(8);
    private Telemetry telemetry;
    public RobotState (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void calculate() throws InterruptedException {
        this.cameraLinearVelocityCB.addAndCalculate(new LinearVelocity(cameraPoses.getAverageSpeed(), cameraPoses.getAverageBearing()));
    }

    public void updateTelemetry(boolean updateTelemetry) {
        telemetry.addData("Pose (ROBOT)", "x:" + robotPoses.getLatestX() + " y:" + robotPoses.getLatestY() + " h:" + robotPoses.getLatestHeading());
        telemetry.addData("Pose (CAM)", "x:" + cameraPoses.getLatestX() + " y:" + cameraPoses.getLatestY() + " h:" + cameraPoses.getLatestHeading());
        telemetry.addData("Robot Bearing (CAM) (CB)", bearingCB.getAverage());
        telemetry.addData("Slide Distance (CB) (mm) ", slideTofCB.getAverage());
        telemetry.addData("Linear Velocity (CAM): ", cameraLinearVelocityCB.toString());
        if (updateTelemetry) {
            telemetry.update();
        }
    }
}
