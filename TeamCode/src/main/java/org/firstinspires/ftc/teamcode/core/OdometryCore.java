package org.firstinspires.ftc.teamcode.core;

public class OdometryCore extends HardwareCore {
    public int leftOdoPosition, rightOdoPosition, rearOdoPosition;
    public int previousLeftOdoPosition, previousRightOdoPosition, previousRearOdoPosition;

    public void runOpMode(boolean autonomousMode) throws InterruptedException {
        super.runOpMode();
    }

    protected void workers(boolean enableController) throws InterruptedException {
        updateOdometry();
        updateIsMoving();
    }

    public void updateOdometry() throws InterruptedException {
        previousRearOdoPosition = rearOdoPosition;
        previousLeftOdoPosition = leftOdoPosition;
        previousRightOdoPosition = rightOdoPosition;
        rearOdoPosition = backRightMotor.getCurrentPosition();
        leftOdoPosition = frontLeftMotor.getCurrentPosition();
        rightOdoPosition = carWashMotor.getCurrentPosition();
    }

    public void updateIsMoving() throws InterruptedException {
        int allowedDiff = 3;
        int rearDiff = Math.abs(rearOdoPosition - previousRearOdoPosition);
        int rightDiff = Math.abs(rightOdoPosition - previousRightOdoPosition);
        int leftDiff = Math.abs(leftOdoPosition - previousLeftOdoPosition);
        if (rearDiff > allowedDiff || rightDiff > allowedDiff || leftDiff > allowedDiff) {
            robotIsMoving = true;
        } else {
            robotIsMoving = false;
        }
    }
}

