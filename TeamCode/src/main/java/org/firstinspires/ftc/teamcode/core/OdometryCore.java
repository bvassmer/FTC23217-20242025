package org.firstinspires.ftc.teamcode.core;

public class OdometryCore extends HardwareCore {
    public int xOdoPosition, yOdoPosition;
    public int previousXOdoPosition, previousYRightOdoPosition;

    public void runOpMode(boolean autonomousMode) throws InterruptedException {
        super.runOpMode();
    }

    protected void workers(boolean enableController) throws InterruptedException {
        updateOdometry();
        updateIsMoving();
    }

    public void updateOdometry() throws InterruptedException {
        previousXOdoPosition = xOdoPosition;
        previousYRightOdoPosition = yOdoPosition;
        xOdoPosition = rearLeftMotor.getCurrentPosition();
        yOdoPosition = rearRightMotor.getCurrentPosition();
    }

    public void updateIsMoving() throws InterruptedException {
        int allowedDiff = 3;
        int xDiff = Math.abs(xOdoPosition - previousXOdoPosition);
        int yDiff = Math.abs(yOdoPosition - previousYRightOdoPosition);
        if (xDiff > allowedDiff || yDiff > allowedDiff ) {
            robotIsMoving = true;
        } else {
            robotIsMoving = false;
        }
    }
}

