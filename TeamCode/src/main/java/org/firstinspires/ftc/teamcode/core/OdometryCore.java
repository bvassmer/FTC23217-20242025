package org.firstinspires.ftc.teamcode.core;

import org.firstinspires.ftc.teamcode.library.GoBildaPinpointDriver;

import java.util.Locale;

public class OdometryCore extends DataCore {
    public int xOdoPosition, yOdoPosition;
    public int previousXOdoPosition, previousYRightOdoPosition;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        setupOdoComputer();
    }

    protected void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        if (autonomousMode) {
            updateOdometry();
            updateIsMoving();
        }
    }

    private void setupOdoComputer() throws InterruptedException {
         /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(302.22, -78.575);

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    public void updateOdometry() throws InterruptedException {
        /*
        Request an update from the Pinpoint odometry computer. This checks almost all outputs
        from the device in a single I2C read.
        */
        odo.update();

        /*
        gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
        */
        odoPoses.addAndCalculate(odo.getPosition());
        // String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

        /*
        gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
        */
        odoVelocityPoses.addAndCalculate(odo.getVelocity());
        // String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));


        /*
        Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
        READY: the device is working as normal
        CALIBRATING: the device is calibrating and outputs are put on hold
        NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
        FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
        FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
        FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
        */
        // telemetry.addData("Status", odo.getDeviceStatus());

        // telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

    }

    public void updateIsMoving() throws InterruptedException {
        // TODO: Move to PoseCircularBuffer as a check everytime a new Pose is added.
        int allowedDiff = 3;
        int xDiff = Math.abs(xOdoPosition - previousXOdoPosition);
        int yDiff = Math.abs(yOdoPosition - previousYRightOdoPosition);
        if (xDiff > allowedDiff || yDiff > allowedDiff ) {
            isMoving = true;
        } else {
            isMoving = false;
            // TODO: see how long this takes and if it is needed
            // odo.recalibrateIMU(); //recalibrates the IMU without resetting position
        }
    }
}

