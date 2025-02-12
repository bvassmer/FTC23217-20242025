package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.GoBildaPinpointDriver;

import java.util.Locale;
import java.util.Objects;

public class OdometryCore extends DataCore {
    public int xOdoPosition, yOdoPosition;
    public int previousXOdoPosition, previousYRightOdoPosition;
    private int runMax = 2;
    private int runCount = 0;

    public enum OdometryState {
        READ,
        WAIT,
    }

    private double ODOMETRY_DELAY_TIME = 0.05;
    private ElapsedTime odometryStateTimer = new ElapsedTime();
    public OdometryState odometryState = OdometryState.READ;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        setupOdoComputer();
        startTimesre();
    }

    protected void workers() throws InterruptedException {
        super.workers();
        if (autonomousMode || Boolean.TRUE.equals(MAP_DEBUG.get(DebugEnum.ODOMETERY))) {
            odometryStateMachine();
            updateIsMoving();
            /* if (runCount <= runMax) {
                if (Boolean.TRUE.equals(this.MAP_DEBUG.get(DebugEnum.TESTING))) {
                    switch (teamColor) {
                        case BLUE:
                            odo.setPosition(MAP_BLUE_POSE.get(POSE.TEST_START));
                            break;
                        case RED:
                            odo.setPosition(MAP_RED_POSE.get(POSE.TEST_START));
                            break;
                    }
                } else {
                    switch (teamColor) {
                        case BLUE:
                            odo.setPosition(MAP_BLUE_POSE.get(POSE.START));
                            break;
                        case RED:
                            odo.setPosition(MAP_RED_POSE.get(POSE.START));
                            break;
                    }
                }
                runCount += 1;
            } */
        }
    }

    private void startTimesre() {
        odometryStateTimer.startTime();
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
        // odo.setOffsets(302.22, -78.575);
        odo.setOffsets(88.601, -153.601);

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
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        // odo.recalibrateIMU();
        odo.resetPosAndIMU();
        sleep(500);
        odo.update();

        if (Boolean.TRUE.equals(this.MAP_DEBUG.get(DebugEnum.TESTING))) {
            switch (teamColor) {
                case BLUE:
                    odo.setPosition(Objects.requireNonNull(MAP_BLUE_POSE.get(POSE.TEST_START)));
                    break;
                case RED:
                    odo.setPosition(Objects.requireNonNull(MAP_RED_POSE.get(POSE.TEST_START)));
                    break;
            }
        } else {
            switch (teamColor) {
                case BLUE:
                    odo.setPosition(Objects.requireNonNull(MAP_BLUE_POSE.get(POSE.START)));
                    break;
                case RED:
                    odo.setPosition(Objects.requireNonNull(MAP_RED_POSE.get(POSE.START)));
                    break;
            }
        }
    }

    public void updateOdometry() throws InterruptedException {
        /*
        Request an update from the Pinpoint odometry computer. This checks almost all outputs
        from the device in a single I2C read.
        */
        Log.d("FTC-23217-OdometryCore", "Odometry Update");
        odo.update();

        /*
        gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
        */
        // odoPoses.addAndCalculate(odo.getPosition());
        // String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

        /*
        gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
        */
        // odoVelocityPoses.addAndCalculate(odo.getVelocity());
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

    private void odometryStateMachine() throws InterruptedException {
        switch (odometryState) {
            case READ:
                updateOdometry();
                odometryStateTimer.reset();
                odometryState = OdometryState.WAIT;
                break;
            case WAIT:
                if (odometryStateTimer.seconds() > ODOMETRY_DELAY_TIME ) {
                    odometryState = OdometryState.READ;
                }
                break;
        }
    }
}

