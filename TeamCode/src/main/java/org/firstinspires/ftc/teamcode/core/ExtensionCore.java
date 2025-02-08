package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

public class ExtensionCore extends CameraCore {
    public enum SLIDE_STATE {
        HOLDING,
        MANUAL_MOVING,
        AUTO_DROPOFF,
        AUTO_PICKUP,
        HANGING,
    };

    public SLIDE_STATE slideState = SLIDE_STATE.HOLDING;
    public final short SLIDE_MIN = 168;
    public final short SLIDE_MIN_SLOW = 178;
    public final short SLIDE_HOOKER_MIN_SLOW = 245;
    public final short SLIDE_HOOKER_MIN = SLIDE_HOOKER_MIN_SLOW + 20; // Jeremy named this variable.
    public final short SLIDE_HOOKER_MAX = SLIDE_HOOKER_MIN + 5; // Jeremy named this variable.
    public final short SLIDE_HOOKER_MAX_SLOW = SLIDE_HOOKER_MAX + 20;
    public final double SLIDE_HOOKER_ROTATION = 0.48;
    public final double SLIDE_HOOKER_REVERSE_ROTATION = 0.52;
    public final short SLIDE_PICKUP_HOOKER_MIN = 22;
    public final short SLIDE_PICKUP_HOOKER_MAX = 26;
    public final short SLIDE_PICKUP_SLOW_MIN = SLIDE_PICKUP_HOOKER_MIN - 8;
    public final short SLIDE_PICKUP_SLOW_MAX = SLIDE_PICKUP_HOOKER_MAX + 8;
    public final double SLIDE_PICKUP_HOOKER_HOOK_ROTATION = 0.26;

    public final short SLIDE_MAX_SLOW = 290;
    public final short SLIDE_MAX = 300;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    protected void workers() throws InterruptedException {
        super.workers();
        extensionStateMachine();
    }


    private void moveToDropoff() throws InterruptedException {
        MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
        if (tofSlideSensorReading < SLIDE_HOOKER_MIN_SLOW) {
            Log.d("FTC-23217-ExtensionCore", "moveToDropoff: Move up fast ");
            slideMotor.setPower(0.8);
        } else if (tofSlideSensorReading < SLIDE_HOOKER_MIN) {
            Log.d("FTC-23217-ExtensionCore", "moveToDropoff: Move up slow ");
            slideMotor.setPower(0.2);
        } else if  (tofSlideSensorReading > SLIDE_HOOKER_MAX_SLOW ) {
            Log.d("FTC-23217-ExtensionCore", "moveToDropoff: Move down fast ");
            slideMotor.setPower(-0.6);
        } else if (tofSlideSensorReading > SLIDE_HOOKER_MAX) {
            Log.d("FTC-23217-ExtensionCore", "moveToDropoff: Move down slow ");
            slideMotor.setPower(-0.15);
        } else {
            Log.d("FTC-23217-ExtensionCore", "moveToDropoff: SLIDE HOLDING ");
            slideState = SLIDE_STATE.HOLDING;
        }
    }

    private void moveToPickup() throws InterruptedException {
        MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
        if (tofSlideSensorReading >= SLIDE_PICKUP_HOOKER_MIN && tofSlideSensorReading <= SLIDE_PICKUP_HOOKER_MAX) {
            // in range
            slideState = SLIDE_STATE.HOLDING;
        } else if (tofSlideSensorReading > SLIDE_PICKUP_SLOW_MAX) {
            // too high, can move fast down
            slideMotor.setPower(-0.6);
        } else if (tofSlideSensorReading > SLIDE_PICKUP_HOOKER_MAX) {
            // too high, in slow zone. move slow down
            slideMotor.setPower(-0.15);
        } else if (tofSlideSensorReading < SLIDE_PICKUP_SLOW_MIN) {
            // too low, can move fast up
            slideMotor.setPower(0.8);
        } else if (tofSlideSensorReading < SLIDE_PICKUP_HOOKER_MIN) {
            // too low, in slow zone. move slow up
            slideMotor.setPower(0.2);
        } else {
            slideState = SLIDE_STATE.HOLDING;
        }
    }

    private void extensionStateMachine() throws InterruptedException {
        switch (slideState) {
            case HOLDING:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, false);
                slideMotor.setPower(0.05);
                break;
            case HANGING:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, false);
                slideMotor.setPower(-0.8);
                break;
            case AUTO_DROPOFF:
                moveToDropoff();
                break;
            case AUTO_PICKUP:
                moveToPickup();
                break;
            case MANUAL_MOVING:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                float y = gamepad2.left_stick_y;
                if (y == 0.0) {
                    stopMoving();
                } else {
                    slideMotor.setPower(-y);
                }
                break;
            default:
        }
    }

    private void stopMoving() throws InterruptedException {
        slideMotor.setPower(-0.2);
        slideMotor.setPower(0);
        slideState = SLIDE_STATE.HOLDING;
    }

}

