package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;

public class ExtensionCore extends CameraCore {
    private boolean autonomousMode = false;
    private Enum.TeamColor teamColor = Enum.TeamColor.BLUE;
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
    public final short SLIDE_HOOKER_MIN_SLOW = 190;
    public final short SLIDE_HOOKER_MIN = SLIDE_HOOKER_MIN_SLOW + 10; // Jeremy named this variable.
    public final short SLIDE_HOOKER_MAX = SLIDE_HOOKER_MIN + 5; // Jeremy named this variable.
    public final short SLIDE_HOOKER_MAX_SLOW = SLIDE_HOOKER_MAX + 10;
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
    public void runOpMode(boolean autonomousMode, Enum.TeamColor teamColor) throws InterruptedException {
        super.runOpMode(autonomousMode, teamColor);
        this.teamColor = teamColor;
        this.autonomousMode = autonomousMode;
    }

    protected void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        extensionStateMachine();
    }


    private void moveToDropoff() throws InterruptedException {
        if (robotState.slideTofCB.getAverage() < SLIDE_HOOKER_MIN_SLOW) {
            Log.d("FTC-23217-ExtensionCore", "moveToDropoff: Move up fast ");
            slideMotor.setPower(0.4);
        } else if (robotState.slideTofCB.getAverage() < SLIDE_HOOKER_MIN) {
            Log.d("FTC-23217-ExtensionCore", "moveToDropoff: Move up slow ");
            slideMotor.setPower(0.2);
        } else if  (robotState.slideTofCB.getAverage() > SLIDE_HOOKER_MAX_SLOW ) {
            Log.d("FTC-23217-ExtensionCore", "moveToDropoff: Move down fast ");
            slideMotor.setPower(-0.3);
        } else if (robotState.slideTofCB.getAverage() > SLIDE_HOOKER_MAX) {
            Log.d("FTC-23217-ExtensionCore", "moveToDropoff: Move down slow ");
            slideMotor.setPower(-0.15);
        } else {
            Log.d("FTC-23217-ExtensionCore", "moveToDropoff: SLIDE HOLDING ");
            slideState = SLIDE_STATE.HOLDING;
        }
    }

    private void moveToPickup() throws InterruptedException {
        if (robotState.slideTofCB.getAverage() >= SLIDE_PICKUP_HOOKER_MIN && robotState.slideTofCB.getAverage() <= SLIDE_PICKUP_HOOKER_MAX) {
            // in range
            slideState = SLIDE_STATE.HOLDING;
        } else if (robotState.slideTofCB.getAverage() > SLIDE_PICKUP_SLOW_MAX) {
            // too high, can move fast down
            slideMotor.setPower(-0.3);
        } else if (robotState.slideTofCB.getAverage() > SLIDE_PICKUP_HOOKER_MAX) {
            // too high, in slow zone. move slow down
            slideMotor.setPower(-0.15);
        } else if (robotState.slideTofCB.getAverage() < SLIDE_PICKUP_SLOW_MIN) {
            // too low, can move fast up
            slideMotor.setPower(0.4);
        } else if (robotState.slideTofCB.getAverage() < SLIDE_PICKUP_HOOKER_MIN) {
            // too low, in slow zone. move slow up
            slideMotor.setPower(0.2);
        } else {
            slideState = SLIDE_STATE.HOLDING;
        }
    }

    private void extensionStateMachine() throws InterruptedException {
        switch (slideState) {
            case HOLDING:
                slideMotor.setPower(0.05);
                break;
            case HANGING:
                slideMotor.setPower(-0.8);
                break;
            case AUTO_DROPOFF:
                moveToDropoff();
                break;
            case AUTO_PICKUP:
                moveToPickup();
                break;
            case MANUAL_MOVING:
                float y = gamepad2.left_stick_y;
                if (y == 0.0) {
                    stopMoving();
                } else {
                    slideMotor.setPower(-y * 0.8);
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

