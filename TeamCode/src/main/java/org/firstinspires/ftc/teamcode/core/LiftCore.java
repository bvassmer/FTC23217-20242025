package org.firstinspires.ftc.teamcode.core;

import android.app.WallpaperInfo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;

public class LiftCore extends ExtensionCore {
    public enum RotationLiftState {
        WAITING,
        MOVE_TO_FRONT,
        MOVE_TO_BACK,
        MOVE_TO_MAX_FRONT,
        MOVE_TO_MAX_BACK,
        MOVE_TO_PICKUP,
        MOVE_TO_DROPOFF,
        DROPOFF,
        PARK,
        MOVING,
    }
    public RotationLiftState rotationLiftState = RotationLiftState.WAITING;
    protected ElapsedTime rotationTimer = new ElapsedTime();
    // math below is for this servo:
    // https://www.gobilda.com/stingray-5-servo-gearbox-0-85-sec-60-12-rpm-1750-oz-in-torque-360-rotation/
    // 0.58 - 0.255 = 0.325 servo range.
    // 117 degrees of motion. 48 degrees / second rotational speed estimate.
    // 1 degree / 1/48th second = 1 degree / 0.02083333333 second
    // 0.02 servo change on 360 degrees is 1 servo change is 7.2 degrees of motion.
    // 0.002777777778 (1 / 360) servo change is 1 degree of motion. 1 degree takes 0.02083333333 (1/48) second to turn.
    final Double ROTATION_TIME = 1.0 / 48.0; // 1 degree takes 0.02083333333 (1/48) second to turn.
    final Double ROTATION_STEP = 1.0 / 360.0;
    final Double SAFE_LIFT_ROTATION_FRONT = 0.255;
    final Double SAFE_LIFT_REVERSE_ROTATION_FRONT = 0.73;
    final Double SAFE_LIFT_ROTATION_BACK = 0.58;
    final Double SAFE_LIFT_REVERSE_ROTATION_BACK = 0.41;
    final double CLAW_OPEN_POSITION = 0.6;
    public final double SLIDE_PICKUP_HOOKER_LIFT_ROTATION = 0.3922;
    public final double SLIDE_PICKUP_HOOKER_LIFT_REVERSE_ROTATION = 0.5928;
    public final double SLIDE_DROPOFF_HOOKER_LIFT_ROTATION = 0.415; // 0.47 old
    public final double SLIDE_DROPOFF_HOOKER_LIFT_REVERSE_ROTATION = 0.57; // 0.52 old
    public final double SLIDE_DROPOFF_RELEASE_HOOKER_LIFT_ROTATION = 0.40;
    public final double SLIDE_DROPIFF_RELEASE_HOOKER_LIFT_REVERSE_ROTATION = 0.60;
    public final double LIFT_ROTATION_PARK = 0.56;
    public final double LIFT_ROTATION_PARK_REVERSE = 0.43;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    protected void workers() throws InterruptedException {
        super.workers();
        rotationStateMachine();
        startTimers();
    }

    private void startTimers() throws InterruptedException {
        rotationTimer.startTime();
    }

    private void moveToPickup() throws InterruptedException {
        liftPivotServoPosition = SLIDE_PICKUP_HOOKER_LIFT_ROTATION;
        liftPivotServoReversePosition = SLIDE_PICKUP_HOOKER_LIFT_REVERSE_ROTATION;
        liftPivotServo.setPosition(liftPivotServoPosition);
        liftPivotServoReverse.setPosition(liftPivotServoReversePosition);
    }

    private void moveToDropoff() throws InterruptedException {
        liftPivotServoPosition = SLIDE_DROPOFF_HOOKER_LIFT_ROTATION;
        liftPivotServoReversePosition = SLIDE_DROPOFF_HOOKER_LIFT_REVERSE_ROTATION;
        liftPivotServo.setPosition(liftPivotServoPosition);
        liftPivotServoReverse.setPosition(liftPivotServoReversePosition);
    }

    private void dropoff() throws InterruptedException {
        liftPivotServoPosition = SLIDE_DROPOFF_RELEASE_HOOKER_LIFT_ROTATION;
        liftPivotServoReversePosition = SLIDE_DROPIFF_RELEASE_HOOKER_LIFT_REVERSE_ROTATION;
        liftPivotServo.setPosition(liftPivotServoPosition);
        liftPivotServoReverse.setPosition(liftPivotServoReversePosition);
    }

    private void park() throws InterruptedException {
        liftPivotServoPosition = LIFT_ROTATION_PARK;
        liftPivotServoReversePosition = LIFT_ROTATION_PARK_REVERSE;
        liftPivotServo.setPosition(liftPivotServoPosition);
        liftPivotServoReverse.setPosition(liftPivotServoReversePosition);
    }

    private void rotationStateMachine() throws InterruptedException {
        switch (rotationLiftState) {
            case WAITING:
                break;
            case MOVE_TO_MAX_BACK:
                rotationTimer.reset();
                liftPivotServoPosition = SAFE_LIFT_ROTATION_BACK;
                liftPivotServoReversePosition = SAFE_LIFT_REVERSE_ROTATION_BACK;
                liftPivotServo.setPosition(liftPivotServoPosition);
                liftPivotServoReverse.setPosition(liftPivotServoReversePosition);
                rotationLiftState = RotationLiftState.MOVING;
                break;
            case MOVE_TO_MAX_FRONT:
                rotationTimer.reset();
                liftPivotServoPosition = SAFE_LIFT_ROTATION_FRONT;
                liftPivotServoReversePosition = SAFE_LIFT_REVERSE_ROTATION_FRONT;
                liftPivotServo.setPosition(liftPivotServoPosition);
                liftPivotServoReverse.setPosition(liftPivotServoReversePosition);
                rotationLiftState = RotationLiftState.MOVING;
                break;
            case MOVE_TO_BACK:
                rotationTimer.reset();
                if (liftPivotServoPosition + ROTATION_STEP < SAFE_LIFT_ROTATION_BACK) {
                    // SAFE, so allow movement
                    liftPivotServoPosition = liftPivotServoPosition +ROTATION_STEP;
                    liftPivotServoReversePosition = liftPivotServoReversePosition - ROTATION_STEP;
                    liftPivotServo.setPosition(liftPivotServoPosition);
                    liftPivotServoReverse.setPosition(liftPivotServoReversePosition);
                    rotationLiftState = RotationLiftState.MOVING;
                } else if (liftPivotServoPosition + ROTATION_STEP >= SAFE_LIFT_ROTATION_BACK) {
                    // NOT SAFE, reset to MAX BACK.
                    rotationLiftState = RotationLiftState.MOVE_TO_MAX_BACK;
                }
                break;
            case MOVE_TO_FRONT:
                rotationTimer.reset();
                if (liftPivotServoPosition - ROTATION_STEP > SAFE_LIFT_ROTATION_FRONT) {
                    // SAFE, so allow movement
                    liftPivotServoPosition = liftPivotServoPosition - ROTATION_STEP;
                    liftPivotServoReversePosition = liftPivotServoReversePosition + ROTATION_STEP;
                    liftPivotServo.setPosition(liftPivotServoPosition);
                    liftPivotServoReverse.setPosition(liftPivotServoReversePosition);
                    rotationLiftState = RotationLiftState.MOVING;
                } else if (liftPivotServoPosition - ROTATION_STEP <= SAFE_LIFT_ROTATION_FRONT) {
                    // NOT SAFE, so reset to MAX FRONT
                    rotationLiftState = RotationLiftState.MOVE_TO_MAX_FRONT;
                }
                break;
            case MOVE_TO_PICKUP:
                moveToPickup();
                rotationLiftState = RotationLiftState.WAITING;
               break;
            case MOVE_TO_DROPOFF:
                moveToDropoff();
                rotationLiftState = RotationLiftState.WAITING;
                break;
            case DROPOFF:
                dropoff();
                rotationLiftState = RotationLiftState.WAITING;
                break;
            case MOVING:
                // TODO: Readd limits around movement to prevent > 42" movement.
                /* if (liftPivotServoPosition != null && (liftPivotServoPosition < SLIDE_DROPOFF_RELEASE_HOOKER_LIFT_ROTATION  || liftPivotServoReversePosition > SLIDE_DROPIFF_RELEASE_HOOKER_LIFT_REVERSE_ROTATION)) {
                    slideState = SLIDE_STATE.AUTO_PICKUP;
                } */
                if (rotationTimer.seconds() > ROTATION_TIME) {
                    rotationLiftState = RotationLiftState.WAITING;
                }
                break;
            case PARK:
                park();
                rotationLiftState = RotationLiftState.WAITING;
                break;
            default:
                // shouldn't get here.
                // TODO: error handling
        }
    }
}
