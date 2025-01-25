package org.firstinspires.ftc.teamcode.core;

import android.app.WallpaperInfo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;

public class LiftCore extends ExtensionCore {
    private boolean autonomousMode = false;
    private Enum.TeamColor teamColor = Enum.TeamColor.BLUE;
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
    final Double ROTATION_TIME = 0.4; // in seconds. equals 0.2s/cycle or 5 Hz.
    final Double SAFE_LIFT_ROTATION_FRONT = 0.255;
    final Double SAFE_LIFT_REVERSE_ROTATION_FRONT = 0.73;
    final Double SAFE_LIFT_ROTATION_BACK = 0.58;
    final Double SAFE_LIFT_REVERSE_ROTATION_BACK = 0.41;
    final double CLAW_OPEN_POSITION = 0.6;
    public final double SLIDE_PICKUP_HOOKER_LIFT_ROTATION = 0.4117;
    public final double SLIDE_PICKUP_HOOKER_LIFT_REVERSE_ROTATION = 0.5867;
    public final double SLIDE_DROPOFF_HOOKER_LIFT_ROTATION = 0.47;
    public final double SLIDE_DROPOFF_HOOKER_LIFT_REVERSE_ROTATION = 0.52;
    public final double SLIDE_DROPOFF_RELEASE_HOOKER_LIFT_ROTATION = 0.40;
    public final double SLIDE_DROPIFF_RELEASE_HOOKER_LIFT_REVERSE_ROTATION = 0.60;
    public final double LIFT_ROTATION_PARK = 0.56;
    public final double LIFT_ROTATION_PARK_REVERSE = 0.43;

    @Override
    public void runOpMode(boolean autonomousMode, Enum.TeamColor teamColor) throws InterruptedException {
        super.runOpMode(autonomousMode, teamColor);
        this.teamColor = teamColor;
        this.autonomousMode = autonomousMode;
    }

    protected void workers(boolean enableController, LinearVelocity currentLinearVelocity, double desiredAngularMovement) throws InterruptedException {
        super.workers(enableController, currentLinearVelocity, desiredAngularMovement);
        telemetry.addData("Lift Pivot Servo Position", liftPivotServo.getPosition());
        telemetry.addData("Lift Pivot Servo Reverse Position", liftPivotServoReverse.getPosition());
        telemetry.addData("Lift Pivot State", rotationLiftState);
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
        if (liftPivotServoPosition < SLIDE_DROPOFF_RELEASE_HOOKER_LIFT_ROTATION  || liftPivotServoReversePosition > SLIDE_DROPIFF_RELEASE_HOOKER_LIFT_REVERSE_ROTATION) {
            slideState = SLIDE_STATE.AUTO_PICKUP;
        }
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
                if (liftPivotServoPosition + 0.02 < SAFE_LIFT_ROTATION_BACK) {
                    // SAFE, so allow movement
                    liftPivotServoPosition = liftPivotServo.getPosition() + 0.02;
                    liftPivotServoReversePosition = liftPivotServoReverse.getPosition() - 0.02;
                    liftPivotServo.setPosition(liftPivotServoPosition);
                    liftPivotServoReverse.setPosition(liftPivotServoReversePosition);
                    rotationLiftState = RotationLiftState.MOVING;
                } else if (liftPivotServoPosition + 0.02 >= SAFE_LIFT_ROTATION_BACK) {
                    // NOT SAFE, reset to MAX BACK.
                    rotationLiftState = RotationLiftState.MOVE_TO_MAX_BACK;
                }
                break;
            case MOVE_TO_FRONT:
                rotationTimer.reset();
                if (liftPivotServoPosition - 0.02 > SAFE_LIFT_ROTATION_FRONT) {
                    // SAFE, so allow movement
                    liftPivotServoPosition = liftPivotServo.getPosition() - 0.02;
                    liftPivotServoReversePosition = liftPivotServoReverse.getPosition() + 0.02;
                    liftPivotServo.setPosition(liftPivotServoPosition);
                    liftPivotServoReverse.setPosition(liftPivotServoReversePosition);
                    rotationLiftState = RotationLiftState.MOVING;
                } else if (liftPivotServoPosition - 0.02 <= SAFE_LIFT_ROTATION_FRONT) {
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
