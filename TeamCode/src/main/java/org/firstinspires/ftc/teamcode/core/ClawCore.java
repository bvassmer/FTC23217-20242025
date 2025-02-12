package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;

public class ClawCore extends LiftCore {
    protected boolean isClawTouching = false;
    public enum ClawPivotState {
        WAITING,
        MOVE_UP,
        MOVE_DOWN,
        MOVING,
        MOVE_TO_DROPOFF,
        MOVE_TO_PICKUP,
    };
    public enum ClawState {
        WAITING,
        OPEN,
        CLOSE,
    };
    public ClawPivotState clawPivotState = ClawPivotState.WAITING;
    public ClawState clawState = ClawState.WAITING;
    protected ElapsedTime pivotTimer = new ElapsedTime();
    final Double PIVOT_TIME = 1.0 / 240.0; // 1 degree takes 1/240 second = 0.004166666667 second
    // math below is for GoBilda Torque Servo
    // Max rotation 300 degrees from 0 to 1 servo distance
    // 0.25 sec / 60 degrees rotation speed (40 RPM)
    // 1 sec / 240 degrees rotation speed
    // 1 degree takes 1/240 second = 0.004166666667 second
    // 0.49 - 0.14 - 0.35 travel servo distance for claw pivot
    // 300 degrees * 0.35 travel distance = 105 degrees of travel
    final Double PIVOT_DISTANCE = 0.35 / 105.0; // 1 degree of travel is 0.35 / 105 = 0.003333333333 servo travel distance


    final double CLAW_CLOSED_POSITION = 0.75;
    final double CLAW_OPEN_POSITION = 0.63;
    final double CLAW_PIVOT_MIN_DOWN_POSITION = 0.14;
    final double CLAW_PIVOT_MAX_UP_POSITION = 0.49;
    final double CLAW_DROPOFF_POSITION = 0.49;
    final double CLAW_PICKUP_POSITION = 0.31;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    protected void workers() throws InterruptedException {
        super.workers();
        updateIsClawTouching();
        pivotClawStateMachine();
        clawStateMachine();
    }

    private void updateIsClawTouching() throws InterruptedException {
        isClawTouching = (isRightTouchSensorPressed || isLeftTouchSensorPressed);
    }

    private void pivotClawStateMachine() throws InterruptedException {
        switch (clawPivotState) {
            case WAITING:
                break;
            case MOVE_DOWN:
                pivotTimer.reset();
                if (clawPivotServoPosition - PIVOT_DISTANCE <= CLAW_PIVOT_MIN_DOWN_POSITION) {
                    clawPivotServoPosition = CLAW_PIVOT_MIN_DOWN_POSITION;
                    clawPivotServo.setPosition(clawPivotServoPosition);
                } else {
                    clawPivotServoPosition = clawPivotServoPosition - PIVOT_DISTANCE;
                    clawPivotServo.setPosition(clawPivotServoPosition);
                }
                clawPivotState = ClawPivotState.MOVING;
                break;
            case MOVE_UP:
                pivotTimer.reset();
                if (clawPivotServoPosition + PIVOT_DISTANCE >= CLAW_PIVOT_MAX_UP_POSITION) {
                    clawPivotServoPosition = CLAW_PIVOT_MAX_UP_POSITION;
                    clawPivotServo.setPosition(clawPivotServoPosition);
                } else {
                    clawPivotServoPosition = clawPivotServoPosition + PIVOT_DISTANCE;
                    clawPivotServo.setPosition(clawPivotServoPosition);
                }
                clawPivotState = ClawPivotState.MOVING;
                break;
            case MOVE_TO_DROPOFF:
                pivotTimer.reset();
                clawPivotServoPosition = CLAW_DROPOFF_POSITION;
                clawPivotServo.setPosition(clawPivotServoPosition);
                clawPivotState = ClawPivotState.MOVING;
                break;
            case MOVE_TO_PICKUP:
                pivotTimer.reset();
                clawPivotServoPosition = CLAW_PICKUP_POSITION;
                clawPivotServo.setPosition(clawPivotServoPosition);
                clawPivotState = ClawPivotState.MOVING;
                break;
            case MOVING:
                if (pivotTimer.seconds() > PIVOT_TIME) {
                    clawPivotState = ClawPivotState.WAITING;
                }
                break;
            default:
                // shouldn't get here.
                // TODO: error handling
        }
    }

    private void clawStateMachine() throws InterruptedException {
        switch (clawState) {
            case WAITING:
                break;
            case OPEN:
                clawServoPosition = CLAW_OPEN_POSITION;
                clawServo.setPosition(clawServoPosition);
                clawState = ClawState.WAITING;
                break;
            case CLOSE:
                clawServoPosition = CLAW_CLOSED_POSITION;
                clawServo.setPosition(clawServoPosition);
                clawState = ClawState.WAITING;
                break;
            default:
                // shouldn't get here.
                // TODO: error handling
        }
    }

}

