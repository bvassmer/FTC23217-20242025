package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;

public class ClawCore extends LiftCore {
    private boolean autonomousMode = false;
    private Enum.TeamColor teamColor = Enum.TeamColor.BLUE;

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
    final Double PIVOT_TIME = 0.2; // in seconds. equals 0.2s/cycle or 5 Hz.

    final double CLAW_CLOSED_POSITION = 0.7;
    final double CLAW_OPEN_POSITION = 0.6;
    final double CLAW_PIVOT_MIN_DOWN_POSITION = 0.14;
    final double CLAW_PIVOT_MAX_UP_POSITION = 0.49;
    final double CLAW_DROPOFF_POSITION = 0.49;
    final double CLAW_PICKUP_POSITION = 0.29;

    @Override
    public void runOpMode(boolean autonomousMode, Enum.TeamColor teamColor) throws InterruptedException {
        super.runOpMode(autonomousMode, teamColor);
        this.teamColor = teamColor;
        this.autonomousMode = autonomousMode;
    }

    protected void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        pivotClawStateMachine();
        clawStateMachine();
    }

    private void pivotClawStateMachine() throws InterruptedException {
        switch (clawPivotState) {
            case WAITING:
                clawPivotServoPosition = clawPivotServo.getPosition();
                break;
            case MOVE_DOWN:
                pivotTimer.reset();
                clawPivotServoPosition = clawPivotServo.getPosition();
                if (clawPivotServoPosition - 0.03 <= CLAW_PIVOT_MIN_DOWN_POSITION) {
                    clawPivotServoPosition = CLAW_PIVOT_MIN_DOWN_POSITION;
                    clawPivotServo.setPosition(clawPivotServoPosition);
                } else {
                    clawPivotServoPosition = clawPivotServoPosition - 0.03;
                    clawPivotServo.setPosition(clawPivotServoPosition);
                }
                clawPivotState = ClawPivotState.MOVING;
                break;
            case MOVE_UP:
                pivotTimer.reset();
                clawPivotServoPosition = clawPivotServo.getPosition();
                if (clawPivotServoPosition + 0.03 >= CLAW_PIVOT_MAX_UP_POSITION) {
                    clawPivotServoPosition = CLAW_PIVOT_MAX_UP_POSITION;
                    clawPivotServo.setPosition(clawPivotServoPosition);
                } else {
                    clawPivotServoPosition = clawPivotServoPosition + 0.03;
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
                clawServoPosition = clawServo.getPosition();
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

