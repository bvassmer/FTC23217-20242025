package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftCore extends DroneCore {
    public enum LiftState {
        WAITING,
        RAISING,
        AUTO_RAISE,
        AUTO_LOWER,
        AUTO_RAISING,
        AUTO_LOWERING,
        LOWERING,
        HOLDING,
    }
    public LiftState liftState = LiftState.WAITING;
    protected ElapsedTime autoLiftTimer = new ElapsedTime();
    final Double LIFT_TIME = 1.1; // in seconds. equals 0.2s/cycle or 5 Hz.
    final Double LOWER_TIME = 1.1; // in seconds. equals 0.2s/cycle or 5 Hz.

    @Override
    public void runOpMode(boolean autonomousMode) throws InterruptedException {
        super.runOpMode(autonomousMode);
        lowerHook();
    }

    protected void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        if (enableController) {
            controllerActions();
        }
        stateMachine();
    }

    private void controllerActions() throws InterruptedException {
        liftArmMovement();
        hookMovement();
    }


    private void liftArmMovement() throws InterruptedException {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        if (up) {
            liftState = LiftState.RAISING;
        } else if (down) {
            liftState = LiftState.LOWERING;
        } else if (liftState != LiftState.AUTO_LOWERING
                && liftState != LiftState.AUTO_RAISING
                && liftState != LiftState.AUTO_RAISE
                && liftState != LiftState.AUTO_LOWER) {
            liftState = LiftState.WAITING;
        }
    }

    private void hookMovement() throws InterruptedException {
        if (gamepad1.a) {
            lowerHook();
        }
        if (gamepad1.y) {
            raiseHook();
        }
    }

    public void raiseHook() throws InterruptedException {
        hookServo.setPosition(0.023);
    }

    public void lowerHook() throws InterruptedException {
        hookServo.setPosition(0.35);
    }

    private void stateMachine() throws InterruptedException {
        // Currently updates readings every 0.2 seconds.
        switch (liftState) {
            case WAITING:
                liftMotor.setPower(0);
                break;
            case LOWERING:
                liftMotor.setPower(-1);
                break;
            case RAISING:
                liftMotor.setPower(1);
                break;
            case AUTO_LOWER:
                autoLiftTimer.reset();
                liftMotor.setPower(-0.8);
                liftState = LiftState.AUTO_LOWERING;
                break;
            case AUTO_RAISE:
                autoLiftTimer.reset();
                liftMotor.setPower(1);
                liftState = LiftState.AUTO_RAISING;
                break;
            case AUTO_LOWERING:
                if (autoLiftTimer.seconds() > LOWER_TIME) {
                    liftState = LiftState.WAITING;
                }
                break;
            case AUTO_RAISING:
                if (autoLiftTimer.seconds() > LIFT_TIME) {
                    liftState = LiftState.HOLDING;
                }
                break;
            case HOLDING:
               liftMotor.setPower(0.01);
                break;
            default:
                // shouldn't get here.
                // TODO: error handling
        }
    }
}
