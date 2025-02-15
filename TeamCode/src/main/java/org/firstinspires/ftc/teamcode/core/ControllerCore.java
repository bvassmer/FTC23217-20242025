package org.firstinspires.ftc.teamcode.core;

public class ControllerCore extends ClawCore {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    public void workers() throws InterruptedException {
        super.workers();
        if (this.enableController) {
            runControllerOne();
            runControllerTwo();
        }
    }

    private void runControllerOne() throws InterruptedException {
        boolean xButton = gamepad1.x;
        boolean bButton = gamepad1.b;
        if (xButton) {
            slideState = SLIDE_STATE.HANGING;
        }
        if (bButton) {
            movementLockOn = false;
        }
    }

    private void runControllerTwo() throws InterruptedException{
        float y = gamepad2.left_stick_y;
        boolean yButton = gamepad2.y;
        boolean xButton = gamepad2.x;
        boolean aButton = gamepad2.a;
        boolean bButton = gamepad2.b;

        // Extension Arm manual movement.
        if (y != 0.0 && slideState != SLIDE_STATE.FIX_OUTSIDE_BOUNDARIES) {
            slideState = SLIDE_STATE.MANUAL_MOVING;
        }

        // Extension Arm presets.
        if (bButton && rotationLiftState == RotationLiftState.WAITING) {
            rotationLiftState = RotationLiftState.MOVE_TO_FRONT;
        } else if (xButton && rotationLiftState == RotationLiftState.WAITING) {
            rotationLiftState = RotationLiftState.MOVE_TO_BACK;
        } else if (gamepad2.dpad_down && rotationLiftState == RotationLiftState.WAITING) {
            rotationLiftState = RotationLiftState.MOVE_TO_PICKUP;
            slideState = SLIDE_STATE.AUTO_PICKUP;
            clawPivotState = ClawPivotState.MOVE_TO_PICKUP;
            clawState = ClawState.OPEN;
        } else if (gamepad2.dpad_up && rotationLiftState == RotationLiftState.WAITING) {
            rotationLiftState = RotationLiftState.MOVE_TO_DROPOFF;
            slideState = SLIDE_STATE.AUTO_DROPOFF;
            clawPivotState = ClawPivotState.MOVE_TO_DROPOFF;
        }

        // Claw
        if (aButton && clawPivotState == ClawPivotState.WAITING) {
            clawPivotState = ClawPivotState.MOVE_DOWN;
        } else if (yButton && clawPivotState == ClawPivotState.WAITING) {
            clawPivotState = ClawPivotState.MOVE_UP;
        }

        // hook open and close
        if (gamepad2.right_bumper && clawState == ClawState.WAITING) {
            clawState = ClawState.CLOSE;
        } else if (gamepad2.left_bumper && clawState == ClawState.WAITING) {
            clawState = ClawState.OPEN;
        }
    }
}
