package org.firstinspires.ftc.teamcode.core;

public class MechamCore extends TelemetryCore {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    public void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        if (enableController) {
            controllerActions();
        }
    }

    private void controllerActions() throws InterruptedException {
        mechamMovement();
    }

    private void mechamMovement() throws InterruptedException {
        double powerReducer = 1;
        double right_stick_x_reducer = 0.5;
        if (slideTofDistance > 400 && angryMantisTofDistance < 300) {
            powerReducer = 0.3;
            right_stick_x_reducer = 0.85;
        } else if (slideTofDistance > 400 && angryMantisTofDistance < 500) {
            powerReducer = 0.5;
            right_stick_x_reducer = 0.6;
        }

        // Original Code where left motors are revered.
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * right_stick_x_reducer;

        // modified code where right motors are reversed.
        /* double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x; */

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // power reducer is in place to help us learn how to drive. can be adjusted as needed.

        double frontLeftPower = (y + x + rx) / denominator * powerReducer;
        double backLeftPower = (y - x + rx) / denominator * powerReducer;
        double frontRightPower = (y - x - rx) / denominator * powerReducer;
        double backRightPower = (y + x - rx) / denominator * powerReducer;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);


    }
}
