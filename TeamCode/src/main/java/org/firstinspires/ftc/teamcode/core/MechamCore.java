package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;

public class MechamCore extends TelemetryCore {
    private boolean autonomousMode = false;
    private boolean debugMode = false;
    private Enum.TeamColor teamColor = Enum.TeamColor.BLUE;
    private Double powerReduction = 0.4;
    public boolean isMoving = false;

    @Override
    public void runOpMode(boolean autonomousMode, Enum.TeamColor teamColor) throws InterruptedException {
        Log.d("FTC-23217-MechamCore", "MechamCore Start.");
        super.runOpMode(autonomousMode, teamColor);
        this.teamColor = teamColor;
        this.autonomousMode = autonomousMode;
    }

    public void workers(boolean enableController, double x, double y, double rx, boolean debugMode) throws InterruptedException {
        this.debugMode = debugMode;
        super.workers(enableController);
        if (autonomousMode) {
            autoMechamMovement(x, y, rx);
        }
        if (enableController) {
            mechamMovement();
        }
    }

    public void stopMovement() throws InterruptedException {
        frontLeftMotor.setPower(0.0);
        rearLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        rearRightMotor.setPower(0.0);
    }

    private void autoMechamMovement(double x, double y, double rx) throws InterruptedException {
        double fixedY = -y;
        double fixedX = -x;
        double fixedRx = -rx;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(fixedY) + Math.abs(fixedX) + Math.abs(fixedRx), 1);

        if (y != 0 || x != 0 || rx != 0) {
            isMoving = true;
        } else {
            isMoving = false;
        }
        // power reducer is in place to help us learn how to drive. can be adjusted as needed.
        Log.d("FTC-23217-autoMechamMovement", "isMoving:" + isMoving + " x:" + fixedX + " y:" + fixedY + " rx:" + fixedRx);

        double frontLeftPower = -(fixedY + fixedX + fixedRx) / denominator;
        double rearLeftPower = (fixedY - fixedX + fixedRx) / denominator;
        double frontRightPower = -(fixedY - fixedX - fixedRx) / denominator;
        double rearRightPower = (fixedY + fixedX - fixedRx) / denominator;

        if (!debugMode) {
            frontLeftMotor.setPower(frontLeftPower);
            rearLeftMotor.setPower(rearLeftPower);
            frontRightMotor.setPower(frontRightPower);
            rearRightMotor.setPower(rearRightPower);
        }
    }

    private void mechamMovement() throws InterruptedException {
        double right_stick_x_reducer = 0.85;

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x * right_stick_x_reducer;
        double rt = gamepad1.right_trigger;
        if (rt > 0.03) {
            // turbo mode
            powerReduction = rt;
        } else {
            powerReduction = 0.4;
        }

        if (y != 0 || x != 0 || rx != 0) {
            isMoving = true;
        } else {
            isMoving = false;
        }
        Log.d("FTC-23217-mechamMovement", "isMoving:" + isMoving + " x:" + x + " y:" + y + " rx:" + rx + " rt:" + rt);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = powerCurve(-(y + x + rx) / denominator);
        double rearLeftPower = powerCurve((y - x + rx) / denominator);
        double frontRightPower = powerCurve(-(y - x - rx) / denominator);
        double rearRightPower = powerCurve((y + x - rx) / denominator);

        frontLeftMotor.setPower(frontLeftPower);
        rearLeftMotor.setPower(rearLeftPower);
        frontRightMotor.setPower(frontRightPower);
        rearRightMotor.setPower(rearRightPower);
    }

    private Double powerCurve(Double power) throws InterruptedException {
        return Math.pow(power, 3) * powerReduction;
    }
}
