package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

public class MechamCore extends TelemetryCore {
    private Double powerReduction = 0.7;
    // ADJUSTMENT values are calibration values to overcome differences in friction and other factors
    // at the four motors in the robot.
    private final double FRONT_RIGHT_ADJUSTMENT = 1.02;
    private final double FRONT_LEFT_ADJUSTMENT = 1.01;
    private final double REAR_RIGHT_ADJUSTMENT = 1.02;
    private final double REAR_LEFT_ADJUSTMENT = 1.02;
    public boolean isMoving = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Log.d("FTC-23217-MechamCore", "MechamCore Start.");
        super.runOpMode();
    }

    public void workers(double x, double y, double rx) throws InterruptedException {
        super.workers();
        if (this.autonomousMode) {
            autoMechamMovement(x, y, rx);
        }
        if (this.enableController) {
            mechamMovement();
        }
    }

    public void stopMovement() throws InterruptedException {
        frontLeftMotor.setPower(0.0);
        rearLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        rearRightMotor.setPower(0.0);
    }

    /* private void autoMechamMovement(double x, double y, double rx) throws InterruptedException {
        double fixedY = -y;
        double fixedX = -x;
        double fixedRx = rx;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        // double denominator = Math.max(Math.abs(fixedY) + Math.abs(fixedX) + Math.abs(fixedRx), 1);

        // Why This Works: ✅ Ensures proportional scaling between x, y, and rx when they all exist.
        // ✅ Fixes interactions between translation and rotation, preventing unstable power outputs.
        // ✅ Prevents movement distortion when moving diagonally or rotating at the same time.
        double magnitude = Math.sqrt(fixedY * fixedY + fixedX * fixedX + fixedRx * fixedRx);
        double denominator = Math.max(magnitude, 1);  // Prevents division by zero
        // double translationMagnitude = Math.sqrt(fixedY * fixedY + fixedX * fixedX);
        // double rotationWeight = 0.5;  // Reduce rotation strength relative to translation
        // double denominator = Math.max(translationMagnitude + Math.abs(fixedRx) * rotationWeight, 1);


        isMoving = (y != 0 || x != 0 || rx != 0);

        double powerReduction = getPowerReduction();
        Log.d("FTC-23217-autoMechamMovement", "isMoving:" + isMoving + " x:" + fixedX + " y:" + fixedY + " rx:" + fixedRx + " powerReduction:" + powerReduction);

        double frontLeftPower = -(fixedY + fixedX + fixedRx) / denominator * powerReduction * FRONT_LEFT_ADJUSTMENT; // negated b/c of 90 degree bevel gears
        double rearLeftPower = (fixedY - fixedX + fixedRx) / denominator * powerReduction * REAR_LEFT_ADJUSTMENT;
        double frontRightPower = -(fixedY - fixedX - fixedRx) / denominator * powerReduction * FRONT_RIGHT_ADJUSTMENT; // negated b/c of 90 degree bevel gears
        double rearRightPower = (fixedY + fixedX - fixedRx) / denominator * powerReduction * REAR_RIGHT_ADJUSTMENT;
        Log.d("FTC-23217-autoMechamMovement", "frontLeftPower:" + frontLeftPower + " rearLeftPower:" + rearLeftPower + " frontRightPower:" + frontRightPower + " rearRightPower:" + rearRightPower);

        if (
                this.autonomousMode
                && Boolean.TRUE.equals(this.MAP_DEBUG.get(DebugEnum.DRIVE_MOTORS))
        ) {
            frontLeftMotor.setPower(frontLeftPower);
            rearLeftMotor.setPower(rearLeftPower);
            frontRightMotor.setPower(frontRightPower);
            rearRightMotor.setPower(rearRightPower);
        }
    } */

    private void autoMechamMovement(double x, double y, double rx) throws InterruptedException {
        double fixedY = -y;
        double fixedX = -x;
        double fixedRx = -rx * 0.6; // Removed negation

        // Normalize movement with lower weight for rotation
        double translationMagnitude = Math.sqrt(fixedY * fixedY + fixedX * fixedX);
        double rotationWeight = 0.5;  // Reduce rotation strength relative to translation
        double denominator = Math.max(translationMagnitude + Math.abs(fixedRx) * rotationWeight, 1);

        isMoving = (y != 0 || x != 0 || rx != 0);

        double powerReduction = getPowerReduction();
        Log.d("FTC-23217-autoMechamMovement", "isMoving:" + isMoving + " x:" + fixedX + " y:" + fixedY + " rx:" + fixedRx + " powerReduction:" + powerReduction);

        double frontLeftPower = sigmoidPowerCurve(-(fixedY + fixedX + fixedRx) / denominator) * FRONT_LEFT_ADJUSTMENT;
        double rearLeftPower = sigmoidPowerCurve((fixedY - fixedX + fixedRx) / denominator) * REAR_LEFT_ADJUSTMENT;
        double frontRightPower = sigmoidPowerCurve(-(fixedY - fixedX - fixedRx) / denominator) * FRONT_RIGHT_ADJUSTMENT;
        double rearRightPower = sigmoidPowerCurve((fixedY + fixedX - fixedRx) / denominator) * REAR_RIGHT_ADJUSTMENT;

        Log.d("FTC-23217-autoMechamMovement", "frontLeftPower:" + frontLeftPower + " rearLeftPower:" + rearLeftPower +
                " frontRightPower:" + frontRightPower + " rearRightPower:" + rearRightPower);

        if (this.autonomousMode && Boolean.TRUE.equals(this.MAP_DEBUG.get(DebugEnum.DRIVE_MOTORS))) {
            frontLeftMotor.setPower(frontLeftPower);
            rearLeftMotor.setPower(rearLeftPower);
            frontRightMotor.setPower(frontRightPower);
            rearRightMotor.setPower(rearRightPower);
        }
    }


    private double getPowerReduction() {
        double powerReduction = 1;
        if (this.sensorForDriveControl != ComponentEnum.NONE) {
            double sensorValue = 0;
            switch (this.sensorForDriveControl) {
                case FRONT_ULTRASONIC_SENSOR:
                    sensorValue = this.ultrasonicFrontSensorReading;
                    break;
                case FRONT_TOF_SENSOR:
                    sensorValue = this.tofFrontSensorReading;
                    break;
            }

            if (sensorValue >= maxMotorPowerSensorValue) {
                powerReduction = 1; // Full power if the sensor is at maximum value
            } else if (sensorValue <= zeroMotorPowerSensorValue && sensorValue > zeroMotorPowerSensorValue * 0.95) {
                powerReduction = 0.0;
            } else if (sensorValue <= zeroMotorPowerSensorValue * 0.95) {
                powerReduction = -0.05; // Zero power if the sensor value is at or below the minimum threshold
            } else {
                powerReduction = (sensorValue - zeroMotorPowerSensorValue) / (maxMotorPowerSensorValue - zeroMotorPowerSensorValue);
            }
            Log.d("FTC-23217-MechamCore-getPowerReduction", "sensorValue" + sensorValue + " powerReduction:" + powerReduction);
        }
        return powerReduction;
    }

    private void mechamMovement() throws InterruptedException {
        double right_stick_x_reducer = 1.0;

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x * right_stick_x_reducer;
        double rt = gamepad1.right_trigger;

        isMoving = (y != 0 || x != 0 || rx != 0);

        Log.d("FTC-23217-mechamMovement", "isMoving:" + isMoving + " x:" + x + " y:" + y + " rx:" + rx + " rt:" + rt);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        // double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Why This Works: ✅ Ensures proportional scaling between x, y, and rx when they all exist.
        // ✅ Fixes interactions between translation and rotation, preventing unstable power outputs.
        // ✅ Prevents movement distortion when moving diagonally or rotating at the same time.
        double magnitude = Math.sqrt(y * y + x * x + rx * rx);
        double denominator = Math.max(magnitude, 1);  // Prevents division by zero

        double frontLeftPower = sigmoidPowerCurve(-(y + x + rx) / denominator) * FRONT_LEFT_ADJUSTMENT; // negated b/c of 90 degree bevel gears
        double rearLeftPower = sigmoidPowerCurve((y - x + rx) / denominator) * REAR_LEFT_ADJUSTMENT;
        double frontRightPower = sigmoidPowerCurve(-(y - x - rx) / denominator) * FRONT_RIGHT_ADJUSTMENT; // negated b/c of 90 degree bevel gears
        double rearRightPower = sigmoidPowerCurve((y + x - rx) / denominator) * REAR_RIGHT_ADJUSTMENT;
        if (Boolean.TRUE.equals(this.MAP_DEBUG.get(DebugEnum.DRIVE_MOTORS))) {
            frontLeftMotor.setPower(frontLeftPower);
            rearLeftMotor.setPower(rearLeftPower);
            frontRightMotor.setPower(frontRightPower);
            rearRightMotor.setPower(rearRightPower);
        }
    }

    private Double sigmoidPowerCurve(Double power) throws InterruptedException {
        double a = 3.0; // Adjusts how aggressively the curve smooths (higher = sharper)

        // Compute sigmoid function
        double sigmoid = (1 / (1 + Math.exp(-a * Math.abs(power)))) - 0.5;
        double output = Math.signum(power) * sigmoid * 2 * powerReduction;

        // Ensure the output is clamped to [-1, 1]
        return Math.max(-1.0, Math.min(1.0, output));
    }
}
