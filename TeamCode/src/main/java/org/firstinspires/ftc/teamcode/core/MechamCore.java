package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

public class MechamCore extends TelemetryCore {
    private Double powerReduction = 0.7;
    // ADJUSTMENT values are calibration values to overcome differences in friction and other factors
    // at the four motors in the robot.
    private final double FRONT_RIGHT_ADJUSTMENT = 1.02;
    private final double FRONT_LEFT_ADJUSTMENT = 1.01;
    private final double REAR_RIGHT_ADJUSTMENT = 1.02;
    private final double REAR_LEFT_ADJUSTMENT = 1.04;
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

        double powerReduction = getPowerReduction();
        Log.d("FTC-23217-autoMechamMovement", "isMoving:" + isMoving + " x:" + fixedX + " y:" + fixedY + " rx:" + fixedRx + " powerReduction:" + powerReduction);

        double frontLeftPower = -(fixedY + fixedX + fixedRx) / denominator * powerReduction * FRONT_LEFT_ADJUSTMENT;
        double rearLeftPower = (fixedY - fixedX + fixedRx) / denominator * powerReduction * REAR_LEFT_ADJUSTMENT;
        double frontRightPower = -(fixedY - fixedX - fixedRx) / denominator * powerReduction * FRONT_RIGHT_ADJUSTMENT;
        double rearRightPower = (fixedY + fixedX - fixedRx) / denominator * powerReduction * REAR_RIGHT_ADJUSTMENT;

        if (
                this.autonomousMode
                && Boolean.TRUE.equals(this.MAP_DEBUG.get(DebugEnum.DRIVE_MOTORS))
        ) {
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
            } else if (sensorValue <= zeroMotorPowerSensorValue) {
                powerReduction = 0; // Zero power if the sensor value is at or below the minimum threshold
            } else {
                powerReduction = (sensorValue - zeroMotorPowerSensorValue) / (maxMotorPowerSensorValue - zeroMotorPowerSensorValue);
            }
        } return powerReduction;
    }

    private void mechamMovement() throws InterruptedException {
        double right_stick_x_reducer = 1.0;

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x * right_stick_x_reducer;
        double rt = gamepad1.right_trigger;
        if (rt > 0.03) {
            // turbo mode
            powerReduction = rt;
        } else {
            powerReduction = 0.6;
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

        double frontLeftPower = powerCurve(-(y + x + rx) / denominator) * FRONT_LEFT_ADJUSTMENT;
        double rearLeftPower = powerCurve((y - x + rx) / denominator) * REAR_LEFT_ADJUSTMENT;
        double frontRightPower = powerCurve(-(y - x - rx) / denominator) * FRONT_RIGHT_ADJUSTMENT;
        double rearRightPower = powerCurve((y + x - rx) / denominator) * REAR_RIGHT_ADJUSTMENT;
        if (Boolean.TRUE.equals(this.MAP_DEBUG.get(DebugEnum.DRIVE_MOTORS))) {
            frontLeftMotor.setPower(frontLeftPower);
            rearLeftMotor.setPower(rearLeftPower);
            frontRightMotor.setPower(frontRightPower);
            rearRightMotor.setPower(rearRightPower);
        }
    }

    private Double powerCurve(Double power) throws InterruptedException {
        return Math.pow(power, 3) * powerReduction;
    }
}
