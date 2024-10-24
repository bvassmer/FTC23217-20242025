package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PixelIntakeCore extends LiftCore {
    final Double CAR_WASH_LIFT_SENSOR_WAIT = 0.5; // in seconds. equals 0.2s/cycle or 5 Hz.
    final Double CAR_WASH_EJECT_SENSOR_WAIT = 2.0; // in seconds. equals 0.2s/cycle or 5 Hz.
    public ElapsedTime carWashTimer = new ElapsedTime();
    public enum CarWashState {
        LIFT,
        MOVING,
        LOWER,
        WAIT,
        DONE_MOVING,
        EJECT_PIXEL,
        EJECTING,
        FORCE_EJECTING,
    };
    public CarWashState carWashState = CarWashState.WAIT;

    @Override
    public void runOpMode(boolean autonomousMode) throws InterruptedException {
        super.runOpMode(autonomousMode);
        startSensorTimers();
    }

    protected void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        if (enableController) {
            controllerActions();
        }
        carWashLiftStateMachine();
    }

    protected void startSensorTimers() {
        carWashTimer.startTime();
    }

    private void controllerActions() throws InterruptedException {
        if (gamepad2.right_bumper && carWashState == CarWashState.WAIT) {
            carWashState = CarWashState.LIFT;
        }
        if (gamepad2.left_bumper && carWashState == CarWashState.WAIT) {
            carWashState = CarWashState.LOWER;
        }
        if (gamepad2.right_trigger > 0.1) {
            pullPixelsIn(-gamepad2.right_trigger);

        }
        if (gamepad2.left_trigger > 0.1) {
            pushPixelsOut(gamepad2.left_trigger);
        }
        if (gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0) {
            stopPixels();
        }
    }

    private void pullPixelsIn(float power) throws InterruptedException {
        angryMantisOpenGrip();
        carWashMotor.setPower(power);
    }

    public void pushPixelsOut(float power) throws InterruptedException {
        carWashMotor.setPower(power);
    }

    public void stopPixels() throws InterruptedException {
        carWashMotor.setPower(0);
    }

    public void autoLowerCarWash() throws InterruptedException {
        carWashState = CarWashState.LOWER;
    }

    protected void carWashLiftStateMachine() {
        // Currently updates readings every 0.2 seconds.
        switch (carWashState) {
            case WAIT:
                break;
            case DONE_MOVING:
                carWashMotor.setPower(0);
                carWashState = CarWashState.WAIT;
                break;
            case LIFT:
                carWashTimer.reset();
                carWashMotor.setPower(1);
                carWashState = CarWashState.MOVING;
                break;
            case LOWER:
                carWashTimer.reset();
                carWashMotor.setPower(-1);
                carWashState = CarWashState.MOVING;
                break;
            case EJECT_PIXEL:
                carWashTimer.reset();
                carWashMotor.setPower(1);
                carWashState = CarWashState.EJECTING;
                break;
            case EJECTING:
                if (carWashTimer.seconds() > CAR_WASH_EJECT_SENSOR_WAIT) {
                    carWashState = CarWashState.DONE_MOVING;
                }
                break;
            case FORCE_EJECTING:
                carWashMotor.setPower(1);
                carWashState = CarWashState.WAIT;
                break;
            case MOVING:
                if (carWashTimer.seconds() > CAR_WASH_LIFT_SENSOR_WAIT) {
                    carWashState = CarWashState.DONE_MOVING;
                }
                break;
            default:
                // shouldn't get here.
                // TODO: error handling
        }
    }


}

