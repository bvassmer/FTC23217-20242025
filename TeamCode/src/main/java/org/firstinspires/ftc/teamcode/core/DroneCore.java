package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.util.ElapsedTime;

public class DroneCore extends ClawCore {
    public enum DroneState {
        WAITING,
        LAUNCH,
        LAUNCHING,
        RESET,
    }
    public DroneState droneState = DroneState.WAITING;
    protected ElapsedTime droneTimer = new ElapsedTime();

    @Override
    public void runOpMode(boolean autonomousMode) throws InterruptedException {
        super.runOpMode(autonomousMode);
        droneServo.setPosition(1);
    }

    protected void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        if (enableController) {
            controllerActions();
        }
        droneStateMachine();
    }

    private void controllerActions() throws InterruptedException {
        droneMovement();
    }

    private void droneMovement() throws InterruptedException {
        boolean launchButton = gamepad2.dpad_up;
        if (launchButton) {
            droneTimer.reset();
            droneState = DroneState.LAUNCH;
        }
    }

    private void droneStateMachine() throws InterruptedException {
        switch (droneState) {
            case WAITING:
                droneServo.setPosition(1);
                break;
            case LAUNCH:
                droneServo.setPosition(0);
                droneState = DroneState.LAUNCHING;
                break;
            case LAUNCHING:
                if (droneTimer.seconds() > 2) {
                    droneState = DroneState.RESET;
                }
                break;
            case RESET:
                droneServo.setPosition(1);
                droneState = DroneState.WAITING;
                break;
            default:
        }
    }


}
