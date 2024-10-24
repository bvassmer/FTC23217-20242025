package org.firstinspires.ftc.teamcode.core;

public class ClawCore extends SensorCore {
    public enum SLIDE_STATE {
        HOLDING,
        MANUAL_MOVING,
        AUTO_UP_MOVING,
        AUTO_DOWN_MOVING,
    }
    public SLIDE_STATE slideState = SLIDE_STATE.HOLDING;
    public enum GripState {
        OPEN,
        CLOSED,
    }
    public GripState gripState = GripState.CLOSED;
    public final short PIXEL_PLACING_THRESHOLD = 430;
    public final short DANGER_ZONE_THRESHOLD = 380;
    public final short DEATH_ZONE_THRESHOLD = 80;
    public final short PIXEL_PICKUP_THRESHOLD = 30;

    public enum MANTIS_MODE {
        PIXEL_PICKUP,
        AVOID,
        INTERNAL_AVOID,
        PIXEL_DROP,
    }
    public MANTIS_MODE mantisMode = MANTIS_MODE.PIXEL_PICKUP;

    @Override
    public void runOpMode(boolean autonomousMode) throws InterruptedException {
        super.runOpMode(autonomousMode);
    }

    protected void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        if (enableController) {
            controllerActions();
        }
        adjustAngryMantisPosition();
        clawStateMachine();
    }

    private void controllerActions() throws InterruptedException {
        angryMantisInternals();
    }

    private void clawStateMachine() throws InterruptedException {
        float y = gamepad2.left_stick_y;
        boolean yButton = gamepad2.y;
        boolean xButton = gamepad2.x;
        if (y != 0.0) {
            slideState = SLIDE_STATE.MANUAL_MOVING;
        } else if (yButton) {
            slideState = SLIDE_STATE.AUTO_UP_MOVING;
        } else if (xButton) {
           slideState = SLIDE_STATE.AUTO_DOWN_MOVING;
        }

        switch (slideState) {
            case AUTO_UP_MOVING:
                if (slideTofDistance >= PIXEL_PLACING_THRESHOLD) {
                    stopMoving();
                } else if (slideTofDistance < DEATH_ZONE_THRESHOLD) {
                        leftSlideMotor.setPower(0.3);
                        rightSlideMotor.setPower(-0.3);
                } else if (slideTofDistance < DANGER_ZONE_THRESHOLD) {
                        leftSlideMotor.setPower(0.5);
                        rightSlideMotor.setPower(-0.5);
                } else {
                    leftSlideMotor.setPower(0.9);
                    rightSlideMotor.setPower(-0.9);
                }
                break;
            case AUTO_DOWN_MOVING:
                if (slideTofDistance <= PIXEL_PICKUP_THRESHOLD) {
                    stopMoving();
                } else if (slideTofDistance < DEATH_ZONE_THRESHOLD) {
                    leftSlideMotor.setPower(-0.3);
                    rightSlideMotor.setPower(0.3);
                } else if (slideTofDistance < DANGER_ZONE_THRESHOLD) {
                    leftSlideMotor.setPower(-0.5);
                    rightSlideMotor.setPower(0.5);
                } else {
                    leftSlideMotor.setPower(-0.9);
                    rightSlideMotor.setPower(0.9);
                }
                break;
            case HOLDING:
                break;
            case MANUAL_MOVING:
                if (y == 0.0) {
                    stopMoving();
                } else if (slideTofDistance < DEATH_ZONE_THRESHOLD) {
                    if (y < 0) {
                        leftSlideMotor.setPower(-y * 0.3);
                        rightSlideMotor.setPower(y * 0.3);
                    } else if (y > 0) {
                        leftSlideMotor.setPower(-y * 0.15);
                        rightSlideMotor.setPower(y * 0.15);
                    }
                } else if (slideTofDistance < DANGER_ZONE_THRESHOLD) {
                    if (y < 0) {
                        leftSlideMotor.setPower(-y * 0.5);
                        rightSlideMotor.setPower(y * 0.5);
                    } else if (y > 0) {
                        leftSlideMotor.setPower(-y * 0.3);
                        rightSlideMotor.setPower(y * 0.3);
                    }
                } else {
                    leftSlideMotor.setPower(-y * 0.9);
                    rightSlideMotor.setPower(y * 0.9);
                }
                break;
            default:
        }
    }

    private void stopMoving() throws InterruptedException {
        leftSlideMotor.setPower(0.2);
        rightSlideMotor.setPower(-0.2);
        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
        slideState = SLIDE_STATE.HOLDING;
    }

    private void adjustAngryMantisPosition() throws InterruptedException {
        if (slideTofDistance < 38) {
            mantisMode = LiftCore.MANTIS_MODE.PIXEL_PICKUP;
            angryMantisDoorClose();
            angryMantisPixelPickupMode();
        } else if (slideTofDistance >= 41 && slideTofDistance < 130.0) {
            mantisMode = LiftCore.MANTIS_MODE.INTERNAL_AVOID;
            angryMantisDoorClose();
            angryMantisInternalAvoidMode();
        } else if (slideTofDistance >= 140.0 && slideTofDistance < 340.0) {
            mantisMode = LiftCore.MANTIS_MODE.AVOID;
            angryMantisDoorClose();
            angryMantisAvoidMode();
        } else if (slideTofDistance >= 360.0 && slideTofDistance < 1200) {
            mantisMode = LiftCore.MANTIS_MODE.PIXEL_DROP;
            angryMantisDropMode();
        }
    }

    private void angryMantisInternals() throws InterruptedException {
        if (gamepad2.b) {
            dropPixelIfSafe();
        }
        if (gamepad2.a) {
            angryMantisCloseGrip();
        }
    }

    public void dropPixelIfSafe() throws InterruptedException{
        angryMantisOpenGrip();
        if (slideTofDistance > DANGER_ZONE_THRESHOLD) {
            angryMantisDoorOpen();
        }
    }

    private void angryMantisDropMode() throws InterruptedException {
        rightClawPivotServo.setPosition(0.7);
    }

    private void angryMantisPixelPickupMode() throws InterruptedException {
        rightClawPivotServo.setPosition(0.43);
    }
    private void angryMantisInternalAvoidMode() throws InterruptedException {
        rightClawPivotServo.setPosition(0.37);
    }
    private void angryMantisAvoidMode() throws InterruptedException {
        rightClawPivotServo.setPosition(0.34);
    }

    public void angryMantisCloseGrip() throws InterruptedException {
        leftClawGripServo.setPosition(0.65);
        rightClawGripServo.setPosition(0.35);
        gripState = GripState.CLOSED;
    }

    public void angryMantisOpenGrip() throws InterruptedException {
        leftClawGripServo.setPosition(0.2);
        rightClawGripServo.setPosition(0.8);
        gripState = GripState.OPEN;
    }

    private void angryMantisDoorClose() throws InterruptedException {
        leftClawDoorServo.setPosition(0);
        rightClawDoorServo.setPosition(0.85);
    }

    private void angryMantisDoorOpen() throws InterruptedException {
        leftClawDoorServo.setPosition(0.8);
        rightClawDoorServo.setPosition(0);
    }
}

