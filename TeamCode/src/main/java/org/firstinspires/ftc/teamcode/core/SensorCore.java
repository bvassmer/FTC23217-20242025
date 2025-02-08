package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorCore extends OdometryCore {
    final short TOF_MIN_DISTANCE = 5; // mm
    final short TOF_MAX_DISTANCE = 1200; // mm
    private final ElapsedTime ultrasonicSensorTimer = new ElapsedTime();
    private final ElapsedTime slideTofSensorTimer = new ElapsedTime();
    private final ElapsedTime colorSensorTimer = new ElapsedTime();
    private final ElapsedTime touchSensorTimer = new ElapsedTime();
    public enum UltrasonicSensorState {
        REQUEST_READING,
        WAIT,
    };
    public enum GenericSensorState {
        WAIT,
        READ,
    }
    UltrasonicSensorState ultraSonicSensorState = UltrasonicSensorState.REQUEST_READING;
    GenericSensorState slideTofSensorState = GenericSensorState.WAIT;
    GenericSensorState colorSensorState = GenericSensorState.WAIT;
    GenericSensorState touchSensorState = GenericSensorState.WAIT;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        startSensorTimers();
    }

    protected void workers() throws InterruptedException {
        super.workers();
        if (
                Boolean.TRUE.equals(MAP_COMPONENT.get(ComponentEnum.SLIDE_TOF_SENSOR))
                || Boolean.TRUE.equals(MAP_COMPONENT.get(ComponentEnum.FRONT_TOF_SENSOR))
                || Boolean.TRUE.equals(MAP_DEBUG.get(DebugEnum.SENSORS))
        ) {
            tofSensorStateMachine();
        }
        if (
                Boolean.TRUE.equals(MAP_COMPONENT.get(ComponentEnum.FRONT_ULTRASONIC_SENSOR))
                || Boolean.TRUE.equals(MAP_DEBUG.get(DebugEnum.SENSORS))
        ) {
            ultrasonicSensorStateMachine();
        }
        if (
                Boolean.TRUE.equals(MAP_COMPONENT.get(ComponentEnum.CLAW_COLOR_SENSOR))
                || Boolean.TRUE.equals(MAP_DEBUG.get(DebugEnum.SENSORS))
        ) {
            colorSensorStateMachine();
        }
        if (
                Boolean.TRUE.equals(MAP_COMPONENT.get(ComponentEnum.CLAW_TOUCH_SENSORS))
                || Boolean.TRUE.equals(MAP_DEBUG.get(DebugEnum.SENSORS))
        ) {
            touchSensorStateMachine();
        }

    }

    private void processTofSensorDistances() {
        short slideTofDistanceRaw = (short)slideTofSensor.getDistance(DistanceUnit.MM);
        if (slideTofDistanceRaw < TOF_MAX_DISTANCE && slideTofDistanceRaw > TOF_MIN_DISTANCE) {
            tofSlideSensorReading = slideTofDistanceRaw;
            slideTofCB.addAndCalculate((double)slideTofDistanceRaw, 0);
        }
        short frontTofDistanceRaw = (short)frontTofSensor.getDistance(DistanceUnit.MM);
        if (frontTofDistanceRaw < TOF_MAX_DISTANCE && frontTofDistanceRaw > TOF_MIN_DISTANCE) {
            tofFrontSensorReading = frontTofDistanceRaw;
            frontTofCB.addAndCalculate((double)frontTofDistanceRaw, 0);
        }
    }

    private void processColorSensorData() {
        clawColorSensorBlue = clawColorSensor.blue();
        clawColorSensorGreen = clawColorSensor.green();
        clawColorSensorRed = clawColorSensor.red();
        clawColorSensorDistance = clawColorSensor.getDistance(DistanceUnit.MM);
    }

    private void processTouchSensorData() {
        isLeftTouchSensorPressed = leftTouchSensor.isPressed();
        isRightTouchSensorPressed = rightTouchSensor.isPressed();
    }

    private void startSensorTimers() {
        ultrasonicSensorTimer.startTime();
        slideTofSensorTimer.startTime();
    }

    private void ultrasonicSensorStateMachine() {
        // Currently updates readings every 0.2 seconds.
        // in seconds. equals 0.2s/cycle or 5 Hz.
        double ULTRASONIC_SENSOR_WAIT = 0.1;
        switch (ultraSonicSensorState) {
            case REQUEST_READING:
                if (ultrasonicSensorTimer.seconds() > ULTRASONIC_SENSOR_WAIT) {
                    requestUltrasonicSensors();
                    ultraSonicSensorState = UltrasonicSensorState.WAIT;
                    ultrasonicSensorTimer.reset();
                    break;
                }
                break;
            case WAIT:
                if (ultrasonicSensorTimer.seconds() > ULTRASONIC_SENSOR_WAIT) {
                    getUltrasonicSensorDistances();
                    ultraSonicSensorState = UltrasonicSensorState.REQUEST_READING;
                    ultrasonicSensorTimer.reset();
                }
                break;
            default:
                // shouldn't get here.
                // TODO: error handling
        }
    }

    private void requestUltrasonicSensors() {
        frontUltraSonicSensor.startReading();
    }

    private void getUltrasonicSensorDistances() {
        short rawFrontDistance = frontUltraSonicSensor.getDistance();
        short ULTRASONIC_MIN_DISTANCE = 20;
        short ULTRASONIC_MAX_DISTANCE = 765;
        if (rawFrontDistance < ULTRASONIC_MAX_DISTANCE && rawFrontDistance > ULTRASONIC_MIN_DISTANCE) {
            ultrasonicFrontSensorReading = rawFrontDistance;
        }
    }

    private void tofSensorStateMachine() {
        double TOF_WAIT_TIME = 0.1;
        switch (slideTofSensorState) {
            case READ:
                processTofSensorDistances();
                slideTofSensorState = GenericSensorState.WAIT;
                slideTofSensorTimer.reset();
                break;
            case WAIT:
                if (slideTofSensorTimer.seconds() > TOF_WAIT_TIME) {
                    slideTofSensorState = GenericSensorState.READ;
                }
                break;
        }
    }

    private void colorSensorStateMachine() {
        double TOF_WAIT_TIME = 0.2;
        switch (colorSensorState) {
            case READ:
                processColorSensorData();
                colorSensorState = GenericSensorState.WAIT;
                colorSensorTimer.reset();
                break;
            case WAIT:
                if (colorSensorTimer.seconds() > TOF_WAIT_TIME) {
                    colorSensorState = GenericSensorState.READ;
                }
                break;
        }
    }

    private void touchSensorStateMachine() {
        double TOF_WAIT_TIME = 0.1;
        switch (touchSensorState) {
            case READ:
                processTouchSensorData();
                touchSensorState = GenericSensorState.WAIT;
                touchSensorTimer.reset();
                break;
            case WAIT:
                if (touchSensorTimer.seconds() > TOF_WAIT_TIME) {
                    touchSensorState = GenericSensorState.READ;
                }
                break;
        }
    }
}
