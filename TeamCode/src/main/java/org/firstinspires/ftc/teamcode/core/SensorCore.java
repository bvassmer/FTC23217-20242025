package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

public class SensorCore extends OdometryCore {
    // public short frontDistance = 0;
    // public short leftDistance = 0;
    // public short rightDistance = 0;
    public short slideTofDistance = 0;
    public short angryMantisTofDistance = 0;
    public ArrayList<Short> slideTofDistanceList = new ArrayList<>();
    public ArrayList<Short> angryMantisTofDistanceList = new ArrayList<>();

    // final Double ULTRASONIC_SENSOR_WAIT = 0.1; // in seconds. equals 0.2s/cycle or 5 Hz.
    // protected ElapsedTime ultrasonicSensorTimer = new ElapsedTime();
    public enum SensorState {
        REQUEST_READING,
        WAIT,
    };
    public enum SensorDirection {
        LEFT,
        RIGHT,
    };

    // SensorState ultraSonicSensorState = SensorState.REQUEST_READING;

    @Override
    public void runOpMode(boolean autonomousMode) throws InterruptedException {
        super.runOpMode(autonomousMode);
        // startSensorTimers();
        startTofSensors();
    }

    protected void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        // clawTofSensor.sensorStateMachine();
        // ultrasonicSensorStateMachine();
        processTofSensorDistances();
    }

    private void startTofSensors() {
    }

    /* protected void startSensorTimers() {
        ultrasonicSensorTimer.startTime();
    } */

    /* protected void ultrasonicSensorStateMachine() {
        // Currently updates readings every 0.2 seconds.
        switch (ultraSonicSensorState) {
            case REQUEST_READING:
                if (ultrasonicSensorTimer.seconds() > ULTRASONIC_SENSOR_WAIT) {
                    requestUltrasonicSensors();
                    ultraSonicSensorState = SensorState.WAIT;
                    ultrasonicSensorTimer.reset();
                    break;
                }
                break;
            case WAIT:
                if (ultrasonicSensorTimer.seconds() > ULTRASONIC_SENSOR_WAIT) {
                    getUltrasonicSensorDistances();
                    ultraSonicSensorState = SensorState.REQUEST_READING;
                    ultrasonicSensorTimer.reset();
                }
                break;
            default:
                // shouldn't get here.
                // TODO: error handling
        }
    } */

    /* protected void requestUltrasonicSensors() {
        // leftUltrasonicSensor.startReading();
        // rightUltrasonicSensor.startReading();
        //frontUltrasonicSensor.startReading();
    } */

    /* protected void getUltrasonicSensorDistances() {
        short rawLeftDistance = leftUltrasonicSensor.getDistance();
        leftDistance = rawLeftDistance;
        processUltrasonicSensorDistance(rawLeftDistance, SensorDirection.LEFT);
        short rawRightDistance = rightUltrasonicSensor.getDistance();
        rightDistance = rawRightDistance;
        processUltrasonicSensorDistance(rawRightDistance, SensorDirection.RIGHT);
    } */

    /* final short ULTRASONIC_MAX_DISTANCE = 765;
    final short ULTRASONIC_MIN_DISTANCE = 0;
    private void processUltrasonicSensorDistance(short distance, SensorDirection direction) {
        if (distance < ULTRASONIC_MAX_DISTANCE && distance > ULTRASONIC_MIN_DISTANCE) {
            switch (direction) {
                case LEFT:
                    leftDistance = distance;
                    break;
                case RIGHT:
                    rightDistance = distance;
                    break;
                default:
            }
        }
    } */

    final short TOF_MAX_DISTANCE = 1200;
    final short TOF_MIN_DISTANCE = 5;
    private void processTofSensorDistances() {
        short slideTofDistanceRaw = (short)slideTofSensor.getDistance(DistanceUnit.MM);
        if (slideTofDistanceRaw < TOF_MAX_DISTANCE && slideTofDistanceRaw > TOF_MIN_DISTANCE) {
            slideTofDistance = slideTofDistanceRaw;
        }
        short angryMantisTofDistanceRaw = (short)angryMantisTofSensor.getDistance(DistanceUnit.MM);
        if (angryMantisTofDistanceRaw < TOF_MAX_DISTANCE && angryMantisTofDistanceRaw > TOF_MIN_DISTANCE) {
            angryMantisTofDistance = angryMantisTofDistanceRaw;
        }
    }
}
