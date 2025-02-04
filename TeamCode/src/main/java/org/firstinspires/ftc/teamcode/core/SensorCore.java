package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.DoubleCircularBuffer;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;

import java.util.ArrayList;
import java.util.Arrays;

public class SensorCore extends OdometryCore {
    final short TOF_MAX_DISTANCE = 1200;
    final short TOF_MIN_DISTANCE = 5;
    private final short ULTRASONIC_MAX_DISTANCE = 765;
    private final short ULTRASONIC_MIN_DISTANCE = 0;
    private final Double ULTRASONIC_SENSOR_WAIT = 0.1; // in seconds. equals 0.2s/cycle or 5 Hz.
    private ElapsedTime ultrasonicSensorTimer = new ElapsedTime();
    public enum SensorState {
        REQUEST_READING,
        WAIT,
    };
    SensorState ultraSonicSensorState = SensorState.REQUEST_READING;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        startSensorTimers();
    }

    protected void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        if (
                Boolean.TRUE.equals(componentState.get(ComponentState.SLIDE_TOF_SENSOR))
        ) {
            processTofSensorDistances();
        }
        if (
                Boolean.TRUE.equals(componentState.get(ComponentState.FRONT_ULTRASONIC_SENSOR))
        ) {
            ultrasonicSensorStateMachine();
        }
    }

    private void processTofSensorDistances() {
        short slideTofDistanceRaw = (short)slideTofSensor.getDistance(DistanceUnit.MM);
        if (slideTofDistanceRaw < TOF_MAX_DISTANCE && slideTofDistanceRaw > TOF_MIN_DISTANCE) {
            slideTofCB.addAndCalculate((double)slideTofDistanceRaw, 0);
        }
    }

    private void startSensorTimers() {
        ultrasonicSensorTimer.startTime();
    }

    private void ultrasonicSensorStateMachine() {
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
    }

    private void requestUltrasonicSensors() {
        frontUltraSonicSensor.startReading();
    }

    private void getUltrasonicSensorDistances() {
        short rawFrontDistance = frontUltraSonicSensor.getDistance();
        if (rawFrontDistance < ULTRASONIC_MAX_DISTANCE && rawFrontDistance > ULTRASONIC_MIN_DISTANCE) {
            frontUltraSonicCB.addAndCalculate(rawFrontDistance, 0);
        }
    }
}
