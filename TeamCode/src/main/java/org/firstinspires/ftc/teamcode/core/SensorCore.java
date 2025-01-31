package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.DoubleCircularBuffer;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;

import java.util.ArrayList;

public class SensorCore extends OdometryCore {
    private Enum.TeamColor teamColor = Enum.TeamColor.BLUE;

    public enum SensorState {
        REQUEST_READING,
        WAIT,
    };
    public enum SensorDirection {
        LEFT,
        RIGHT,
    };

    SensorState ultraSonicSensorState = SensorState.REQUEST_READING;

    public void runOpMode(boolean autonomousMode, Enum.TeamColor teamColor) throws InterruptedException {
        super.runOpMode(autonomousMode);
        this.teamColor = teamColor;
    }

    protected void workers(boolean enableController) throws InterruptedException {
        processTofSensorDistances();
    }

    final short TOF_MAX_DISTANCE = 1200;
    final short TOF_MIN_DISTANCE = 5;
    private void processTofSensorDistances() {
        short slideTofDistanceRaw = (short)slideTofSensor.getDistance(DistanceUnit.MM);
        if (slideTofDistanceRaw < TOF_MAX_DISTANCE && slideTofDistanceRaw > TOF_MIN_DISTANCE) {
            robotState.slideTofCB.addAndCalculate((double)slideTofDistanceRaw, 0);
        }
    }
}
