package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.DoubleCircularBuffer;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;
import org.firstinspires.ftc.teamcode.library.LinearVelocityCircularBuffer;
import org.firstinspires.ftc.teamcode.library.PoseCircularBuffer;
import org.firstinspires.ftc.teamcode.library.ShortCircularBuffer;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class DataCore extends HardwareCore {
    public boolean isMoving = false;
    protected Enum.TeamColor teamColor = Enum.TeamColor.BLUE;
    protected boolean autonomousMode = false;
    protected double startAngle = 0.0;
    protected int circularBufferCapacity = 100;
    // currentPose is the current center of the robot.
    public PoseCircularBuffer robotPoses = new PoseCircularBuffer(circularBufferCapacity, 3, DistanceUnit.INCH, AngleUnit.DEGREES);
    public PoseCircularBuffer cameraPoses = new PoseCircularBuffer(circularBufferCapacity, 3, DistanceUnit.INCH, AngleUnit.DEGREES);
    public PoseCircularBuffer odoPoses = new PoseCircularBuffer(circularBufferCapacity, 3, DistanceUnit.INCH, AngleUnit.DEGREES);
    public PoseCircularBuffer odoVelocityPoses = new PoseCircularBuffer(circularBufferCapacity, 3, DistanceUnit.INCH, AngleUnit.DEGREES);
    public DoubleCircularBuffer bearingCB = new DoubleCircularBuffer(circularBufferCapacity, true, true);
    public DoubleCircularBuffer slideTofCB = new DoubleCircularBuffer(circularBufferCapacity, true, false);
    public ShortCircularBuffer frontUltraSonicCB = new ShortCircularBuffer(circularBufferCapacity, true, false);
    public LinearVelocityCircularBuffer cameraLinearVelocityCB = new LinearVelocityCircularBuffer(circularBufferCapacity);
    public double tickFrequency = 0;
    protected enum ComponentState {
        SLIDE_TOF_SENSOR,
        FRONT_ULTRASONIC_SENSOR,
        CAMERA,
        CLAW_TOUCH_SENSORS,
        CLAW_COLOR_SENSOR,
    }
    Map<ComponentState, Boolean> componentState = Stream.of(new Object[][] {
                    { ComponentState.SLIDE_TOF_SENSOR, false },
                    { ComponentState.FRONT_ULTRASONIC_SENSOR, false },
                    { ComponentState.CAMERA, false },
                    { ComponentState.CLAW_TOUCH_SENSORS, false },
                    { ComponentState.CLAW_COLOR_SENSOR, false },
            })
            .collect(Collectors.toMap(data -> (ComponentState) data[0], data -> (Boolean) data[1]));

    public void runOpMode() throws InterruptedException {
        Log.d("FTC-23217", "DataCore Start.");
        super.runOpMode(autonomousMode);
        outputTelemetry(true);
    }

    protected void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        calculateTickTime();
        outputTelemetry(false);
    }

    public void calculate() throws InterruptedException {
        this.cameraLinearVelocityCB.addAndCalculate(new LinearVelocity(cameraPoses.getAverageSpeed(), cameraPoses.getAverageHeading()));
    }

    private void outputTelemetry(boolean updateTelemetry) throws InterruptedException {
        if (robotPoses.isEmpty()) {
            telemetry.addData("Pose (ROBOT)", "NONE");
        } else {
            telemetry.addData("Pose (ROBOT)", "x:" + robotPoses.getLatestX() + " y:" + robotPoses.getLatestY() + " h:" + robotPoses.getLatestHeading());
        }
        if (cameraPoses.isEmpty()) {
            telemetry.addData("Pose (CAM)", "NONE");
        } else {
            telemetry.addData("Pose (CAM)", "x:" + cameraPoses.getLatestX() + " y:" + cameraPoses.getLatestY() + " h:" + cameraPoses.getLatestHeading());
        }
        if (odoPoses.isEmpty()) {
            telemetry.addData("Pose (ODO)", "NONE");
        } else {
            telemetry.addData("Pose (ODO)", "x:" + odoPoses.getLatestX() + " y:" + odoPoses.getLatestY() + " h:" + odoPoses.getLatestHeading());
        }
        if (odoVelocityPoses.isEmpty()) {
            telemetry.addData("Velocity (ODO)", "NONE");
        } else {
            telemetry.addData("Velocity (ODO)", "x:" + odoVelocityPoses.getLatestX() + " y:" + odoVelocityPoses.getLatestY() + " h:" + odoVelocityPoses.getLatestHeading());
        }

        telemetry.addData("Robot Bearing (CAM) (CB)", bearingCB.getAverage());
        telemetry.addData("Slide Distance (CB) (mm) ", slideTofCB.getAverage());
        telemetry.addData("Front Distance (US) (mm) ", frontUltraSonicCB.getAverage());

        if (cameraLinearVelocityCB.isEmpty()) {
            telemetry.addData("Linear Velocity (CAM)", "NONE");
        } else {
            telemetry.addData("Linear Velocity (CAM): ", cameraLinearVelocityCB.toString());
        }

        telemetry.addData("Tick Frequency (t/s)", tickFrequency);
        if (updateTelemetry) {
            telemetry.update();
        }
    }
    private void calculateTickTime() throws InterruptedException {
        /*
        This code prints the loop frequency of the REV Control Hub. This frequency is effected
        by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
        of time each cycle takes and finds the frequency (number of updates per second) from
        that cycle time.
         */
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        tickFrequency = 1/loopTime;
        oldTime = newTime;
    }

}
