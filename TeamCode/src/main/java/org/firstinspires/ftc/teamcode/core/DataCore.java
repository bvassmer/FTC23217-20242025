package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.DoubleCircularBuffer;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;
import org.firstinspires.ftc.teamcode.library.LinearVelocityCircularBuffer;
import org.firstinspires.ftc.teamcode.library.PoseCircularBuffer;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class DataCore extends HardwareCore {
    public boolean isMoving = false;
    protected Enum.TeamColor teamColor = Enum.TeamColor.BLUE;
    protected boolean autonomousMode = false;
    protected boolean enableController = false;
    protected double startAngle = 0.0;
    protected int circularBufferCapacity = 100;
    protected final int TOF_SENSOR_CAPACITY = 4;
    // currentPose is the current center of the robot.
    protected PoseCircularBuffer robotPoses = new PoseCircularBuffer(circularBufferCapacity, 3, DistanceUnit.INCH, AngleUnit.DEGREES);
    protected PoseCircularBuffer cameraPoses = new PoseCircularBuffer(circularBufferCapacity, 3, DistanceUnit.INCH, AngleUnit.DEGREES);
    protected PoseCircularBuffer odoPoses = new PoseCircularBuffer(circularBufferCapacity, 3, DistanceUnit.INCH, AngleUnit.DEGREES);
    protected PoseCircularBuffer odoVelocityPoses = new PoseCircularBuffer(circularBufferCapacity, 3, DistanceUnit.INCH, AngleUnit.DEGREES);
    protected DoubleCircularBuffer bearingCB = new DoubleCircularBuffer(circularBufferCapacity, true, true);
    protected DoubleCircularBuffer frontTofCB = new DoubleCircularBuffer(TOF_SENSOR_CAPACITY, true, false);
    protected DoubleCircularBuffer slideTofCB = new DoubleCircularBuffer(TOF_SENSOR_CAPACITY, true, false);
    protected int ultrasonicFrontSensorReading = 19;
    protected double tofFrontSensorReading = 5;
    protected double tofSlideSensorReading = 5;
    protected boolean isLeftTouchSensorPressed = false;
    protected boolean isRightTouchSensorPressed = false;
    protected double clawColorSensorDistance = 0.0;
    protected int clawColorSensorRed = 0;
    protected int clawColorSensorGreen = 0;
    protected int clawColorSensorBlue = 0;
    protected LinearVelocityCircularBuffer cameraLinearVelocityCB = new LinearVelocityCircularBuffer(circularBufferCapacity);
    protected double tickFrequency = 0;
    protected ComponentEnum sensorForDriveControl = ComponentEnum.NONE;
    protected int zeroMotorPowerSensorValue = 100;
    protected int maxMotorPowerSensorValue = 300;
    protected enum ComponentEnum {
        SLIDE_TOF_SENSOR,
        FRONT_TOF_SENSOR,
        FRONT_ULTRASONIC_SENSOR,
        CAMERA,
        CLAW_TOUCH_SENSORS,
        CLAW_COLOR_SENSOR,
        NONE,
    }
    protected enum DebugEnum {
        SENSORS,
        CAMERA,
        DRIVE_MOTORS,
        ODOMETERY,
        TESTING,
    }

    protected Map<ComponentEnum, Boolean> MAP_COMPONENT = Stream.of(new Object[][] {
                    { ComponentEnum.SLIDE_TOF_SENSOR, false },
                    { ComponentEnum.FRONT_ULTRASONIC_SENSOR, false },
                    { ComponentEnum.CAMERA, false },
                    { ComponentEnum.CLAW_TOUCH_SENSORS, false },
                    { ComponentEnum.CLAW_COLOR_SENSOR, false },
            })
            .collect(Collectors.toMap(data -> (ComponentEnum) data[0], data -> (Boolean) data[1]));
    protected Map<DebugEnum, Boolean> MAP_DEBUG = Stream.of(new Object[][] {
                    { DebugEnum.SENSORS, false },
                    { DebugEnum.CAMERA, false },
                    { DebugEnum.DRIVE_MOTORS, true },
                    { DebugEnum.ODOMETERY, false },
                    { DebugEnum.TESTING, false},
            })
            .collect(Collectors.toMap(data -> (DebugEnum) data[0], data -> (Boolean) data[1]));
    protected enum POSE {
        START,
        DROP_OFF,
        DROP_OFF_ALIGNMENT,
        DROP_OFF_BACKUP,
        WAYPOINT_PRE_PICKUP,
        PICKUP_TURN,
        PICKUP,
        PICKUP_TO_DROP_OFF_TRANSITION,
        PICKUP_TO_DROP_OFF_ROTATION,
        PARK,
        WAYPOINT_ONE_SPECIMEN_ONE,
        WAYPOINT_TWO_SPECIMEN_ONE,
        WAYPOINT_THREE_SPECIMEN_ONE,
        PUSH_SPECIMEN_ONE,
        WAYPOINT_ONE_SPECIMEN_TWO,
        WAYPOINT_TWO_SPECIMEN_TWO,
        PUSH_SPECIMEN_TWO,
        WAYPOINT_ONE_SPECIMEN_THREE,
        WAYPOINT_TWO_SPECIMEN_THREE,
        PUSH_SPECIMEN_THREE,
        TEST_START,
        TEST_TURN,
    }
    protected final Map<POSE, Pose2D> MAP_BLUE_POSE = Stream.of(new Object[][] {
                    { POSE.START, new Pose2D(DistanceUnit.INCH, -21.25, 56, AngleUnit.DEGREES, 180)},
                    { POSE.DROP_OFF_ALIGNMENT, new Pose2D(DistanceUnit.INCH, -6, 42, AngleUnit.DEGREES, 180)},
                    { POSE.DROP_OFF, new Pose2D(DistanceUnit.INCH, -6, 30, AngleUnit.DEGREES, 180)},
                    { POSE.DROP_OFF_BACKUP, new Pose2D(DistanceUnit.INCH, -6, 40, AngleUnit.DEGREES, 180)},
                    { POSE.WAYPOINT_PRE_PICKUP, new Pose2D(DistanceUnit.INCH, -51, 36, AngleUnit.DEGREES, 180)},
                    { POSE.PICKUP_TURN, new Pose2D(DistanceUnit.INCH, -51, 36, AngleUnit.DEGREES, 0)},
                    { POSE.PICKUP, new Pose2D(DistanceUnit.INCH, -54, 49, AngleUnit.DEGREES, 0)},
                    { POSE.PICKUP_TO_DROP_OFF_TRANSITION, new Pose2D(DistanceUnit.INCH, -32, 42, AngleUnit.DEGREES, 0)},
                    { POSE.PICKUP_TO_DROP_OFF_ROTATION, new Pose2D(DistanceUnit.INCH, -32, 42, AngleUnit.DEGREES, 180)},
                    { POSE.PARK, new Pose2D(DistanceUnit.INCH, -70, 46, AngleUnit.DEGREES, 180)},
                    { POSE.WAYPOINT_ONE_SPECIMEN_ONE, new Pose2D(DistanceUnit.INCH, -43, 35, AngleUnit.DEGREES, 180)},
                    { POSE.WAYPOINT_TWO_SPECIMEN_ONE, new Pose2D(DistanceUnit.INCH, -43, 3, AngleUnit.DEGREES, 180)},
                    { POSE.WAYPOINT_THREE_SPECIMEN_ONE, new Pose2D(DistanceUnit.INCH, -49, 3, AngleUnit.DEGREES, 180)},
                    { POSE.PUSH_SPECIMEN_ONE, new Pose2D(DistanceUnit.INCH, -49, 46, AngleUnit.DEGREES, 180)},
                    { POSE.WAYPOINT_ONE_SPECIMEN_TWO, new Pose2D(DistanceUnit.INCH, -49, 3, AngleUnit.DEGREES, 180)},
                    { POSE.WAYPOINT_TWO_SPECIMEN_TWO, new Pose2D(DistanceUnit.INCH, -60, 3, AngleUnit.DEGREES, 180)},
                    { POSE.PUSH_SPECIMEN_TWO, new Pose2D(DistanceUnit.INCH, -60, 46, AngleUnit.DEGREES, 180)},
                    { POSE.WAYPOINT_ONE_SPECIMEN_THREE, new Pose2D(DistanceUnit.INCH, -60, 3, AngleUnit.DEGREES, 180)},
                    { POSE.WAYPOINT_TWO_SPECIMEN_THREE, new Pose2D(DistanceUnit.INCH, -66, 3, AngleUnit.DEGREES, 180)},
                    { POSE.PUSH_SPECIMEN_THREE, new Pose2D(DistanceUnit.INCH, -66, 46, AngleUnit.DEGREES, 180)},
                    { POSE.TEST_START, new Pose2D(DistanceUnit.INCH, -21.25, 56, AngleUnit.DEGREES, 180)},
                    { POSE.TEST_TURN, new Pose2D(DistanceUnit.INCH, -45, 35, AngleUnit.DEGREES, 270)},
            })
            .collect(Collectors.toMap(data -> (POSE) data[0], data -> (Pose2D) data[1]));

    protected final Map<AutoDriveCore.GenericMoveStepState, Pose2D> MAP_BLUE_STEP_POSE = Stream.of(new Object[][] {
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_ONE_SPECIMEN_ONE, MAP_BLUE_POSE.get(POSE.WAYPOINT_ONE_SPECIMEN_ONE) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_TWO_SPECIMEN_ONE, MAP_BLUE_POSE.get(POSE.WAYPOINT_TWO_SPECIMEN_ONE) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_THREE_SPECIMEN_ONE, MAP_BLUE_POSE.get(POSE.WAYPOINT_THREE_SPECIMEN_ONE) },
                    { AutoDriveCore.GenericMoveStepState.PUSH_SPECIMEN_ONE, MAP_BLUE_POSE.get(POSE.PUSH_SPECIMEN_ONE) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_ONE_SPECIMEN_TWO, MAP_BLUE_POSE.get(POSE.WAYPOINT_ONE_SPECIMEN_TWO) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_TWO_SPECIMEN_TWO, MAP_BLUE_POSE.get(POSE.WAYPOINT_TWO_SPECIMEN_TWO) },
                    { AutoDriveCore.GenericMoveStepState.PUSH_SPECIMEN_TWO, MAP_BLUE_POSE.get(POSE.PUSH_SPECIMEN_TWO) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_ONE_SPECIMEN_THREE, MAP_BLUE_POSE.get(POSE.WAYPOINT_ONE_SPECIMEN_THREE) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_TWO_SPECIMEN_THREE, MAP_BLUE_POSE.get(POSE.WAYPOINT_TWO_SPECIMEN_THREE) },
                    { AutoDriveCore.GenericMoveStepState.PUSH_SPECIMEN_THREE, MAP_BLUE_POSE.get(POSE.PUSH_SPECIMEN_THREE) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_PRE_PICKUP, MAP_BLUE_POSE.get(POSE.WAYPOINT_PRE_PICKUP) },
                    { AutoDriveCore.GenericMoveStepState.PICKUP_TURN, MAP_BLUE_POSE.get(POSE.PICKUP_TURN) },
            })
            .collect(Collectors.toMap(data -> (AutoDriveCore.GenericMoveStepState) data[0], data -> (Pose2D) data[1]));

    protected final Map<POSE, Pose2D> MAP_RED_POSE = Stream.of(new Object[][] {
                    { POSE.START, new Pose2D(DistanceUnit.INCH, 21.25, -56, AngleUnit.DEGREES, 0)},
                    { POSE.DROP_OFF_ALIGNMENT, new Pose2D(DistanceUnit.INCH, 6, -42, AngleUnit.DEGREES, 0)},
                    { POSE.DROP_OFF, new Pose2D(DistanceUnit.INCH, 6, -30, AngleUnit.DEGREES, 0)},
                    { POSE.DROP_OFF_BACKUP, new Pose2D(DistanceUnit.INCH, 6, -40, AngleUnit.DEGREES, 0)},
                    { POSE.WAYPOINT_PRE_PICKUP, new Pose2D(DistanceUnit.INCH, 51, -36, AngleUnit.DEGREES, 0)},
                    { POSE.PICKUP_TURN, new Pose2D(DistanceUnit.INCH, 51, -36, AngleUnit.DEGREES, 180)},
                    { POSE.PICKUP, new Pose2D(DistanceUnit.INCH, 54, -49, AngleUnit.DEGREES, 180)},
                    { POSE.PICKUP_TO_DROP_OFF_TRANSITION, new Pose2D(DistanceUnit.INCH, 32, -42, AngleUnit.DEGREES, 180)},
                    { POSE.PICKUP_TO_DROP_OFF_ROTATION, new Pose2D(DistanceUnit.INCH, 32, -42, AngleUnit.DEGREES, 0)},
                    { POSE.PARK, new Pose2D(DistanceUnit.INCH, 70, -46, AngleUnit.DEGREES, 0)},
                    { POSE.WAYPOINT_ONE_SPECIMEN_ONE, new Pose2D(DistanceUnit.INCH, 42, -35, AngleUnit.DEGREES, 0)},
                    { POSE.WAYPOINT_TWO_SPECIMEN_ONE, new Pose2D(DistanceUnit.INCH, 42, -3, AngleUnit.DEGREES, 0)},
                    { POSE.WAYPOINT_THREE_SPECIMEN_ONE, new Pose2D(DistanceUnit.INCH, 49, -3, AngleUnit.DEGREES, 0)},
                    { POSE.PUSH_SPECIMEN_ONE, new Pose2D(DistanceUnit.INCH, 49, -46, AngleUnit.DEGREES, 0)},
                    { POSE.WAYPOINT_ONE_SPECIMEN_TWO, new Pose2D(DistanceUnit.INCH, 49, -3, AngleUnit.DEGREES, 0)},
                    { POSE.WAYPOINT_TWO_SPECIMEN_TWO, new Pose2D(DistanceUnit.INCH, 60, -3, AngleUnit.DEGREES, 0)},
                    { POSE.PUSH_SPECIMEN_TWO, new Pose2D(DistanceUnit.INCH, 60, -46, AngleUnit.DEGREES, 0)},
                    { POSE.WAYPOINT_ONE_SPECIMEN_THREE, new Pose2D(DistanceUnit.INCH, 60, -3, AngleUnit.DEGREES, 0)},
                    { POSE.WAYPOINT_TWO_SPECIMEN_THREE, new Pose2D(DistanceUnit.INCH, 66, -3, AngleUnit.DEGREES, 0)},
                    { POSE.PUSH_SPECIMEN_THREE, new Pose2D(DistanceUnit.INCH, 66, -46, AngleUnit.DEGREES, 0)},
                    { POSE.TEST_START, new Pose2D(DistanceUnit.INCH, 60, -36, AngleUnit.DEGREES, 0)},
                    { POSE.TEST_TURN, new Pose2D(DistanceUnit.INCH, 60, -36, AngleUnit.DEGREES, 180)},
            })
            .collect(Collectors.toMap(data -> (POSE) data[0], data -> (Pose2D) data[1]));

    protected final Map<AutoDriveCore.GenericMoveStepState, Pose2D> MAP_RED_STEP_POSE = Stream.of(new Object[][] {
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_ONE_SPECIMEN_ONE, MAP_RED_POSE.get(POSE.WAYPOINT_ONE_SPECIMEN_ONE) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_TWO_SPECIMEN_ONE, MAP_RED_POSE.get(POSE.WAYPOINT_TWO_SPECIMEN_ONE) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_THREE_SPECIMEN_ONE, MAP_RED_POSE.get(POSE.WAYPOINT_THREE_SPECIMEN_ONE) },
                    { AutoDriveCore.GenericMoveStepState.PUSH_SPECIMEN_ONE, MAP_RED_POSE.get(POSE.PUSH_SPECIMEN_ONE) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_ONE_SPECIMEN_TWO, MAP_RED_POSE.get(POSE.WAYPOINT_ONE_SPECIMEN_TWO) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_TWO_SPECIMEN_TWO, MAP_RED_POSE.get(POSE.WAYPOINT_TWO_SPECIMEN_TWO) },
                    { AutoDriveCore.GenericMoveStepState.PUSH_SPECIMEN_TWO, MAP_RED_POSE.get(POSE.PUSH_SPECIMEN_TWO) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_ONE_SPECIMEN_THREE, MAP_RED_POSE.get(POSE.WAYPOINT_ONE_SPECIMEN_THREE) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_TWO_SPECIMEN_THREE, MAP_RED_POSE.get(POSE.WAYPOINT_TWO_SPECIMEN_THREE) },
                    { AutoDriveCore.GenericMoveStepState.PUSH_SPECIMEN_THREE, MAP_RED_POSE.get(POSE.PUSH_SPECIMEN_THREE) },
                    { AutoDriveCore.GenericMoveStepState.WAYPOINT_PRE_PICKUP, MAP_RED_POSE.get(POSE.WAYPOINT_PRE_PICKUP) },
                    { AutoDriveCore.GenericMoveStepState.PICKUP_TURN, MAP_RED_POSE.get(POSE.PICKUP_TURN) },
            })
            .collect(Collectors.toMap(data -> (AutoDriveCore.GenericMoveStepState) data[0], data -> (Pose2D) data[1]));

    public void runOpMode() throws InterruptedException {
        Log.d("FTC-23217", "DataCore Start.");
        super.runOpMode(autonomousMode);
        outputTelemetry(true);
    }

    protected void workers() throws InterruptedException {
        super.workers();
        calculateTickTime();
        outputTelemetry(false);
    }

    public void calculate() throws InterruptedException {
        this.cameraLinearVelocityCB.addAndCalculate(new LinearVelocity(cameraPoses.getAverageSpeed(), cameraPoses.getAverageHeading()));
    }

    private void outputTelemetry(boolean updateTelemetry) throws InterruptedException {
        if (autonomousMode || Boolean.TRUE.equals(MAP_DEBUG.get(DebugEnum.ODOMETERY))) {
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
            /* if (odoPoses.isEmpty()) {
                telemetry.addData("Pose (ODO)", "NONE");
            } else {
                telemetry.addData("Pose (ODO)", "x:" + odoPoses.getLatestX() + " y:" + odoPoses.getLatestY() + " h:" + odoPoses.getLatestHeading());
            }
            if (odoVelocityPoses.isEmpty()) {
                telemetry.addData("Velocity (ODO)", "NONE");
            } else {
                telemetry.addData("Velocity (ODO)", "x:" + odoVelocityPoses.getLatestX() + " y:" + odoVelocityPoses.getLatestY() + " h:" + odoVelocityPoses.getLatestHeading());
            } */
            telemetry.addData("Robot Bearing (CAM) (CB)", bearingCB.getAverage());
            if (cameraLinearVelocityCB.isEmpty()) {
                telemetry.addData("Linear Velocity (CAM)", "NONE");
            } else {
                telemetry.addData("Linear Velocity (CAM): ", cameraLinearVelocityCB.toString());
            }
            telemetry.addData("Pose (ODO)", "X:" + odo.getPosition().getX(DistanceUnit.INCH) + " Y:" + odo.getPosition().getY(DistanceUnit.INCH) + " Heading(non-norm):" + Math.toDegrees(odo.getHeading()) + " Heading(norm):" +odo.getPosition().getHeading(AngleUnit.DEGREES));
        }

        telemetry.addData("Slide Distance (CB) (mm) ", slideTofCB.getAverage());
        telemetry.addData("Slide Distance (Direct) (mm) ", tofSlideSensorReading);
        telemetry.addData("Tick Frequency (t/s)", tickFrequency);
        if (autonomousMode || Boolean.TRUE.equals(this.MAP_DEBUG.get(DebugEnum.SENSORS))) {
            telemetry.addData("Front Distance (US) (mm) ", this.ultrasonicFrontSensorReading);
            telemetry.addData("Front Distance (ToF) (mm) ", this.tofFrontSensorReading);

            telemetry.addData("Claw Color Sensor Distance (mm) ", clawColorSensorDistance);
            telemetry.addData("Claw Color Sensor Red ", clawColorSensorRed);
            telemetry.addData("Claw Color Sensor Green ", clawColorSensorGreen);
            telemetry.addData("Claw Color Sensor Blue ", clawColorSensorBlue);
            telemetry.addData("Claw Touch Pressed (Left) ", isLeftTouchSensorPressed);
            telemetry.addData("Claw Touch Pressed (Right) ", isRightTouchSensorPressed);
        }


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
