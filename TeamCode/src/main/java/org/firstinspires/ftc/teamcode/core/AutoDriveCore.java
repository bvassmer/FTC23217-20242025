package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;

import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;


public class AutoDriveCore extends MechamCore {

    private Pose2D currentDestination;
    private double desiredMovementBearing;
    private double desiredXPower = 0.0;
    private double desiredYPower = 0.0;
    private double desiredRxPower = 0.0;
    private double DROPOFF_PIVOT_DELAY = 0.4;
    private final ElapsedTime dropoffPivotTimer = new ElapsedTime();
    public MoveToDropoffState moveToDropoffState = MoveToDropoffState.ROTATE_SLIDE_TO_DROPOFF;
    public DropoffState dropoffState = DropoffState.PIVOT_LIFT_ARM;
    public int currentTaskStep = 0;
    public AutoDriveState autoDriveState = AutoDriveState.WAITING;
    public StepState stepState = StepState.START;
    private ElapsedTime moveToDropoffTimer = new ElapsedTime();
    private double MOVE_TO_DROPOFF_DELAY = 0.75;


    // Center of course is 0, 0

    // X can be -72 to 72 in inches (144 inches or ~3580 mm across)
    // X is positive away from the audience. X is positive to the right direction looking from the Red Alliance. X is positive to the left direction looking from the Blue Alliance.
    // X is negative toward teh audience. X is negative to the left direction looking from the Red Alliance. X is negative to the right direction looking from the Blue Alliance.

    // Y can be -72 to 72 in inches (144 inches or ~3580 mm across)
    // Y is positive on the Blue Alliance side of the course.
    // Y is negative on the Red Alliance side of the course.

    // each tile is about 24 in x 24 in
    // robot size is max 18 in x 18 in
    private final Pose2D RED_RIGHT_START_POSE = new Pose2D(DistanceUnit.INCH, 24, -54, AngleUnit.DEGREES, 0);
    private final Pose2D RED_RIGHT_DROPOFF_POSE = new Pose2D(DistanceUnit.INCH, 3, -32, AngleUnit.DEGREES, 0);

    public void runOpMode(AutoDriveState startMode) throws InterruptedException {
        Log.d("FTC-23217-AutoDriveCore", "AutoDriveCore Start. Color:" + teamColor);
        autoDriveState = startMode;
        super.runOpMode();
    }

    public void workers() throws InterruptedException {
        super.workers(desiredXPower, desiredYPower, desiredRxPower);
        if (this.autonomousMode) {
            autoDriveStateMachine();
            telemetryOutput();
        }
    }

    private void telemetryOutput() throws InterruptedException {
        telemetry.addData("AutoDriveCode: autonomousMode", autonomousMode);
        telemetry.addData("AutoDriveCode: AutoDriveState: ", autoDriveState);
        telemetry.addData("AutoDriveCode: StepState: ", stepState);
        telemetry.addData("AutoDriveCore: Desired Movement Bearing:" , desiredMovementBearing);
        telemetry.addData("AutoDriveCore: desiredXPower: ", desiredXPower);
        telemetry.addData("AutoDriveCore: desiredYPower: ", desiredYPower);
        telemetry.addData("AutoDriveCore: desiredRxPower: ", desiredRxPower);
    }

    private void updatePreviousPoses(boolean move) throws  InterruptedException {
    }

    private void calculateMovementBearing() throws InterruptedException {
        double x1 = odo.getPosition().getX(DistanceUnit.INCH);
        double y1 = odo.getPosition().getY(DistanceUnit.INCH);
        double x2 = currentDestination.getX(DistanceUnit.INCH);
        double y2 = currentDestination.getY(DistanceUnit.INCH);

        // Calculate differences in coordinates
        double deltaX = x2 - x1;
        double deltaY = y2 - y1;

        // Use Math.atan2 to calculate the angle in radians
        double angleRad = Math.atan2(deltaX, deltaY);

        // Convert angle to degrees
        double angleDeg = Math.toDegrees(angleRad);

        // Normalize the angle to the range [0°, 360°]
        double movementBearing = (angleDeg + 360) % 360;

        Log.d("FTC-23217-AutoDriveCore", "calculateMovementBearing: angleDeg:" + angleDeg + " desiredMovementBearing:" + desiredMovementBearing);

        desiredMovementBearing = movementBearing;
    }

    // Function to break speed into x and y components from bearing
    public void calculateSpeedComponents() throws InterruptedException {
        // Convert bearing from degrees to radians
        double radians = Math.toRadians(desiredMovementBearing);
        double robotHeading = odo.getPosition().getHeading(AngleUnit.RADIANS); // Get robot's current heading

        // Adjust bearing relative to the robot's heading
        double adjustedRadians = radians - robotHeading;

        double x1 = odo.getPosition().getX(DistanceUnit.INCH);
        double y1 = odo.getPosition().getY(DistanceUnit.INCH);
        double x2 = currentDestination.getX(DistanceUnit.INCH);
        double y2 = currentDestination.getY(DistanceUnit.INCH);


        boolean withIn5 = isXYWithinBuffer(x1, y1, x2, y2, 5);
        boolean withIn10 = isXYWithinBuffer(x1, y1, x2, y2, 10);
        boolean withIn20 = isXYWithinBuffer(x1, y1, x2, y2, 20);
        boolean withIn40 = isXYWithinBuffer(x1, y1, x2, y2, 40);
        boolean withIn100 = isXYWithinBuffer(x1, y1, x2, y2, 100);

        double desiredSpeed = 0.0;
        if (withIn5) {
            desiredSpeed = 0.3;
        } else if (withIn10) {
            desiredSpeed = 0.35;
        } else if (withIn20) {
            desiredSpeed = 0.5;
        } else if (withIn40) {
            desiredSpeed = 0.7;
        } else if (withIn100) {
            desiredSpeed = 0.8;
        }

        // Calculate x and y components of speed
        double xSpeed = desiredSpeed * Math.sin(adjustedRadians);  // x component (horizontal speed)
        double ySpeed = desiredSpeed * Math.cos(adjustedRadians);  // y component (vertical speed)

        // TODO: Make sure this still works now that we are accounting for angle of robot heading.
        /* if (teamColor == Enum.TeamColor.BLUE) {
            xSpeed *= -1;
            ySpeed *= -1;
        } */

        Log.d("FTC-23217-AutoDriveCore", "calculateSpeedComponents: desiredSpeed:" + desiredSpeed + " xSpeed:" + xSpeed + " ySpeed:" + ySpeed);

        // Return the components as an array: [x, y]
        desiredXPower = xSpeed;
        desiredYPower = ySpeed;
    }

    private void calculateAngularMovement() throws InterruptedException {
        double currentAngularBearing = odo.getPosition().getHeading(AngleUnit.DEGREES);
        double currentDesiredAngularBearing = currentDestination.getHeading(AngleUnit.DEGREES);

        // Normalize the angles to be within [0, 360)
        currentAngularBearing = (currentAngularBearing + 360) % 360;
        currentDesiredAngularBearing = (currentDesiredAngularBearing + 360) % 360;

        // Calculate the difference in angles
        double delta = currentDesiredAngularBearing - currentAngularBearing;

        // Normalize the difference to the range [-180, 180]
        if (delta > 180) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }

        // Determine the direction and the angle to turn
        double turnAngle = Math.abs(delta);

        // String direction = delta > 0 ? "right" : "left";
        if (delta > 60) {
            desiredRxPower = 0.3;
        } else if (delta > 20) {
            desiredRxPower = 0.2;
        } else if (delta > 0.5) {
            desiredRxPower = 0.1;
        } else if (delta < -60) {
            desiredRxPower = -0.3;
        } else if (delta < -20) {
            desiredRxPower = -0.2;
        } else if (delta < -0.5) {
            desiredRxPower = -0.1;
        } else {
            desiredRxPower = 0.0;
        }

        if (teamColor == Enum.TeamColor.BLUE) {
            desiredRxPower *= -1;
        }
        Log.d("FTC-23217-AutoDriveCore", "calculateAngularMovement: delta:" + delta + " desiredRxPower:" + desiredRxPower);
    }

    private boolean atDestination() throws InterruptedException {
        boolean distanceGood = isDistanceGood();
        boolean angleGood = isAngleGood();
        if (distanceGood && angleGood) {
            return true;
        }
        return false;
    }

    private boolean isXYWithinBuffer(double x1, double y1, double x2, double y2, double buffer) throws InterruptedException {
        double distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
        return distance <= buffer;
    }

    private boolean isHeadingWithinBuffer(double headingToCheck, double targetHeading, double buffer) throws InterruptedException {
        // Normalize all angles to be between 0 and 360 degrees
        headingToCheck = (headingToCheck + 360) % 360;
        targetHeading = (targetHeading + 360) % 360;

        // Calculate the lower and upper bounds of the range
        double lowerBound = (targetHeading - buffer + 360) % 360;
        double upperBound = (targetHeading + buffer) % 360;

        // Check if the number is within the range
        if (lowerBound <= upperBound) {
            // Normal range (no wraparound)
            return headingToCheck >= lowerBound && headingToCheck <= upperBound;
        } else {
            // Range wraps around 360
            return headingToCheck >= lowerBound || headingToCheck <= upperBound;
        }
    }

    private boolean isAngleGood() throws InterruptedException {
        return isHeadingWithinBuffer(
                odo.getPosition().getHeading(AngleUnit.DEGREES),
                currentDestination.getHeading(AngleUnit.DEGREES),
                4);
    }
    private boolean isDistanceGood() throws InterruptedException {
        return isXYWithinBuffer(
                    odo.getPosition().getX(DistanceUnit.INCH),
                    odo.getPosition().getY(DistanceUnit.INCH),
                    currentDestination.getX(DistanceUnit.INCH),
                    currentDestination.getY(DistanceUnit.INCH),
                   4);
    }

    private void calculateMovements(boolean canDirection, boolean canRotate) throws InterruptedException {
        if (!atDestination()) {
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: Moving");
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: currentX (odo)" + odo.getPosition().getX(DistanceUnit.INCH) + " currentY:" + odo.getPosition().getY(DistanceUnit.INCH));
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: destX" + currentDestination.getX(DistanceUnit.INCH) + " destY:" + currentDestination.getY(DistanceUnit.INCH));
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: currHeading (odo)" + odo.getPosition().getHeading(AngleUnit.DEGREES));
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: destHeading" + currentDestination.getHeading(AngleUnit.DEGREES));

            if (isDistanceGood()) {
                Log.d("FTC-23217-AutoDriveCore", "calculateMovements: Distance is good. Stopping movement.");
                stopMoving(true, false);
            } else {
                Log.d("FTC-23217-AutoDriveCore", "calculateMovements: Moving.");
                if (canDirection) {
                    // find direction we need to move
                    calculateMovementBearing();
                    // find x, y power required to move desired direction
                    calculateSpeedComponents();
                }
            }

            if (isAngleGood()) {
                Log.d("FTC-23217-AutoDriveCore", "calculateMovements: Angle is good. Stopping rotation.");
                stopMoving(false, true);
            } else {
                Log.d("FTC-23217-AutoDriveCore", "calculateMovements: Rotating.");
                if (canRotate) {
                    // find power required to rotate to desired bearing
                    calculateAngularMovement();
                }
            }
        } else {
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: At Destination!");
        }
    }

    public enum AutoDriveState {
        WAITING,
        DRIVING_HANGING_PARTS_START,
        DRIVING_PARK_START,
        DRIVING,
        STOP,
    }
    private void autoDriveStateMachine() throws InterruptedException {
        Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: autoDriveState:" + autoDriveState);
        Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: stepState:" + stepState);
        Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: dropoffState:" + dropoffState);
        Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: moveToDropoffState:" + moveToDropoffState);
        Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: odoX:" + odo.getPosition().getX(DistanceUnit.INCH) + " odoY:" + odo.getPosition().getY(DistanceUnit.INCH) + " odoH:" + odo.getPosition().getHeading(AngleUnit.DEGREES));
        updatePreviousPoses(true);
        switch (autoDriveState) {
            case WAITING:
                stopMoving(true, true);
                break;
            case DRIVING_HANGING_PARTS_START:
                stepState = StepState.NEXT_STEP;
                autoDriveState = AutoDriveState.DRIVING;
                stepStateMachine();
                break;
            case DRIVING_PARK_START:
                stepState = StepState.PARK;
                autoDriveState = AutoDriveState.DRIVING;
                stepStateMachine();
                break;
            case DRIVING:
                stepStateMachine();
                break;
            case STOP:
                stopMoving(true, true);
                break;
        }
    }
    private void stopMoving(boolean movement, boolean angle) throws InterruptedException {
        if (movement) {
            desiredXPower = 0.0;
            desiredYPower = 0.0;
            stopMovement();
        }
        if (angle) {
            desiredRxPower = 0.0;
        }
    }

    private void resetStateMachines() throws InterruptedException {
        moveToDropoffState = MoveToDropoffState.ROTATE_SLIDE_TO_DROPOFF;
        dropoffState = DropoffState.PIVOT_LIFT_ARM;
    }

    private void resetComponentMap() throws InterruptedException {
        MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, false);
        MAP_COMPONENT.put(ComponentEnum.FRONT_ULTRASONIC_SENSOR, false);
        MAP_COMPONENT.put(ComponentEnum.CAMERA, false);
        MAP_COMPONENT.put(ComponentEnum.CLAW_TOUCH_SENSORS, false);
        MAP_COMPONENT.put(ComponentEnum.CLAW_COLOR_SENSOR, false);
    }

    private void resetMotorPowerSensorData() throws InterruptedException {
        sensorForDriveControl = ComponentEnum.NONE;
        zeroMotorPowerSensorValue = 500;
        maxMotorPowerSensorValue = 100000;
    }

    private static final StepState[] TASK_LIST = new StepState[]{
            StepState.START,
            StepState.MOVE_TO_DROPOFF,
            StepState.DROPOFF,
            StepState.PUSH_SPECIMENS,
            StepState.MOVE_TO_WAYPOINT_ONE_SPECIMEN_ONE,
            StepState.MOVE_TO_WAYPOINT_TWO_SPECIMEN_ONE,
            StepState.MOVE_TO_WAYPOINT_THREE_SPECIMEN_ONE,
            StepState.MOVE_TO_PUSH_SPECIMEN_ONE,
            StepState.MOVE_TO_WAYPOINT_ONE_SPECIMEN_TWO,
            StepState.MOVE_TO_WAYPOINT_TWO_SPECIMEN_TWO,
            StepState.MOVE_TO_PUSH_SPECIMEN_TWO,
            StepState.MOVE_TO_WAYPOINT_ONE_SPECIMEN_THREE,
            StepState.MOVE_TO_WAYPOINT_TWO_SPECIMEN_THREE,
            StepState.MOVE_TO_PUSH_SPECIMEN_THREE,
            StepState.MOVE_TO_PICKUP_TURN,
            /* StepState.PICKUP,
            StepState.MOVE_TO_DROPOFF,
            StepState.DROPOFF,
            StepState.PICKUP,
            StepState.MOVE_TO_DROPOFF,
            StepState.DROPOFF,
            StepState.PICKUP,
            StepState.MOVE_TO_DROPOFF,
            StepState.DROPOFF,
            StepState.PARK, */
            StepState.STOP
    };
    public enum StepState {
        START,
        NEXT_STEP,
        MOVE_TO_DROPOFF,
        DROPOFF,
        PUSH_SPECIMENS,
        PICKUP,
        PARK,
        STOP,
    }
    public enum GenericMoveStepState {
        WAYPOINT_ONE_SPECIMEN_ONE, // Move from dropoff to corner turn to avoid middle structure.
        WAYPOINT_TWO_SPECIMEN_ONE, // Move forward to get behind ground specimens
        WAYPOINT_THREE_SPECIMEN_ONE, // Move to right to get lined up with specimen one.
        PUSH_SPECIMEN_ONE, // Push specimen one into parking zone
        WAYPOINT_ONE_SPECIMEN_TWO, // Move forward to get behind specimens (should be same location as MOVE_TO_WAYPOINT_THREE_SPECIMEN_ONE)
        WAYPOINT_TWO_SPECIMEN_TWO, // Move to right to get lined up with specimen two
        PUSH_SPECIMEN_TWO, // Push specimen two into parking zone
        WAYPOINT_ONE_SPECIMEN_THREE, // Move forward to get behind specimen three. (should be same location as MOVE_TO_WAYPOINT_TWO_SPECIMEN_TWO)
        WAYPOINT_TWO_SPECIMEN_THREE, // Move to right to get lined up with specimen three
        PUSH_SPECIMEN_THREE, // Push specimen three into parking zone
        PICKUP_TURN, // Reset out of the park zone and line up with pickup location. turn around. PICKUP should be after this.
        NEXT_STEP,
        DONE,
    }
    private void stepStateMachine() throws InterruptedException {
        switch (stepState) {
            case START:
                stepState = TASK_LIST[currentTaskStep];
                break;
            case NEXT_STEP:
                currentTaskStep += 1;
                resetStateMachines();
                resetComponentMap();
                resetMotorPowerSensorData();
                stepState = TASK_LIST[currentTaskStep];
                break;
            case MOVE_TO_DROPOFF:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                if (moveToDropoffState == MoveToDropoffState.DONE) {
                    stepState = StepState.NEXT_STEP;
                } else {
                    moveToDropoffStateMachine();
                }
                break;
            case DROPOFF:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                MAP_COMPONENT.put(ComponentEnum.FRONT_TOF_SENSOR, true);

                if (dropoffState == DropoffState.DONE) {
                    stepState = StepState.NEXT_STEP;
                } else {
                    dropoffStateMachine();
                }
                break;
            case MOVE_TO_WAYPOINT_ONE_SPECIMEN_ONE:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.WAYPOINT_ONE_SPECIMEN_ONE);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.WAYPOINT_ONE_SPECIMEN_ONE);
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, true);
                } else {
                    stepState = StepState.MOVE_TO_WAYPOINT_TWO_SPECIMEN_ONE;
                }
                break;
            case MOVE_TO_WAYPOINT_TWO_SPECIMEN_ONE:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.WAYPOINT_TWO_SPECIMEN_ONE);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.WAYPOINT_TWO_SPECIMEN_ONE);
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, true);
                } else {
                    stepState = StepState.MOVE_TO_WAYPOINT_THREE_SPECIMEN_ONE;
                }
                break;
            case MOVE_TO_WAYPOINT_THREE_SPECIMEN_ONE:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.WAYPOINT_THREE_SPECIMEN_ONE);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.WAYPOINT_THREE_SPECIMEN_ONE);
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, true);
                } else {
                    stepState = StepState.MOVE_TO_PUSH_SPECIMEN_ONE;
                }
                break;
            case MOVE_TO_PUSH_SPECIMEN_ONE:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.PUSH_SPECIMEN_ONE);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.PUSH_SPECIMEN_ONE);
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, true);
                } else {
                    stepState = StepState.MOVE_TO_WAYPOINT_ONE_SPECIMEN_TWO;
                }
                break;
            case MOVE_TO_WAYPOINT_ONE_SPECIMEN_TWO:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.WAYPOINT_ONE_SPECIMEN_TWO);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.WAYPOINT_ONE_SPECIMEN_TWO);
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, true);
                } else {
                    stepState = StepState.MOVE_TO_WAYPOINT_TWO_SPECIMEN_TWO;
                }
                break;
            case MOVE_TO_WAYPOINT_TWO_SPECIMEN_TWO:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.WAYPOINT_TWO_SPECIMEN_TWO);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.WAYPOINT_TWO_SPECIMEN_TWO);
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, true);
                } else {
                    stepState = StepState.MOVE_TO_PUSH_SPECIMEN_TWO;
                }
                break;
            case MOVE_TO_PUSH_SPECIMEN_TWO:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.PUSH_SPECIMEN_TWO);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.PUSH_SPECIMEN_TWO);
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, true);
                } else {
                    stepState = StepState.MOVE_TO_WAYPOINT_ONE_SPECIMEN_THREE;
                }
                break;
            case MOVE_TO_WAYPOINT_ONE_SPECIMEN_THREE:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.WAYPOINT_ONE_SPECIMEN_THREE);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.WAYPOINT_ONE_SPECIMEN_THREE);
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, true);
                } else {
                    stepState = StepState.MOVE_TO_WAYPOINT_TWO_SPECIMEN_THREE;
                }
                break;
            case MOVE_TO_WAYPOINT_TWO_SPECIMEN_THREE:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.WAYPOINT_TWO_SPECIMEN_THREE);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.WAYPOINT_TWO_SPECIMEN_THREE);
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, true);
                } else {
                    stepState = StepState.MOVE_TO_PUSH_SPECIMEN_THREE;
                }
                break;
            case MOVE_TO_PUSH_SPECIMEN_THREE:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.PUSH_SPECIMEN_THREE);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.PUSH_SPECIMEN_THREE);
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, true);
                } else {
                    /* switch (teamColor) {
                        case BLUE:
                            odo.setPosition(new Pose2D(
                                    DistanceUnit.INCH,
                                    -69.0,
                                    45.7,
                                    AngleUnit.DEGREES,
                                    180
                            ));
                            break;
                        case RED:
                            odo.setPosition(new Pose2D(
                                    DistanceUnit.INCH,
                                    69.0,
                                    -45.7,
                                    AngleUnit.DEGREES,
                                    0
                            ));
                            break;
                    }*/
                    stepState = StepState.MOVE_TO_PICKUP_TURN;
                }
                break;
            case MOVE_TO_PICKUP_TURN:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.PICKUP_TURN);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.PICKUP_TURN);
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, true);
                } else {
                    stepState = StepState.STOP;
                }
                break;
            case PICKUP:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                MAP_COMPONENT.put(ComponentEnum.FRONT_TOF_SENSOR, true);
                MAP_COMPONENT.put(ComponentEnum.FRONT_ULTRASONIC_SENSOR, true);
                MAP_COMPONENT.put(ComponentEnum.CLAW_TOUCH_SENSORS, true);
                break;
            case PARK:
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.PARK);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.PARK);
                        break;
                }
                rotationLiftState = RotationLiftState.PARK;
                // updatePreviousPoses(true);
                if (atDestination()) {
                    autoDriveState = AutoDriveState.STOP;
                } else {
                    calculateMovements(true, true);
                }
                break;
            case STOP:
            default:
                autoDriveState = AutoDriveState.STOP;
                stepState = StepState.STOP;
                break;
        }
    }

    public enum MoveToDropoffState {
        ROTATE_SLIDE_TO_DROPOFF,
        MOVE_SLIDE_TO_DROPOFF,
        MOVE_WRIST_TO_DROPOFF,
        MOVE_TO_DROPOFF_ALIGNMENT,
        MOVE_TO_DROPOFF,
        DONE,

    }
    private void moveToDropoffStateMachine() throws InterruptedException {
       switch (moveToDropoffState) {
           case ROTATE_SLIDE_TO_DROPOFF:
               moveToDropoffTimer.reset();
               rotationLiftState = RotationLiftState.MOVE_TO_DROPOFF;
               moveToDropoffState = MoveToDropoffState.MOVE_SLIDE_TO_DROPOFF;
               break;
           case MOVE_SLIDE_TO_DROPOFF:
               slideState = SLIDE_STATE.AUTO_DROPOFF;
               moveToDropoffState = MoveToDropoffState.MOVE_WRIST_TO_DROPOFF;
               break;
           case MOVE_WRIST_TO_DROPOFF:
               clawPivotState = ClawPivotState.MOVE_TO_DROPOFF;
               if (moveToDropoffTimer.seconds() > MOVE_TO_DROPOFF_DELAY) {
                   moveToDropoffState = MoveToDropoffState.MOVE_TO_DROPOFF;
               }
               break;
           case MOVE_TO_DROPOFF_ALIGNMENT:
               sensorForDriveControl = ComponentEnum.FRONT_TOF_SENSOR;
               switch (teamColor) {
                   case BLUE:
                       currentDestination = MAP_BLUE_POSE.get(POSE.DROP_OFF_ALIGNMENT);
                       break;
                   case RED:
                       currentDestination = MAP_RED_POSE.get(POSE.DROP_OFF_ALIGNMENT);
                       break;
               }
               if (!atDestination()) {
                   zeroMotorPowerSensorValue = 30;
                   maxMotorPowerSensorValue = 200;
                   calculateMovements(true, true);
               } else {
                   moveToDropoffState = MoveToDropoffState.MOVE_TO_DROPOFF;
               }
               break;
           case MOVE_TO_DROPOFF:
               sensorForDriveControl = ComponentEnum.FRONT_TOF_SENSOR;
               switch (teamColor) {
                   case BLUE:
                       currentDestination = MAP_BLUE_POSE.get(POSE.DROP_OFF);
                       break;
                   case RED:
                       currentDestination = MAP_RED_POSE.get(POSE.DROP_OFF);
                       break;
               }
               if (!atDestination()) {
                   zeroMotorPowerSensorValue = 30;
                   maxMotorPowerSensorValue = 200;
                   calculateMovements(true, true);
               } else {
                   moveToDropoffState = MoveToDropoffState.DONE;
               }
               break;
           case DONE:
               break;
       }
    }

    public enum DropoffState {
        // MOVE_TO_ATTACH,
        PIVOT_LIFT_ARM,
        PIVOTING_LIFT_ARM,
        OPEN_CLAW,
        MOVE_TO_BACKUP,
        RETRACT,
        DONE,
    }
    private void dropoffStateMachine() throws InterruptedException {
        switch (dropoffState) {
            /* case MOVE_TO_ATTACH:
                switch (teamColor) {
                    case BLUE:
                        // currentDestination = POSE_DROPOFF_ATTACH_BLUE;
                        break;
                    case RED:
                        // currentDestination = POSE_DROPOFF_ATTACH_RED;
                        break;
                }
                if (!atDestination()) {
                    sensorForDriveControl = ComponentEnum.FRONT_TOF_SENSOR;
                    zeroMotorPowerSensorValue = 30;
                    maxMotorPowerSensorValue = 200;
                    calculateMovements(true, false);
                } else {
                    stopMoving(true, true);
                    resetMotorPowerSensorData();
                    dropoffState = DropoffState.PIVOT_LIFT_ARM;
                }
                break; */
            case PIVOT_LIFT_ARM:
                rotationLiftState = RotationLiftState.DROPOFF;
                dropoffState = DropoffState.PIVOTING_LIFT_ARM;
                dropoffPivotTimer.reset();
                break;
            case PIVOTING_LIFT_ARM:
                if (dropoffPivotTimer.seconds() > DROPOFF_PIVOT_DELAY) {
                    dropoffState = DropoffState.OPEN_CLAW;
                }
                break;
            case OPEN_CLAW:
                clawState = ClawState.OPEN;
                dropoffState = DropoffState.RETRACT;
                break;
            case RETRACT:
                rotationLiftState = RotationLiftState.MOVE_TO_PICKUP;
                slideState = SLIDE_STATE.AUTO_PICKUP;
                clawPivotState = ClawPivotState.MOVE_TO_PICKUP;
                dropoffState = DropoffState.MOVE_TO_BACKUP;
                break;
            case MOVE_TO_BACKUP:
                resetMotorPowerSensorData();
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.DROP_OFF_BACKUP);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.DROP_OFF_BACKUP);
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, true);
                } else {
                    dropoffState = DropoffState.DONE;
                }
                break;
            case DONE:
                break;
        }
    }
}
