package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.AngularVelocity;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;
import org.firstinspires.ftc.teamcode.library.PoseCircularBuffer;
import org.opencv.core.Mat;


public class AutoDriveCore extends MechamCore {
    private boolean debugMode = false;
    private final int POSE_QUEUE_SIZE = 8;
    private final Pose2D POSE_DROPOFF_RED = new Pose2D(DistanceUnit.INCH, 74, 42, AngleUnit.DEGREES, 0);
    private final Pose2D POSE_DROPOFF_ATTACH_RED = new Pose2D(DistanceUnit.INCH, 74, 43, AngleUnit.DEGREES, 0);
    private final Pose2D POSE_DROPOFF_BLUE = new Pose2D(DistanceUnit.INCH, 74, 45, AngleUnit.DEGREES, 0);
    private final Pose2D POSE_DROPOFF_ATTACH_BLUE = new Pose2D(DistanceUnit.INCH, 74, 47, AngleUnit.DEGREES, 0);
    private final Pose2D POSE_DROPOFF_BACKUP_POSE = new Pose2D(DistanceUnit.INCH, 74, 30, AngleUnit.DEGREES, 0);
    private final Pose2D POSE_DROPOFF_TURN_AROUND_POSE = new Pose2D(DistanceUnit.INCH, 80, 30, AngleUnit.DEGREES, 180);
    private final Pose2D POSE_PARK = new Pose2D(DistanceUnit.INCH, 125, 16, AngleUnit.DEGREES, 0);
    private final Pose2D POSE_PICKUP = new Pose2D(DistanceUnit.INCH, 120, 24, AngleUnit.DEGREES, 180);
    private final Pose2D POSE_MOVE_TO_PUSH_ONE = new Pose2D(DistanceUnit.INCH, 120, 24, AngleUnit.DEGREES, 0);
    private final Pose2D POSE_MOVE_TO_PUSH_TWO = new Pose2D(DistanceUnit.INCH, 120, 72, AngleUnit.DEGREES, 0);
    private final Pose2D POSE_MOVE_TO_PUSH_THREE = new Pose2D(DistanceUnit.INCH, 130, 72, AngleUnit.DEGREES, 180);
    private LinearVelocity currentLinearVelocity = new LinearVelocity(0.0, 0.0);
    private Pose2D currentDestination;
    private double desiredMovementBearing;
    private double desiredXPower = 0.0;
    private double desiredYPower = 0.0;
    private double desiredRxPower = 0.0;
    private double DROPOFF_PIVOT_DELAY = 1.0;
    private ElapsedTime dropoffPivotTimer = new ElapsedTime();
    public MoveToDropoffState moveToDropoffState = MoveToDropoffState.ROTATE_SLIDE_TO_DROPOFF;
    public DropoffState dropoffState = DropoffState.MOVE_TO_ATTACH;
    public int currentTaskStep = 0;
    public AutoDriveState autoDriveState = AutoDriveState.WAITING;
    public StepState stepState = StepState.START;


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

    public void workers(boolean enableController, boolean debugMode) throws InterruptedException {
        this.debugMode = debugMode;
        super.workers(enableController, desiredXPower, desiredYPower, desiredRxPower, debugMode);
        if (this.autonomousMode) {
            autoDriveStateMachine();
            telemetryOutput();
        }
    }

    private void telemetryOutput() throws InterruptedException {
        telemetry.addData("AutoDriveCode: debugMode", debugMode);
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
        double x1 = odoPoses.getLatestX();
        double y1 = odoPoses.getLatestY();
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

        desiredMovementBearing = movementBearing;
    }

    // Function to break speed into x and y components from bearing
    public void calculateSpeedComponents() throws InterruptedException {
        if (aprilTagDetected()) {
            // Convert bearing from degrees to radians
            double radians = Math.toRadians(desiredMovementBearing);
            double x1 = odoPoses.getLatestX();
            double y1 = odoPoses.getLatestY();
            double x2 = currentDestination.getX(DistanceUnit.INCH);
            double y2 = currentDestination.getY(DistanceUnit.INCH);


            boolean withIn5 = isXYWithinBuffer(x1, y1, x2, y2, 5);
            boolean withIn10 = isXYWithinBuffer(x1, y1, x2, y2, 10);
            boolean withIn20 = isXYWithinBuffer(x1, y1, x2, y2, 20);
            boolean withIn40 = isXYWithinBuffer(x1, y1, x2, y2, 40);
            boolean withIn100 = isXYWithinBuffer(x1, y1, x2, y2, 100);
            double desiredSpeed = 0.0;

            if (withIn5) {
                desiredSpeed = 0.2;
            } else if (withIn10) {
                desiredSpeed = 0.3;
            } else if (withIn20) {
                desiredSpeed = 0.4;
            } else if (withIn40) {
                desiredSpeed = 0.5;
            } else if (withIn100) {
                desiredSpeed = 0.6;
            } else {
                desiredSpeed = 0.0;
            }


            // Calculate x and y components of speed
            double xSpeed = desiredSpeed * Math.sin(radians);  // x component (horizontal speed)
            double ySpeed = desiredSpeed * Math.cos(radians);  // y component (vertical speed)

            // Return the components as an array: [x, y]
            desiredXPower = xSpeed;
            desiredYPower = ySpeed;
        } else {
            desiredXPower = 0.0;
            desiredYPower = 0.0;
        }
    }

    private void calculateAngularMovement() throws InterruptedException {
        if (aprilTagDetected()) {
            // TODO: This is no longer using an average of bearings. Need to make sure it still works.
            double currentAngularBearing = odoPoses.getLatestHeading();
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
            if (delta > 20) {
                desiredRxPower = 0.4;
            } else if (delta > 3) {
                desiredRxPower = 0.2;
            } else if (delta < -20) {
                desiredRxPower = -0.4;
            } else if (delta < -3) {
                desiredRxPower = -0.2;
            } else {
                desiredRxPower = 0.0;
            }
        } else {
            desiredRxPower = 0.0;
        }
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
                odoPoses.getLatestHeading(),
                currentDestination.getHeading(AngleUnit.DEGREES),
                6);
    }
    private boolean isDistanceGood() throws InterruptedException {
        return isXYWithinBuffer(
                    odoPoses.getLatestX(),
                    odoPoses.getLatestY(),
                    currentDestination.getX(DistanceUnit.INCH),
                    currentDestination.getY(DistanceUnit.INCH),
                   4);
    }

    private void calculateMovements(boolean canDirection, boolean canRotate) throws InterruptedException {
        if (!atDestination()) {
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: Moving");
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: currentX (odo)" + odoPoses.getLatestX() + " currentY:" + odoPoses.getLatestY());
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: destX" + currentDestination.getX(DistanceUnit.INCH) + " destY:" + currentDestination.getY(DistanceUnit.INCH));
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: currHeading (odo)" + odoPoses.getLatestHeading());
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
        updatePreviousPoses(true);
        if (currentAprilTag != null  && aprilTagDetected()) {
            Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: April Tag Detected: " + currentAprilTag.id);
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
        } else {
            Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: April Tag NOT Detected");
            stopMoving(true, true);
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
        dropoffState = DropoffState.MOVE_TO_ATTACH;
    }

    private static StepState[] TASK_LIST = new StepState[]{
            StepState.START,
            StepState.MOVE_TO_DROPOFF,
            StepState.DROPOFF,
            StepState.PARK,
            StepState.STOP
    };
    public enum StepState {
        START,
        NEXT_STEP,
        MOVE_TO_DROPOFF,
        DROPOFF,
        PARK,
        STOP,
    }
    private void stepStateMachine() throws InterruptedException {
        switch (stepState) {
            case START:
                stepState = TASK_LIST[currentTaskStep];
                break;
            case NEXT_STEP:
                currentTaskStep += 1;
                resetStateMachines();
                stepState = TASK_LIST[currentTaskStep];
                break;
            case MOVE_TO_DROPOFF:
                switch (teamColor) {
                    case BLUE:
                        currentDestination = POSE_DROPOFF_BLUE;
                        break;
                    case RED:
                        currentDestination = POSE_DROPOFF_RED;
                        break;
                }
                if (atDestination()) {
                    stepState = StepState.NEXT_STEP;
                } else {
                    moveToDropoffStateMachine();
                }
                break;
            case DROPOFF:
                if (dropoffState == DropoffState.DONE) {
                    stepState = StepState.NEXT_STEP;
                } else {
                    dropoffStateMachine();
                }
                break;
            case PARK:
                currentDestination = POSE_PARK;
                rotationLiftState = RotationLiftState.PARK;
                // updatePreviousPoses(true);
                if (atDestination()) {
                    autoDriveState = AutoDriveState.STOP;
                } else {
                    calculateMovements(true, false);
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
        MOVE_TO_DROPOFF,
    }
    private void moveToDropoffStateMachine() throws InterruptedException {
       switch (moveToDropoffState) {
           case ROTATE_SLIDE_TO_DROPOFF:
               rotationLiftState = RotationLiftState.MOVE_TO_DROPOFF;
               moveToDropoffState = MoveToDropoffState.MOVE_SLIDE_TO_DROPOFF;
               break;
           case MOVE_SLIDE_TO_DROPOFF:
               slideState = SLIDE_STATE.AUTO_DROPOFF;
               moveToDropoffState = MoveToDropoffState.MOVE_WRIST_TO_DROPOFF;
               break;
           case MOVE_WRIST_TO_DROPOFF:
               clawPivotState = ClawPivotState.MOVE_TO_DROPOFF;
               moveToDropoffState = MoveToDropoffState.MOVE_TO_DROPOFF;
               break;
           case MOVE_TO_DROPOFF:
               calculateMovements(true, false);
               break;
       }
    }

    public enum DropoffState {
        MOVE_TO_ATTACH,
        PIVOT_LIFT_ARM,
        PIVOTING_LIFT_ARM,
        OPEN_CLAW,
        MOVE_TO_BACKUP,
        RETRACT,
        DONE,
    }
    private void dropoffStateMachine() throws InterruptedException {
        switch (dropoffState) {
            case MOVE_TO_ATTACH:
                // updatePreviousPoses(true);
                switch (teamColor) {
                    case BLUE:
                        currentDestination = POSE_DROPOFF_ATTACH_BLUE;
                        break;
                    case RED:
                        currentDestination = POSE_DROPOFF_ATTACH_RED;
                        break;
                }
                if (!atDestination()) {
                    calculateMovements(true, false);
                } else {
                    stopMoving(true, true);
                    dropoffState = DropoffState.PIVOT_LIFT_ARM;
                }
                break;
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
                // updatePreviousPoses(true);
                currentDestination = POSE_DROPOFF_BACKUP_POSE;
                if (!atDestination()) {
                    calculateMovements(true, false);
                } else {
                    dropoffState = DropoffState.DONE;
                }
                break;
            case DONE:
                break;
        }
    }
}
