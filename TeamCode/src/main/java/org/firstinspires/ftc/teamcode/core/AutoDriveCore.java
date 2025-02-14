package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.jvm.Gen;

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
    private double distanceBuffer = 3.3;
    private double DROPOFF_PIVOT_DELAY = 0.2;
    private double previousDerivative = 0.0;
    double kP = 0.016; // Reduce aggressive correction was 0.008
    double kI = 0.00003;
    double kD = 0.001; // Reduce oscillation was 0.003


    double previousError = 0;
    double integralSum = 0;
    double maxTurnSpeed = 1.0; // Prevent excessive rotation speed
    private final ElapsedTime dropoffPivotTimer = new ElapsedTime();
    public MoveToDropoffState moveToDropoffState = MoveToDropoffState.ROTATE_SLIDE_TO_DROPOFF;
    public DropoffState dropoffState = DropoffState.PIVOT_LIFT_ARM;
    public int currentTaskStep = 0;
    public AutoDriveState autoDriveState = AutoDriveState.WAITING;
    public StepState stepState = StepState.START;
    private ElapsedTime moveToDropoffTimer = new ElapsedTime();


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
        startTimers();
    }

    public void workers() throws InterruptedException {
        super.workers(desiredXPower, desiredYPower, desiredRxPower);
        if (this.autonomousMode) {
            autoDriveStateMachine();
            telemetryOutput();
        }
    }

    private void startTimers() throws InterruptedException {
        pickupClawTimer.startTime();
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

    /* // Function to break speed into x and y components from bearing
    public void calculateSpeedComponents(double slowDistance, double maxSpeed) throws InterruptedException {
        // Convert bearing from degrees to radians
        double radians = Math.toRadians(desiredMovementBearing);
        double robotHeading = odo.getHeading(); // Get robot's current heading - radians
        double MIN_SPEED = 0.17;

        // Adjust bearing relative to the robot's heading
        double adjustedRadians = radians - robotHeading;

        double x1 = odo.getPosition().getX(DistanceUnit.INCH);
        double y1 = odo.getPosition().getY(DistanceUnit.INCH);
        double x2 = currentDestination.getX(DistanceUnit.INCH);
        double y2 = currentDestination.getY(DistanceUnit.INCH);

        // Calculate the Euclidean distance to the destination
        double distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));

        // Determine speed based on distance
        double desiredSpeed;
        if (distance > slowDistance) {
            desiredSpeed = maxSpeed;  // Move at max speed when far away
        } else {
            // Smoothly reduce speed using an S-curve when within SLOW_DISTANCE
            double progress = distance / slowDistance;  // Normalize distance to [0,1]
            double smoothProgress = 0.5 * (1 - Math.cos(progress * Math.PI)); // Ease-out function
            desiredSpeed = MIN_SPEED + smoothProgress * (maxSpeed - MIN_SPEED);
        }

        // Calculate x and y components of speed
        double xSpeed = desiredSpeed * Math.sin(adjustedRadians);  // x component (horizontal speed)
        double ySpeed = desiredSpeed * Math.cos(adjustedRadians);  // y component (vertical speed)

        Log.d("FTC-23217-AutoDriveCore", "calculateSpeedComponents: desiredSpeed:" + desiredSpeed + " xSpeed:" + xSpeed + " ySpeed:" + ySpeed);

        // Return the components as an array: [x, y]
        desiredXPower = xSpeed;
        desiredYPower = ySpeed;
    }

     private void calculateAngularMovement(double maxSpeed) throws InterruptedException {
        double currentAngularBearing = Math.toDegrees(odo.getHeading());
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

        // Define turning parameters
        double MAX_TURN_SPEED = 0.5;   // Maximum turn speed
        double MIN_TURN_SPEED = 0.19;  // Minimum turn speed (to prevent jitter)
        double SLOW_ANGLE = 70;        // Start slowing down when within this angle

        // Normalize progress: 1 when far, 0 when close
        double progress = Math.min(1.0, Math.abs(delta) / SLOW_ANGLE);

        // Apply cosine easing for smooth deceleration
        double smoothProgress = 0.5 * (1 - Math.cos(progress * Math.PI));

        // Compute desired turn speed
        double rxPower = MIN_TURN_SPEED + smoothProgress * (MAX_TURN_SPEED - MIN_TURN_SPEED);

        // Apply direction
        rxPower *= Math.signum(delta);

        /* // Scale rotation by translational movement strength (prevents rotation from overpowering)
        double translationalSpeed = Math.sqrt(desiredXPower * desiredXPower + desiredYPower * desiredYPower);
        rxPower *= Math.min(1.0, translationalSpeed / maxSpeed); // Ensures rotation is not dominant

        // If the angle difference is very small, stop turning
        if (Math.abs(delta) < 0.5) {
            rxPower = 0.0;
        }
        desiredRxPower = rxPower; //

        // If the angle difference is very small, stop turning
        if (Math.abs(delta) < 0.5) {
            rxPower = 0.0;
        }
        desiredRxPower = rxPower;

        /* if (teamColor == Enum.TeamColor.BLUE) {
            desiredRxPower *= -1;
        }
        Log.d("FTC-23217-AutoDriveCore", "calculateAngularMovement: delta:" + delta + " desiredRxPower:" + desiredRxPower);
    } */

    /* private void calculateAngularMovement() throws InterruptedException {
        double currentAngularBearing = Math.toDegrees(odo.getHeading());
        double currentDesiredAngularBearing = currentDestination.getHeading(AngleUnit.DEGREES);

        // Normalize angles
        currentAngularBearing = (currentAngularBearing + 360) % 360;
        currentDesiredAngularBearing = (currentDesiredAngularBearing + 360) % 360;

        // Calculate error (difference between desired and current heading)
        double error = currentDesiredAngularBearing - currentAngularBearing;

        // Normalize error to range [-180, 180]
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        // PID Calculations
        integralSum += error;  // Accumulate error over time (Integral)
        double derivative = error - previousError;  // Change in error (Derivative)
        previousError = error;  // Store current error for next cycle

        // Compute PID output (rxPower)
        double rxPower = (kP * error) + (kI * integralSum) + (kD * derivative);

        // Limit rxPower to prevent excessive rotation speed
        rxPower = Math.max(-maxTurnSpeed, Math.min(maxTurnSpeed, rxPower));

        // Apply rotation adjustment
        desiredRxPower = rxPower;

        Log.d("FTC-23217-AutoDriveCore-calculateAngularMovement", "PID Rotation: error=" + error + " rxPower=" + rxPower);
    } */

    private double getDistanceToTarget(double x1, double y1, double x2, double y2) throws InterruptedException {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    public void calculateSpeedComponents(double slowDistance, double maxSpeed) throws InterruptedException {
        double MIN_SPEED = 0.23;

        // Get current position from odometry
        Pose2D currentPose = odo.getPosition();
        double x1 = currentPose.getX(DistanceUnit.INCH);
        double y1 = currentPose.getY(DistanceUnit.INCH);
        double heading = currentPose.getHeading(AngleUnit.RADIANS); // Convert heading to radians

        // Get target position
        double x2 = currentDestination.getX(DistanceUnit.INCH);
        double y2 = currentDestination.getY(DistanceUnit.INCH);

        // Compute distance to target
        double distance = getDistanceToTarget(x1, y1, x2, y2);

        // Adjust speed based on distance
        double desiredSpeed;
        if (distance > slowDistance) {
            desiredSpeed = maxSpeed;
        } else {
            double progress = distance / slowDistance;
            double smoothProgress = 0.5 * (1 - Math.cos(progress * Math.PI));
            desiredSpeed = MIN_SPEED + smoothProgress * (maxSpeed - MIN_SPEED);
        }

        // Reduce speed if rotation is strong
        /* double rotationFactor = Math.max(0.3, 1.0 - Math.abs(desiredRxPower) * 2.0);
        desiredSpeed *= rotationFactor; */
        double rotationFactor = Math.max(0.9, 1.0 - Math.abs(desiredRxPower) * 1.3); // ✅ Less aggressive reduction
        desiredSpeed *= rotationFactor;

        // Compute absolute movement direction (from current to destination)
        double movementAngle = Math.atan2(y2 - y1, x2 - x1); // Field-centric angle

        // Convert to robot-centric using current heading
        double relativeAngle = movementAngle - heading; // Adjust for robot frame

        // ✅ FIX: Swap sin and cos to correctly map motion
        double xSpeed = desiredSpeed * Math.cos(relativeAngle); // Cos for x (strafe)
        double ySpeed = desiredSpeed * Math.sin(relativeAngle); // Sin for y (forward/backward)

        desiredXPower = xSpeed;
        desiredYPower = ySpeed;

        Log.d("FTC-23217-AutoDriveCore", "Speed: " + desiredSpeed + " xSpeed:" + xSpeed + " ySpeed:" + ySpeed + " rotationFactor:" + rotationFactor);
    }

    private void calculateAngularMovement() throws InterruptedException {
        // ✅ Use normalized heading directly
        double currentAngularBearing = odo.getPosition().getHeading(AngleUnit.DEGREES);
        double currentDesiredAngularBearing = currentDestination.getHeading(AngleUnit.DEGREES);

        // ✅ Compute distance to target
        Pose2D currentPose = odo.getPosition();
        double x1 = currentPose.getX(DistanceUnit.INCH);
        double y1 = currentPose.getY(DistanceUnit.INCH);
        double x2 = currentDestination.getX(DistanceUnit.INCH);
        double y2 = currentDestination.getY(DistanceUnit.INCH);
        double distanceToTarget = getDistanceToTarget(x1, y1, x2, y2);

        Log.d("FTC-23217-AutoDriveCore-calculateAngularMovement",
                "Normalized currentAngularBearing=" + currentAngularBearing +
                        " currentDesiredAngularBearing=" + currentDesiredAngularBearing);

        // ✅ Calculate error & ensure it's within [-180, 180]
        double error = currentDesiredAngularBearing - currentAngularBearing;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        Log.d("FTC-23217-AutoDriveCore-calculateAngularMovement",
                "Pre PID Error: " + error);

        /* double WINDUP_GUARD = 120.0;

        // ✅ Capped integral accumulation (instead of full reset)
        integralSum += error;
        integralSum = Math.max(-WINDUP_GUARD, Math.min(WINDUP_GUARD, integralSum)); */

        // ✅ Smoothed D-term to prevent jitter
        double rawDerivative = error - previousError;
        double derivative = (0.7 * rawDerivative) + (0.3 * previousDerivative);
        previousError = error;
        previousDerivative = derivative; // Store for next cycle

        // Compute PID output
        double rxPower = (kP * error) + (kI * integralSum) + (kD * derivative);

        // ✅ New: Adjust turn power based on **distance to target**
        /* double distanceFactor = Math.min(1.0, distanceToTarget / 2.0); // Adjust denominator to tune sensitivity

        // ✅ New: Keep rotation power high if far away, reduce if close
        double closeToTargetFactor = Math.max(1.0, Math.min(1.0, Math.abs(error) / WINDUP_GUARD)); */

        // rxPower *= closeToTargetFactor * distanceFactor; // Scale by both angle and distance

        // ✅ Clamp rxPower
        rxPower = Math.max(-maxTurnSpeed, Math.min(maxTurnSpeed, rxPower));

        /* if (Math.abs(rxPower) < 0.08) {
            rxPower = 0.0;
        } */

        desiredRxPower = rxPower;

        Log.d("FTC-23217-AutoDriveCore-calculateAngularMovement",
                "PID Rotation: error=" + error + " rxPower=" + rxPower /*+
                        " closeToTargetFactor=" + closeToTargetFactor +
                        " distanceFactor=" + distanceFactor*/);
    }




    private boolean atDestination() throws InterruptedException {
        boolean distanceGood = isDistanceGood();
        boolean angleGood = isAngleGood();
        Log.d("FTC-23217-AutoDriveCore-atDestination", "currentDestination" + currentDestination.toString());
        Log.d("FTC-23217-AutoDriveCore-atDestination", "distanceGood:" + distanceGood + " angleGood:" + angleGood);
        return distanceGood && angleGood;
    }

    private boolean isXYWithinBuffer(double x1, double y1, double x2, double y2, double buffer) throws InterruptedException {
        double distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
        return distance <= buffer;
    }

    private boolean isHeadingWithinBuffer(double headingToCheck, double targetHeading, double buffer) throws InterruptedException {
        // ✅ Normalize angles to range [-180, 180]
        headingToCheck = ((headingToCheck + 180) % 360 + 360) % 360 - 180;
        targetHeading = ((targetHeading + 180) % 360 + 360) % 360 - 180;

        // ✅ Compute the buffer range
        double lowerBound = targetHeading - buffer;
        double upperBound = targetHeading + buffer;

        // ✅ Normalize bounds to [-180, 180]
        lowerBound = ((lowerBound + 180) % 360 + 360) % 360 - 180;
        upperBound = ((upperBound + 180) % 360 + 360) % 360 - 180;
        Log.d("FTC-23217-AutoDriveCore-isHeadingWithinBuffer", "headingToCheck:" + headingToCheck + " targetHeading:" + targetHeading + " lowerBound:" + lowerBound + " upperBound:" + upperBound);

        // ✅ Handle wraparound cases correctly
        if (lowerBound <= upperBound) {
            return headingToCheck >= lowerBound && headingToCheck <= upperBound;
        } else {
            return headingToCheck >= lowerBound || headingToCheck <= upperBound;
        }
    }

    private boolean isAngleGood() throws InterruptedException {
        return isHeadingWithinBuffer(
                Math.toDegrees(odo.getHeading()),
                currentDestination.getHeading(AngleUnit.DEGREES),
                3);
    }
    private boolean isDistanceGood() throws InterruptedException {
        return isXYWithinBuffer(
                    odo.getPosition().getX(DistanceUnit.INCH),
                    odo.getPosition().getY(DistanceUnit.INCH),
                    currentDestination.getX(DistanceUnit.INCH),
                    currentDestination.getY(DistanceUnit.INCH),
                    this.distanceBuffer);
    }

    private void calculateMovements(boolean canDirection, boolean canRotate, boolean reversePower, double slowDistance, double maxSpeed) throws InterruptedException {
        if (!atDestination()) {
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: Moving");
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: currentX (odo)" + odo.getPosition().getX(DistanceUnit.INCH) + " currentY:" + odo.getPosition().getY(DistanceUnit.INCH));
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: destX" + currentDestination.getX(DistanceUnit.INCH) + " destY:" + currentDestination.getY(DistanceUnit.INCH));
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: currHeading (odo)" + Math.toDegrees(odo.getHeading()));
            Log.d("FTC-23217-AutoDriveCore", "calculateMovements: destHeading" + currentDestination.getHeading(AngleUnit.DEGREES));

            if (isAngleGood()) {
                Log.d("FTC-23217-AutoDriveCore", "calculateMovements: Angle is good. Stopping rotation.");
                calculateAngularMovement();
                stopMoving(false, true, reversePower);
            } else {
                Log.d("FTC-23217-AutoDriveCore", "calculateMovements: Rotating.");
                if (canRotate) {
                    // find power required to rotate to desired bearing
                    calculateAngularMovement();
                }
            }

            if (isDistanceGood()) {
                Log.d("FTC-23217-AutoDriveCore", "calculateMovements: Distance is good. Stopping movement.");
                calculateMovementBearing();
                calculateSpeedComponents(slowDistance, maxSpeed);
                stopMoving(true, false, reversePower);
            } else {
                Log.d("FTC-23217-AutoDriveCore", "calculateMovements: Moving.");
                if (canDirection) {
                    // find direction we need to move
                    calculateMovementBearing();
                    // find x, y power required to move desired direction
                    calculateSpeedComponents(slowDistance, maxSpeed);
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
        TESTING,
        DRIVING,
        STOP,
    }
    private void autoDriveStateMachine() throws InterruptedException {
        Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: autoDriveState:" + autoDriveState);
        Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: stepState:" + stepState);
        Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: dropoffState:" + dropoffState);
        Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: moveToDropoffState:" + moveToDropoffState);
        Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: pickupState:" + pickupState);
        Log.d("FTC-23217-AutoDriveCore", "autoDriveStateMachine: odoX:" + odo.getPosition().getX(DistanceUnit.INCH) + " odoY:" + odo.getPosition().getY(DistanceUnit.INCH) + " odoH:" + Math.toDegrees(odo.getHeading()));
        switch (autoDriveState) {
            case WAITING:
                stopMoving(true, true, false);
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
            case TESTING:
                stepState = StepState.TESTING;
                stepStateMachine();
                break;
            case STOP:
                stopMoving(true, true, false);
                break;
        }
    }
    private void stopMoving(boolean movement, boolean angle, boolean reversePower) throws InterruptedException {
        if (movement) {
            Log.d("FTC-23217-AutoDriveCore-stopMoving", "Stop Movement.");
            desiredXPower = 0.0;
            desiredYPower = 0.0;
            if (reversePower) {
                Pose2D vel = odo.getVelocity();
                if (vel.getX(DistanceUnit.INCH) > 0.1) {
                    Log.d("FTC-23217-AutoDriveCore-stopMoving", "Reversing X -Power.");
                    desiredXPower = -0.05;
                } else if (vel.getX(DistanceUnit.INCH) < -0.1) {
                    Log.d("FTC-23217-AutoDriveCore-stopMoving", "Reversing X +Power.");
                    desiredXPower = 0.05;
                }
                if (vel.getY(DistanceUnit.INCH) > 0.1) {
                    Log.d("FTC-23217-AutoDriveCore-stopMoving", "Reversing Y -Power.");
                    desiredYPower = -0.05;
                } else if (vel.getY(DistanceUnit.INCH) < -0.1) {
                    Log.d("FTC-23217-AutoDriveCore-stopMoving", "Reversing Y +Power.");
                    desiredYPower = 0.05;
                }
            }
            // stopMovement();
        }
        if (angle) {
            Log.d("FTC-23217-AutoDriveCore-stopMoving", "Stopping Angular Power.");
            desiredRxPower = 0.0;
        }
    }

    private void resetStateMachines() throws InterruptedException {
        moveToDropoffState = MoveToDropoffState.ROTATE_SLIDE_TO_DROPOFF;
        dropoffState = DropoffState.PIVOT_LIFT_ARM;
        pickupState = PickupState.MOVE_TO_WALL;
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
        zeroMotorPowerSensorValue = 200;
        maxMotorPowerSensorValue = 300;
        distanceBuffer = 3.3;
    }

    private static final StepState[] TASK_LIST = new StepState[]{
            StepState.START,
            StepState.MOVE_TO_DROP_OFF,
            StepState.DROP_OFF,
            StepState.PUSH_SPECIMENS,
            StepState.PICKUP,
            StepState.PICKUP_TO_DROP_OFF,
            StepState.MOVE_TO_DROP_OFF,
            StepState.DROP_OFF,
            StepState.DROP_OFF_TO_PICKUP,
            /*StepState.PICKUP,
            StepState.MOVE_TO_DROPOFF,
            StepState.DROPOFF,
            StepState.MOVE_TO_PICKUP,
            StepState.PICKUP,
            StepState.MOVE_TO_DROPOFF,
            StepState.DROPOFF,
            StepState.PARK, */
            StepState.STOP,
    };
    public enum StepState {
        START,
        NEXT_STEP,
        MOVE_TO_DROP_OFF,
        DROP_OFF_TO_PICKUP,
        DROP_OFF,
        PUSH_SPECIMENS,
        PICKUP,
        PICKUP_TO_DROP_OFF,
        PARK,
        STOP,
        TESTING,
        DONE
    }
    public enum GenericMoveStepState {
        WAYPOINT_ONE_SPECIMEN_ONE, // Move from drop off to corner turn to avoid middle structure.
        WAYPOINT_TWO_SPECIMEN_ONE, // Move forward to get behind ground specimens
        WAYPOINT_THREE_SPECIMEN_ONE, // Move to right to get lined up with specimen one.
        PUSH_SPECIMEN_ONE, // Push specimen one into parking zone
        WAYPOINT_ONE_SPECIMEN_TWO, // Move forward to get behind specimens (should be same location as MOVE_TO_WAYPOINT_THREE_SPECIMEN_ONE)
        WAYPOINT_TWO_SPECIMEN_TWO, // Move to right to get lined up with specimen two
        PUSH_SPECIMEN_TWO, // Push specimen two into parking zone
        WAYPOINT_ONE_SPECIMEN_THREE, // Move forward to get behind specimen three. (should be same location as MOVE_TO_WAYPOINT_TWO_SPECIMEN_TWO)
        WAYPOINT_TWO_SPECIMEN_THREE, // Move to right to get lined up with specimen three
        PUSH_SPECIMEN_THREE, // Push specimen three into parking zone
        WAYPOINT_PRE_PICKUP,
        PICKUP_TURN, // Reset out of the park zone and line up with pickup location. turn around. PICKUP should be after this.
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
            case MOVE_TO_DROP_OFF:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                MAP_COMPONENT.put(ComponentEnum.FRONT_ULTRASONIC_SENSOR, true);
                distanceBuffer = 6.0;
                if (moveToDropoffState == MoveToDropoffState.DONE) {
                    stepState = StepState.NEXT_STEP;
                } else {
                    moveToDropOffStateMachine();
                }
                break;
            case DROP_OFF:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                MAP_COMPONENT.put(ComponentEnum.FRONT_TOF_SENSOR, true);

                distanceBuffer = 3.3;
                if (dropoffState == DropoffState.DONE) {
                    stepState = StepState.NEXT_STEP;
                } else {
                    dropoffStateMachine();
                }
                break;
            case DROP_OFF_TO_PICKUP:
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.DROP_OFF_TO_PICKUP_TRANSITION);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.DROP_OFF_TO_PICKUP_TRANSITION);
                        break;
                }
                if (atDestination()) {
                    stepState = StepState.NEXT_STEP;
                } else {
                    calculateMovements(true, true, false, 20, 0.8);
                }
                break;
            case PUSH_SPECIMENS:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                distanceBuffer = 6.0;

                if (pushSpecimensStep == StepState.DONE) {
                    stepState = StepState.NEXT_STEP;
                } else {
                    pushSpecimensStateMachine();
                }
                break;
            case PICKUP:
                MAP_COMPONENT.put(ComponentEnum.SLIDE_TOF_SENSOR, true);
                MAP_COMPONENT.put(ComponentEnum.FRONT_TOF_SENSOR, true);
                MAP_COMPONENT.put(ComponentEnum.CLAW_TOUCH_SENSORS, true);
                distanceBuffer = 3.3;
                if (pickupState == PickupState.STOP) {
                    stepState = StepState.NEXT_STEP;
                } else {
                    pickupStateMachine();
                }
                break;
            case PICKUP_TO_DROP_OFF:
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.PICKUP_TO_DROP_OFF_TRANSITION);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.PICKUP_TO_DROP_OFF_TRANSITION);
                        break;
                }
                if (atDestination()) {
                    stepState = StepState.NEXT_STEP;
                } else {
                    calculateMovements(true, true, false, 20, 0.8);
                }
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
                if (atDestination()) {
                    autoDriveState = AutoDriveState.STOP;
                } else {
                    calculateMovements(true, true, false, 20, 0.8);
                }
                break;
            case TESTING:
                currentDestination = MAP_BLUE_POSE.get(POSE.TEST_TURN);
                if (atDestination()) {
                    autoDriveState = AutoDriveState.STOP;
                } else {
                    calculateMovements(true, true, false, 40, 1.0);
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
    private void moveToDropOffStateMachine() throws InterruptedException {
        double MOVE_TO_DROP_OFF_DELAY = 0.3;
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
               if (moveToDropoffTimer.seconds() > MOVE_TO_DROP_OFF_DELAY) {
                   moveToDropoffState = MoveToDropoffState.MOVE_TO_DROPOFF_ALIGNMENT;
               }
               break;
           case MOVE_TO_DROPOFF_ALIGNMENT:
               switch (teamColor) {
                   case BLUE:
                       currentDestination = MAP_BLUE_POSE.get(POSE.DROP_OFF_ALIGNMENT);
                       break;
                   case RED:
                       currentDestination = MAP_RED_POSE.get(POSE.DROP_OFF_ALIGNMENT);
                       break;
               }
               if (!atDestination()) {
                   calculateMovements(true, true, true, 40, 0.8);
               } else {
                   moveToDropoffState = MoveToDropoffState.MOVE_TO_DROPOFF;
               }
               break;
           case MOVE_TO_DROPOFF:
               sensorForDriveControl = ComponentEnum.FRONT_ULTRASONIC_SENSOR;
               zeroMotorPowerSensorValue = 210;
               maxMotorPowerSensorValue = 260;
               switch (teamColor) {
                   case BLUE:
                       currentDestination = MAP_BLUE_POSE.get(POSE.DROP_OFF);
                       break;
                   case RED:
                       currentDestination = MAP_RED_POSE.get(POSE.DROP_OFF);
                       break;
               }
               if (!atDestination()) {
                   calculateMovements(true, true, true, 60, 0.8);
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
                    calculateMovements(true, true, false, 20, 0.9);
                } else {
                    dropoffState = DropoffState.DONE;
                }
                break;
            case DONE:
                break;
        }
    }

    private static final GenericMoveStepState[] PUSH_SPECIMEN_TASK_LIST = new GenericMoveStepState[]{
        GenericMoveStepState.WAYPOINT_ONE_SPECIMEN_ONE, // Move from dropoff to corner turn to avoid middle structure.
        GenericMoveStepState.WAYPOINT_TWO_SPECIMEN_ONE, // Move forward to get behind ground specimens
        GenericMoveStepState.WAYPOINT_THREE_SPECIMEN_ONE, // Move to right to get lined up with specimen one.
        GenericMoveStepState.PUSH_SPECIMEN_ONE, // Push specimen one into parking zone
        GenericMoveStepState.WAYPOINT_ONE_SPECIMEN_TWO, // Move forward to get behind specimens (should be same location as MOVE_TO_WAYPOINT_THREE_SPECIMEN_ONE)
        GenericMoveStepState.WAYPOINT_TWO_SPECIMEN_TWO, // Move to right to get lined up with specimen two
        GenericMoveStepState.PUSH_SPECIMEN_TWO, // Push specimen two into parking zone
        GenericMoveStepState.WAYPOINT_ONE_SPECIMEN_THREE, // Move forward to get behind specimen three. (should be same location as MOVE_TO_WAYPOINT_TWO_SPECIMEN_TWO)
        GenericMoveStepState.WAYPOINT_TWO_SPECIMEN_THREE, // Move to right to get lined up with specimen three
        GenericMoveStepState.PUSH_SPECIMEN_THREE, // Push specimen three into parking zone
        GenericMoveStepState.WAYPOINT_PRE_PICKUP, // Push specimen three into parking zone
        GenericMoveStepState.PICKUP_TURN, // Reset out of the park zone and line up with pickup location. turn around. PICKUP should be after this.
        GenericMoveStepState.DONE,
    };
    private StepState pushSpecimensStep = StepState.PUSH_SPECIMENS;
    private int currentPushSpecimensStep = 0;
    private void pushSpecimensStateMachine() throws InterruptedException {
        switch (pushSpecimensStep) {
            case PUSH_SPECIMENS:
                if (PUSH_SPECIMEN_TASK_LIST[currentPushSpecimensStep] == GenericMoveStepState.DONE) {
                    pushSpecimensStep = StepState.DONE;
                } else {
                    switch (teamColor) {
                        case BLUE:
                            currentDestination = MAP_BLUE_STEP_POSE.get(PUSH_SPECIMEN_TASK_LIST[currentPushSpecimensStep]);
                            break;
                        case RED:
                            currentDestination = MAP_RED_STEP_POSE.get(PUSH_SPECIMEN_TASK_LIST[currentPushSpecimensStep]);
                            break;
                    }
                    if (!atDestination()) {
                        calculateMovements(true, true, false, 10, 1.0);
                    } else {
                        pushSpecimensStep = StepState.NEXT_STEP;
                    }
                }
                break;
            case NEXT_STEP:
                currentPushSpecimensStep += 1;
                pushSpecimensStep = StepState.PUSH_SPECIMENS;
                break;
            case DONE:
                break;
        }
    }

    public enum PickupState {
        MOVE_TO_WALL,
        CLOSE_CLAW,
        STOP;
    }
    public PickupState pickupState = PickupState.MOVE_TO_WALL;
    private ElapsedTime pickupClawTimer = new ElapsedTime();
    private final double PICKUP_CLAW_DELAY = 0.35;

    private void pickupStateMachine() throws InterruptedException {
        switch (pickupState) {
            case MOVE_TO_WALL:
                zeroMotorPowerSensorValue = 190;
                maxMotorPowerSensorValue = 290;
                switch (teamColor) {
                    case BLUE:
                        currentDestination = MAP_BLUE_POSE.get(POSE.PICKUP);
                        break;
                    case RED:
                        currentDestination = MAP_RED_POSE.get(POSE.PICKUP);
                        break;
                }
                if (isClawTouching /* || atDestination()*/) {
                    // claw is touching, move on!
                    pickupState = PickupState.CLOSE_CLAW;
                    pickupClawTimer.reset();
                    break;
                }
                calculateMovements(true, true, true, 60, 0.8);
                break;
            case CLOSE_CLAW:
                clawState = ClawState.CLOSE;
                if (pickupClawTimer.seconds() > PICKUP_CLAW_DELAY) {
                    pickupState = PickupState.STOP;
                }
                break;
            case STOP:
                break;
        }

    }
}
