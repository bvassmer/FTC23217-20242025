package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.core.VisionCore;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AutonomousCore extends VisionCore {
    Pose2d currentPose;
    public Enum.Color color;
    final double SKIP_TOO_LARGE_TIMER = 10.0;
    protected ElapsedTime liftWaitTimer = new ElapsedTime();
    protected ElapsedTime skipTooLargeTimer = new ElapsedTime();
    protected ElapsedTime sensorUpdateTimer = new ElapsedTime();
    public Enum.AutonomousState autonomousState = Enum.AutonomousState.LOWER_CAR_WASH;
    public Enum.Strike strikeFound = Enum.Strike.NOT_FOUND;
    public boolean skipTooLarge = true;

    public void runOpMode(Enum.Color iColor) throws InterruptedException {
        super.runOpMode();
        color = iColor;
    }

    public void workers(SampleMecanumDrive drive) throws InterruptedException {
        super.workers(false);
        AutonomousStateMachine(drive);
    }

    private void changeToAprilTagVisionProcessor() throws InterruptedException {
        initAprilTag();
    }

    private void checkForPixel(SampleMecanumDrive drive) throws InterruptedException {
        // Check for Pixels in view
        if (tfod != null) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if (currentRecognitions.size() > 0) {
                telemetry.addData("# Objects Detected", currentRecognitions.size());

                // Step through the list of recognitions and display info for each one.
                for (Recognition recognition : currentRecognitions) {
                    double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    double width = recognition.getWidth();
                    double height = recognition.getHeight();


                    telemetry.addData(""," ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position", "%.0f / %.0f", x, y);
                    telemetry.addData("- Size", "%.0f x %.0f", width, height);

                    if (skipTooLarge && (width > 275 || height > 275)) {
                        telemetry.addData("Skipped", "Too Large");
                    } else if (strikeFound != Enum.Strike.NOT_FOUND) {
                    } else {
                        strikeFound = findStrike(x, y, width, height);
                        setStrikeMovement();
                        telemetry.addData("Found", strikeFound);
                    }
                }   // end for() loop
            }
        }
    }

    private Enum.Strike findStrike(double x, double y, double width, double height) {
        if (x < 187) {
            return Enum.Strike.LEFT;
        }
        if (x < 450) {
            return Enum.Strike.CENTER;
        }
        return Enum.Strike.RIGHT;
    }

    private void setStrikeMovement() {
        switch (strikeFound) {
            case LEFT:
                autonomousState = Enum.AutonomousState.MOVE_TO_LEFT_STRIKE;
                break;
            case CENTER:
                autonomousState = Enum.AutonomousState.MOVE_TO_CENTER_STRIKE;
                break;
            case RIGHT:
                autonomousState = Enum.AutonomousState.MOVE_TO_RIGHT_STRIKE;
                break;
        }
    }

    private void goForwardSmall(SampleMecanumDrive drive) throws InterruptedException {
        Trajectory trajectory1 = drive.trajectoryBuilder(currentPose)
                .forward(5)
                .build();
        drive.followTrajectory(trajectory1);
        currentPose = trajectory1.end();
    }
    private void goBackwardSmall(SampleMecanumDrive drive) throws InterruptedException {
        Trajectory trajectory1 = drive.trajectoryBuilder(currentPose)
                .back(4)
                .build();
        drive.followTrajectory(trajectory1);
        currentPose = trajectory1.end();
    }
    private void rotateRobot(SampleMecanumDrive drive, double rotationAngle) throws InterruptedException {
        drive.turn(Math.toRadians(rotationAngle * 1.3));
    }

    private void moveRobotLeft(SampleMecanumDrive drive, double distance) throws InterruptedException {
        Trajectory trajectory1 = drive.trajectoryBuilder(currentPose)
                .strafeLeft(distance)
                .build();
        drive.followTrajectory(trajectory1);
        currentPose = trajectory1.end();

    }
    private void moveRobotRight(SampleMecanumDrive drive, double distance) throws InterruptedException {
        Trajectory trajectory1 = drive.trajectoryBuilder(currentPose)
                .strafeRight(distance)
                .build();
        drive.followTrajectory(trajectory1);
        currentPose = trajectory1.end();

    }

    private int getDesiredAprilTagId() throws InterruptedException {
        switch (color) {
            case RED:
                switch (strikeFound) {
                    case LEFT:
                        return 4;
                    case CENTER:
                        return 5;
                    case RIGHT:
                        return 6;
                }
                break;
            case BLUE:
                switch (strikeFound) {
                    case LEFT:
                        return 1;
                    case CENTER:
                        return 2;
                    case RIGHT:
                        return 3;
                }
                break;
        }
        return 0;
    }

    private AprilTagDetection getBestDetection(List<AprilTagDetection> currentDetections) throws InterruptedException {
        int desiredAprilTagId = getDesiredAprilTagId();
        double closestXDistance = 1000000;
        AprilTagDetection bestDetection = null;
        for (int i = 0; i < currentDetections.size(); i++) {
            AprilTagDetection detection = currentDetections.get(i);
            if (detection.id == desiredAprilTagId) {
                bestDetection = detection;
                break;
            }
            if (detection.ftcPose.x < closestXDistance) {
                bestDetection = detection;
                closestXDistance = detection.ftcPose.x;
            }
        }
        return bestDetection;
    }

    private void moveBasedOnXValue(SampleMecanumDrive drive, double x) throws InterruptedException {
        if (x > 0) {
            moveRobotLeft(drive, Math.abs(x * 1.2));
        } else if (x < 0) {
            moveRobotRight(drive, Math.abs(x * 1.2));
        }
    }


    private void findAprilTagAndAdjust(SampleMecanumDrive drive, List<AprilTagDetection> currentDetections) throws InterruptedException {
        if (currentDetections.size() > 0) {
            int desiredAprilTagId = getDesiredAprilTagId();
            AprilTagDetection bestDetection = getBestDetection(currentDetections);
            if (bestDetection.id == desiredAprilTagId) {
                if (bestDetection.ftcPose.x > -2 && bestDetection.ftcPose.x < 2 && bestDetection.ftcPose.roll < 2 && bestDetection.ftcPose.roll > -2) {
                    // we are centered and angled in. move forward and drop dat pixel.
                    sensorUpdateTimer.reset();
                    autonomousState = Enum.AutonomousState.FIND_BACKDROP;
                } else if (bestDetection.ftcPose.x > -2 && bestDetection.ftcPose.x < 2 ) {
                    rotateRobot(drive, -bestDetection.ftcPose.roll);
                } else if (bestDetection.ftcPose.roll < 2 && bestDetection.ftcPose.roll > -2) {
                    moveBasedOnXValue(drive, -bestDetection.ftcPose.x);
                } else {
                    // we found our tag, rotate and center on it.
                    rotateRobot(drive, -bestDetection.ftcPose.roll);
                    moveBasedOnXValue(drive, -bestDetection.ftcPose.x);
                }
            } else if (bestDetection.id > desiredAprilTagId) {
                // we are to the right of our desired april tag, rotate to be 0 deg and shift left
                rotateRobot(drive, -bestDetection.ftcPose.roll);
                moveRobotLeft(drive, 5);

            } else if (bestDetection.id < desiredAprilTagId) {
                // we are to the left of our desired april tag, rotate to be 0 deg and shift right
                rotateRobot(drive, -bestDetection.ftcPose.roll);
                moveRobotRight(drive, 5);
            }
        } else if (angryMantisTofDistance < 120) {
            // too close?
            autonomousState = Enum.AutonomousState.APRIL_TAG_SHIFT_BACKWARD;
        } else if (angryMantisTofDistance > 250) {
            // to far?
            autonomousState = Enum.AutonomousState.APRIL_TAG_SHIFT_FORWARD;
        }
    }

    private void findBackdrop(SampleMecanumDrive drive) throws InterruptedException {
        if (angryMantisTofDistance > 70) {
            goForwardSmall(drive);
        } else {
            autonomousState = Enum.AutonomousState.POWER_FORWARD;
        }
    }

    private void AutonomousStateMachine(SampleMecanumDrive drive) throws InterruptedException {
        switch (autonomousState) {
            case LOWER_CAR_WASH:
                angryMantisCloseGrip();
                autoLowerCarWash();
                autonomousState = Enum.AutonomousState.WAIT_FOR_LOWERED_CAR_WASH;
                break;
            case WAIT_FOR_LOWERED_CAR_WASH:
                if (carWashState == CarWashState.WAIT) {
                    autonomousState = Enum.AutonomousState.RAISE_LIFT;
                }
                break;
            case RAISE_LIFT:
                liftState = LiftState.AUTO_RAISE;
                autonomousState = Enum.AutonomousState.RAISING_LIFT;
                break;
            case RAISING_LIFT:
                if (liftState == LiftState.HOLDING) {
                    autonomousState = Enum.AutonomousState.RAISE_HOOK;
                }
                break;
            case RAISE_HOOK:
                raiseHook();
                skipTooLargeTimer.reset();
                autonomousState = Enum.AutonomousState.FIND_PIXEL;
                break;
            case LOWER_LIFT:
                liftState = LiftState.AUTO_LOWER;
                autonomousState = Enum.AutonomousState.LOWERING_LIFT;
                break;
            case LOWERING_LIFT:
                if (liftState == LiftState.WAITING) {
                    autonomousState = Enum.AutonomousState.MOVE_TO_BACKOFF;
                }
                break;
            case LOWER_HOOK:
                lowerHook();
                autonomousState = Enum.AutonomousState.LOWER_LIFT;
                break;
            case RESET_HOOK:
                lowerHook();
                raiseHook();
                skipTooLargeTimer.reset();
                autonomousState = Enum.AutonomousState.FIND_PIXEL;
            case MOVING:
                break;
            case FIND_PIXEL:
                if (!drive.isBusy()) {
                    if (skipTooLargeTimer.seconds() > SKIP_TOO_LARGE_TIMER) {
                        skipTooLarge = false;
                        autonomousState = Enum.AutonomousState.RESET_HOOK;
                    } else {
                        checkForPixel(drive);
                    }
                }
                break;
            case EJECT_PIXEL:
                if (!drive.isBusy()) {
                    changeToAprilTagVisionProcessor();
                    carWashState = CarWashState.EJECT_PIXEL;
                    autonomousState = Enum.AutonomousState.WAIT_FOR_EJECTED_PIXEL;
                }
                break;
            case WAIT_FOR_EJECTED_PIXEL:
                if (carWashState == CarWashState.WAIT) {
                    autonomousState = Enum.AutonomousState.LOWER_LIFT;
                }
                break;
            case LIFT_EXTENSION_ARM:
                liftWaitTimer.reset();
                autonomousState = Enum.AutonomousState.LIFTING_EXTENSION_ARM;
                break;
            case LIFTING_EXTENSION_ARM:
                slideState = SLIDE_STATE.AUTO_UP_MOVING;
                autonomousState = Enum.AutonomousState.MOVE_TO_BACKDROP;
                break;
            case FIND_APRIL_TAG:
                if (!drive.isBusy() && slideState == SLIDE_STATE.HOLDING) {
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    findAprilTagAndAdjust(drive, currentDetections);
                }
                break;
            case APRIL_TAG_SHIFT_FORWARD:
                goForwardSmall(drive);
                autonomousState = Enum.AutonomousState.FIND_APRIL_TAG;
                break;
            case APRIL_TAG_SHIFT_BACKWARD:
                goBackwardSmall(drive);
                autonomousState = Enum.AutonomousState.FIND_APRIL_TAG;
                break;
            case FIND_BACKDROP:
                if (!drive.isBusy() && slideState == SLIDE_STATE.HOLDING) {
                    findBackdrop(drive);
                }
                break;
            case POWER_FORWARD:
                if(!drive.isBusy()) {
                    goForwardSmall(drive);
                   autonomousState = Enum.AutonomousState.DROP_PIXEL;
                }
                break;
            case DROP_PIXEL:
                dropPixelIfSafe();
                autonomousState = Enum.AutonomousState.DONE;
                break;
            case DONE:
                break;
            default:
        }
    }
}
