package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Config
@Autonomous(group = "autonomousModes")
public class RedRightAsync extends AutonomousCore {
    public Enum.AutonomousState redRightAutonomousState = Enum.AutonomousState.WAITING;
    public Enum.Color color = Enum.Color.RED;
    Pose2d startPose = new Pose2d(-12, 60, Math.toRadians(270));
    private Enum.AutonomousState currentStep = Enum.AutonomousState.MOVE_TO_RIGHT_STRIKE;

    public void runOpMode() throws InterruptedException {
        super.runOpMode(color);

        startSensorTimers();

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        currentPose = startPose;

        waitForStart();

        initVision(RED_LEFT_MODEL);
        initTfod();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            drive.update();
            workers(drive);
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Autonomous State", autonomousState);
            telemetry.addData("Red Right State", redRightAutonomousState);
            telemetry.addData("Red Right Step", currentStep);
            telemetry.addData("Car Wash State", carWashState);
            telemetry.addData("Strike Found", strikeFound);
            telemetry.addData("Mantis Distance", angryMantisTofDistance);
            telemetry.addData("Slide Distance", slideTofDistance);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        };
    }

    public void workers(SampleMecanumDrive drive) throws InterruptedException {
        super.workers(drive);
        stateMachine(drive);
    }

    // Pose2d startPose = new Pose2d(36, 60, Math.toRadians(90));
    protected Trajectory generateToRightStrikeTrajectory(Pose2d start, SampleMecanumDrive drive) throws InterruptedException {
        Trajectory trajectory = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(-27, 40, Math.toRadians(280)))
                .build();
        return trajectory;
    }

    protected Trajectory generateToCenterStrikeTrajectory(Pose2d start, SampleMecanumDrive drive) throws InterruptedException {
        Trajectory trajectory = drive.trajectoryBuilder(start)
                .lineTo(new Vector2d(-17, 35))
                .build();
        return trajectory;
    }

    protected Trajectory generateToLeftStrikeTrajectory(Pose2d start, SampleMecanumDrive drive) throws InterruptedException {
        ArrayList<Trajectory> trajectories = new ArrayList<>();
        Trajectory trajectory = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(-18,30, Math.toRadians(310)))
                .build();
        return trajectory;
    }

    protected Trajectory generateBackoffTrajectory(Pose2d start, SampleMecanumDrive drive, double angle) throws InterruptedException {
        Trajectory trajectory = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(-40, 30, Math.toRadians(angle)))
                .build();
        return trajectory;
    }


    protected Trajectory generateToBackdropTrajectory(Pose2d start, SampleMecanumDrive drive, double angle) throws InterruptedException {
        Trajectory trajectory = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(-48, 30, Math.toRadians(angle)))
                .build();
        return trajectory;
    }

    private void stateMachine(SampleMecanumDrive drive) throws InterruptedException {
        switch (redRightAutonomousState) {
            case WAITING:
                if ((autonomousState == Enum.AutonomousState.MOVE_TO_LEFT_STRIKE
                        || autonomousState == Enum.AutonomousState.MOVE_TO_CENTER_STRIKE
                        || autonomousState == Enum.AutonomousState.MOVE_TO_RIGHT_STRIKE
                        || autonomousState == Enum.AutonomousState.MOVE_TO_RIGHT_STRIKE2
                        || autonomousState == Enum.AutonomousState.MOVE_TO_BACKOFF
                        || autonomousState == Enum.AutonomousState.MOVE_TO_BACKDROP) && !drive.isBusy()) {
                    redRightAutonomousState = Enum.AutonomousState.MOVING;
                }
                break;
            case MOVING:
                if (!drive.isBusy()) {
                    if (strikeFound != Enum.Strike.NOT_FOUND) {
                        switch (autonomousState) {
                            case MOVE_TO_LEFT_STRIKE:
                                Trajectory trajectory3 = generateToLeftStrikeTrajectory(currentPose, drive);
                                drive.followTrajectoryAsync(trajectory3);
                                currentPose = trajectory3.end();
                                autonomousState = Enum.AutonomousState.EJECT_PIXEL;
                                redRightAutonomousState = Enum.AutonomousState.WAITING;
                                break;
                            case MOVE_TO_CENTER_STRIKE:
                                Trajectory trajectory2 = generateToCenterStrikeTrajectory(currentPose, drive);
                                drive.followTrajectoryAsync(trajectory2);
                                currentPose = trajectory2.end();
                                autonomousState = Enum.AutonomousState.EJECT_PIXEL;
                                redRightAutonomousState = Enum.AutonomousState.WAITING;
                                break;
                            case MOVE_TO_RIGHT_STRIKE:
                                Trajectory trajectory1 = generateToRightStrikeTrajectory(currentPose, drive);
                                drive.followTrajectoryAsync(trajectory1);
                                currentPose = trajectory1.end();
                                autonomousState = Enum.AutonomousState.EJECT_PIXEL;
                                redRightAutonomousState = Enum.AutonomousState.WAITING;
                                break;
                            case MOVE_TO_BACKOFF:
                                double angleBackoff;
                                switch (strikeFound) {
                                    case LEFT:
                                        angleBackoff = 170;
                                        break;
                                    default:
                                        angleBackoff = 175;
                                }
                                Trajectory trajectory4 = generateBackoffTrajectory(currentPose, drive, angleBackoff);
                                drive.followTrajectoryAsync(trajectory4);
                                currentPose = trajectory4.end();
                                autonomousState = Enum.AutonomousState.LIFT_EXTENSION_ARM;
                                redRightAutonomousState = Enum.AutonomousState.WAITING;
                                break;
                            case MOVE_TO_BACKDROP:
                                double angleBackdrop;
                                switch (strikeFound) {
                                    case LEFT:
                                        angleBackdrop = 170;
                                        break;
                                    default:
                                        angleBackdrop = 175;
                                }
                                Trajectory trajectory5 = generateToBackdropTrajectory(currentPose, drive, angleBackdrop);
                                drive.followTrajectoryAsync(trajectory5);
                                currentPose = trajectory5.end();
                                autonomousState = Enum.AutonomousState.FIND_APRIL_TAG;
                                redRightAutonomousState = Enum.AutonomousState.WAITING;
                                break;
                            default:
                                break;
                        }
                    }
                } else {
                    redRightAutonomousState = Enum.AutonomousState.WAITING;
                }
                break;
            case DONE:
                break;
            default:
        }
    }
}
