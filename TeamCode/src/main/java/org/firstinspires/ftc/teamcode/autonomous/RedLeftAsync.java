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
public class RedLeftAsync extends AutonomousCore {
    public Enum.AutonomousState redLeftAutonomousState = Enum.AutonomousState.WAITING;
    public Enum.Color color = Enum.Color.RED;

    Pose2d startPose = new Pose2d(36, 60, Math.toRadians(270));
    private ArrayList<Enum.AutonomousState> steps = new ArrayList<Enum.AutonomousState>()
        {{
            add(Enum.AutonomousState.WAITING);
            add(Enum.AutonomousState.MOVE_TO_RIGHT_STRIKE);
            add(Enum.AutonomousState.MOVE_TO_CENTER_STRIKE);
            add(Enum.AutonomousState.MOVE_TO_LEFT_STRIKE);
            add(Enum.AutonomousState.MOVE_TO_BRIDGE_STOP);
            add(Enum.AutonomousState.MOVE_TO_BACKDROP_RIGHT);
            add(Enum.AutonomousState.MOVE_TO_BACKDROP);
        }};
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
            telemetry.addData("Red Right State", redLeftAutonomousState);
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
                .lineToLinearHeading(new Pose2d(39, 30, Math.toRadians(200)))
                .build();
        return trajectory;
    }

    protected Trajectory generateToCenterStrikeTrajectory(Pose2d start, SampleMecanumDrive drive) throws InterruptedException {
        Trajectory trajectory = drive.trajectoryBuilder(start)
                .lineTo(new Vector2d(41, 35))
                .build();
        return trajectory;
    }

    protected Trajectory generateToLeftStrikeTrajectory(Pose2d start, SampleMecanumDrive drive) throws InterruptedException {
        Trajectory trajectory = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(36,39, Math.toRadians(310)))
                .build();
        return trajectory;
    }

    private void stateMachine(SampleMecanumDrive drive) throws InterruptedException {
        switch (redLeftAutonomousState) {
            case WAITING:
                if (autonomousState == Enum.AutonomousState.MOVE_TO_BACKDROP
                        || autonomousState == Enum.AutonomousState.MOVE_TO_BACKOFF
                        || autonomousState == Enum.AutonomousState.LOWERING_LIFT
                        || autonomousState == Enum.AutonomousState.LOWER_LIFT) {

                    lowerHook();
                    liftState = LiftState.AUTO_LOWER;
                    autonomousState = Enum.AutonomousState.DONE;
                    redLeftAutonomousState = Enum.AutonomousState.DONE;
                } else if ((autonomousState == Enum.AutonomousState.MOVE_TO_LEFT_STRIKE
                        || autonomousState == Enum.AutonomousState.MOVE_TO_CENTER_STRIKE
                        || autonomousState == Enum.AutonomousState.MOVE_TO_RIGHT_STRIKE) && !drive.isBusy()) {
                    redLeftAutonomousState = Enum.AutonomousState.MOVING;
                }
                break;
            case MOVING:
                if (!drive.isBusy()) {
                    if (strikeFound != Enum.Strike.NOT_FOUND) {
                        switch (strikeFound) {
                            case LEFT:
                                Trajectory trajectory3 = generateToLeftStrikeTrajectory(currentPose, drive);
                                drive.followTrajectoryAsync(trajectory3);
                                currentPose = trajectory3.end();
                                autonomousState = Enum.AutonomousState.EJECT_PIXEL;
                                redLeftAutonomousState = Enum.AutonomousState.WAITING;
                                break;
                            case CENTER:
                                Trajectory trajectory2 = generateToCenterStrikeTrajectory(currentPose, drive);
                                drive.followTrajectoryAsync(trajectory2);
                                currentPose = trajectory2.end();
                                autonomousState = Enum.AutonomousState.EJECT_PIXEL;
                                redLeftAutonomousState = Enum.AutonomousState.WAITING;
                                break;
                            case RIGHT:
                                Trajectory trajectory1 = generateToRightStrikeTrajectory(currentPose, drive);
                                drive.followTrajectoryAsync(trajectory1);
                                currentPose = trajectory1.end();
                                autonomousState = Enum.AutonomousState.EJECT_PIXEL;
                                redLeftAutonomousState = Enum.AutonomousState.WAITING;
                                break;
                        }
                    } else {
                        redLeftAutonomousState = Enum.AutonomousState.WAITING;
                    }
                }
                break;
            case DONE:
                break;
            default:
        }
    }
}
