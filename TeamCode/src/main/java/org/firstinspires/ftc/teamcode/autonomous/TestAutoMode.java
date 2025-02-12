package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.core.AutoDriveCore;

@Autonomous(group = "autonomousModes")
public class TestAutoMode extends AutoDriveCore {
    public void runOpMode() throws InterruptedException {
        this.autonomousMode = true;
        this.startAngle = 180.0;
        this.enableController = false;
        this.teamColor = Enum.TeamColor.BLUE;
        this.MAP_DEBUG.put(DebugEnum.DRIVE_MOTORS, true);
        this.MAP_DEBUG.put(DebugEnum.TESTING, true);
        stepState = StepState.TESTING;
        autoDriveState = AutoDriveState.TESTING;
        super.runOpMode(AutoDriveState.TESTING);
        waitForStart();
        resetRuntime();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            super.workers();
        };
    }
}
