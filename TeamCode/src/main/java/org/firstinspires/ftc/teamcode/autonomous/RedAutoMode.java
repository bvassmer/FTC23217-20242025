package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.core.AutoDriveCore;

@Autonomous(group = "autonomousModes")
public class RedAutoMode extends AutoDriveCore {
    public void runOpMode() throws InterruptedException {
        this.autonomousMode = true;
        this.startAngle = 0.0;
        this.enableController = false;
        this.teamColor = Enum.TeamColor.RED;
        this.MAP_DEBUG.put(DebugEnum.DRIVE_MOTORS, true);
        super.runOpMode(AutoDriveState.DRIVING_HANGING_PARTS_START);
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            super.workers();
        };
    }
}
