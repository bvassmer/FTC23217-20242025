package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.core.AutoDriveCore;

@Autonomous(group = "autonomousModes")
public class RedParkAutoMode extends AutoDriveCore {
    public void runOpMode() throws InterruptedException {
        super.runOpMode(true, Enum.TeamColor.RED, AutoDriveState.DRIVING_PARK_START);
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            super.workers(false, false);
        };
    }
}
