package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.core.AutoDriveCore;

@Autonomous(group = "autonomousModes")
public class BlueAutoMode extends AutoDriveCore {
    public void runOpMode() throws InterruptedException {
        this.autonomousMode = true;
        this.startAngle = 180.0;
        this.teamColor = Enum.TeamColor.BLUE;
        super.runOpMode(AutoDriveState.DRIVING_HANGING_PARTS_START);
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            super.workers(false, false);
        };
    }
}
