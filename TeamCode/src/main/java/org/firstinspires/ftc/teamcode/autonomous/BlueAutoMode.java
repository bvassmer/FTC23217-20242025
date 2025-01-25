package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.core.AutoDriveCore;

@Autonomous(group = "autonomousModes")
public class BlueAutoMode extends AutoDriveCore {
    public void runOpMode() throws InterruptedException {
        Log.d("FTC-23217-BlueAutoMode", "BlueAutoMode Start.");
        super.runOpMode(true, Enum.TeamColor.BLUE, AutoDriveState.DRIVING_HANGING_PARTS_START);
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            super.workers(false, false);
        };
    }
}
