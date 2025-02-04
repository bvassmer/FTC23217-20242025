package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.core.AutoDriveCore;

@Autonomous(group = "autonomousModes")
public class BlueParkAutoMode extends AutoDriveCore {
    public void runOpMode() throws InterruptedException {
        this.autonomousMode = true;
        this.startAngle = 180.0;
        this.teamColor = Enum.TeamColor.BLUE;
        super.runOpMode(AutoDriveState.DRIVING_PARK_START);
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            Log.d("FTC-23217-BlueAutoMode", "while loop");
            super.workers(false, false);
        };
    }
}
