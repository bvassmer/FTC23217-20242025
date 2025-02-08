package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.core.AutoDriveCore;

@TeleOp
public class BasicTeleOp extends AutoDriveCore {
    public void runOpMode() throws InterruptedException {
        Log.d("FTC-23217", "BasicTeleOp Start.");
        this.autonomousMode = false;
        this.enableController = true;
        this.teamColor = Enum.TeamColor.BLUE;
        this.MAP_DEBUG.put(DebugEnum.DRIVE_MOTORS, true);

        super.runOpMode();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            super.workers();
        }
    }
}
