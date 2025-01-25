package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.core.AutoDriveCore;
import org.firstinspires.ftc.teamcode.core.MechamCore;

@TeleOp
public class BasicTeleOp extends AutoDriveCore {
    private boolean autonomousMode = false;
    private Enum.TeamColor teamColor = Enum.TeamColor.BLUE;
    public void runOpMode() throws InterruptedException {
        Log.d("FTC-23217", "BasicTeleOp Start.");

        super.runOpMode(autonomousMode, teamColor);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            super.workers(true, false);
        }
    }
}
