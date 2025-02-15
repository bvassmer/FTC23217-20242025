package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.core.AutoDriveCore;

@TeleOp
public class TestSensorsLeftBlue extends AutoDriveCore {
    public void runOpMode() throws InterruptedException {
        Log.d("FTC-23217", "TestSensors Start.");
        this.courseSide = CourseSide.LEFT;
        this.autonomousMode = false;
        this.teamColor = Enum.TeamColor.BLUE;
        this.enableController = false;
        this.MAP_DEBUG.put(DebugEnum.SENSORS, true);
        this.MAP_DEBUG.put(DebugEnum.DRIVE_MOTORS, false);
        this.MAP_DEBUG.put(DebugEnum.ODOMETERY, true);


        super.runOpMode();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            super.workers();
        }
    }
}
