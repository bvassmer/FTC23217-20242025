package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.core.AutoDriveCore;

@Autonomous(group = "autonomousModes")
public class BlueSpecimenAutoMode extends AutoDriveCore {
    public void runOpMode() throws InterruptedException {
        this.courseSide = CourseSide.RIGHT;
        this.autonomousMode = true;
        this.startAngle = 180.0;
        this.enableController = false;
        this.teamColor = Enum.TeamColor.BLUE;
        this.movementLockOn = false;
        parkOnly = false;
        this.MAP_DEBUG.put(DebugEnum.DRIVE_MOTORS, true);
        super.runOpMode(AutoDriveState.DRIVING_HANGING_PARTS_START);
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            super.workers();
        };
    }
}
