package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.MechamCore;

@TeleOp
public class BasicTeleOp extends MechamCore {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            super.workers(true);
        }
    }
}
