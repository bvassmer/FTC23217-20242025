package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.MechamCore;

@TeleOp
public class TestVisionSwitch extends MechamCore {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        initVision(BLUE_RIGHT_MODEL);
        initTfod();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            super.workers(false);
            workers();
        }
    }

    public void workers() throws InterruptedException {
        if (gamepad1.a) {
            initAprilTag();
            // start
        }
    }
}
