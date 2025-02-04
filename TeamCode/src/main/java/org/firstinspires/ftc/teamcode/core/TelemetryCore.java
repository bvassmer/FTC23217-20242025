package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;

public class TelemetryCore extends ControllerCore {
    @Override
    public void runOpMode() throws InterruptedException {
        Log.d("FTC-23217-TelemetryCore", "TelemetryCore Start.");
        super.runOpMode();
    }

    public void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        outputTelemetry(enableController);
    }

    public void outputTelemetry(boolean enableController) throws InterruptedException {
        telemetry.addData("Auto mode", autonomousMode);
        telemetry.addData("Enable Controller", enableController);
        telemetry.addData("liftPivotServo Position", liftPivotServoPosition);
        telemetry.addData("liftPivotServoReverse Position", liftPivotServoReversePosition);
        telemetry.addData("clawPivotServoPosition Position", clawPivotServoPosition);
        telemetry.addData("clawServoPosition Position", clawServoPosition);
        telemetry.addData("clawServoState", clawPivotState);
        telemetry.addData("searchState", searchState);
        telemetry.addData("searchTimer", searchTimer);
        telemetry.addData("webcamServoPosition", webcamServoPosition);

        telemetry.update();
    }
}
