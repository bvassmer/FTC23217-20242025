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

    public void workers() throws InterruptedException {
        super.workers();
        outputTelemetry();
    }

    public void outputTelemetry() throws InterruptedException {
        telemetry.addData("Lift Pivot Servo Position", liftPivotServoPosition);
        telemetry.addData("Lift Pivot Servo Position (Rev)", liftPivotServoReversePosition);
        telemetry.addData("clawPivotServoPosition Position", clawPivotServoPosition);
        telemetry.addData("clawServoPosition Position", clawServoPosition);
        telemetry.addData("clawServoState", clawPivotState);
        telemetry.addData("webcamServoPosition", webcamServoPosition);
        telemetry.addData("Lift Pivot State", rotationLiftState);
        telemetry.addData("Auto mode", this.autonomousMode);
        telemetry.addData("Enable Controller", this.enableController);
        if (autonomousMode) {
            telemetry.addData("searchState", searchState);
            telemetry.addData("searchTimer", searchTimer);
        }

        telemetry.update();
    }
}
