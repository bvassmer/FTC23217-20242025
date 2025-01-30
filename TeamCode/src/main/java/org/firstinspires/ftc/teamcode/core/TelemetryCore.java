package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.LinearVelocity;

public class TelemetryCore extends ControllerCore {
    private boolean autonomousMode = false;
    private Enum.TeamColor teamColor = Enum.TeamColor.BLUE;
    @Override
    public void runOpMode(boolean autonomousMode, Enum.TeamColor teamColor) throws InterruptedException {
        Log.d("FTC-23217-TelemetryCore", "TelemetryCore Start.");
        super.runOpMode(autonomousMode, teamColor);
        this.teamColor = teamColor;
        this.autonomousMode = autonomousMode;
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
