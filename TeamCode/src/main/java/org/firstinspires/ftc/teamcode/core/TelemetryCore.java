package org.firstinspires.ftc.teamcode.core;


import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.List;

public class TelemetryCore extends VisionCore {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    public void workers(boolean enableController) throws InterruptedException {
        super.workers(enableController);
        outputTelemetry(enableController);
    }

    public void outputTelemetry(boolean enableController) throws InterruptedException {
        telemetry.addData("Mantis is ", angryMantisTouchSensor.isPressed() ? "*** TOUCHING ***" : "not touching.");
        telemetry.addData("Mantis distance (mm)", angryMantisTofDistance - 20);
        telemetry.addData("Mantis Safe for Pixels?", mantisMode == MANTIS_MODE.PIXEL_PICKUP && gripState == GripState.OPEN ? "Safe": "*** DANGER: NOT SAFE! ***");
        telemetry.addData("Grip is ", gripState);

        telemetry.addData("Slide distance (mm) ", slideTofDistance);
        telemetry.addData("Angry Mantis State", mantisMode);
        telemetry.addData("Slide state is ", slideState);
        telemetry.addData("drone servo position", droneServo.getPosition());
        telemetry.addData("drone state", droneState);

        if (tfod != null) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# TFOD Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }   // end for() loop
        }

        if ( aprilTag != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");
        }
        telemetry.update();
    }
}
