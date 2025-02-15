package org.firstinspires.ftc.teamcode.core;

import android.util.Log;
import android.util.Size;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.AprilTagLocation;
import org.firstinspires.ftc.teamcode.library.Coordinate;
import org.firstinspires.ftc.teamcode.library.DoubleCircularBuffer;
import org.firstinspires.ftc.teamcode.library.RobotBounds;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class CameraCore extends SensorCore {
    public enum SearchState {
        INITIALIZE,
        WAITING_FOR_START,
        CHECKING_FOR_APRILTAG,
        CENTER_WEBCAM_ON_APRILTAG,
        CENTERING_WEBCAM_ON_APRILTAG,
        SEARCHING_LEFT,
        SEARCHING_RIGHT,
        MOVE_LEFT,
        MOVE_RIGHT
    }
    public SearchState searchState = SearchState.WAITING_FOR_START;
    protected ElapsedTime searchTimer = new ElapsedTime();
    final static double SEARCH_WAIT_TIME = 0.2; // in seconds. equals 0.2s/cycle or 5 Hz.
    final static double CENTER_WAIT_TIME = 0.2; // in seconds. equals 0.2s/cycle or 5 Hz.
    final static double WEBCAM_SEARCH_STEP = 0.1;
    final static double BLUE_ANGLE_MIN = 75.0;
    final static double BLUE_ANGLE_MAX = 345.0;
    final static double RED_ANGLE_MIN = -105.0;
    final static double RED_ANGLE_MAX = 165.0;
    final static double BLUE_ANGLE_DIFF = BLUE_ANGLE_MAX - BLUE_ANGLE_MIN;
    final static double RED_ANGLE_DIFF = RED_ANGLE_MAX - RED_ANGLE_MIN;
    final static double WEBCAM_SERVO_MIN = 0.0;
    final static double WEBCAM_SERVO_MAX = 1.0;
    final static double SPECIMEN_DROPOFF_BEARING = 0.0;
    final static double SPECIMEN_PICKUP_BEARING = 180.0;
    private SearchState previousSearchState = SearchState.SEARCHING_RIGHT;

    // location of each april tag on the course grid.
    // X,Y origin is corner to the left of the starting position of robot
    // X,Y grid is 1 inch boxes
    // distances are in inches
    // Blue
    // Tag ID 13 - (0, 24)
    // Tag ID 12 - (72, 0)
    // Tag ID 11 - (144, 24)
    // Tag ID 14 - (0, 120)
    // Tag ID 15 - (72, 144)
    // Tag ID 16 - (144, 120)
    /* final static List<AprilTagLocation> blueAprilTagPoses = new ArrayList<>(Arrays.asList(
        new AprilTagLocation(13, 0, 24, 270), // 0
        new AprilTagLocation(12, 72, 0, 180), // 180
        new AprilTagLocation(11, 144, 24, 90),  // 90
        new AprilTagLocation(14, 0, 120, 270), // 270
        new AprilTagLocation(15, 72, 144, 0), // 270
        new AprilTagLocation(16, 144, 120, 90) // 90
    )); */
    /* final static List<AprilTagLocation> blueAprilTagPoses = new ArrayList<>(Arrays.asList(
            new AprilTagLocation(13, 72, 48, 270), // 0
            new AprilTagLocation(12, 0, 72, 180), // 180
            new AprilTagLocation(11, -72, 48, 90),  // 90
            new AprilTagLocation(14, 72, -48, 270), // 270
            new AprilTagLocation(15, 0, -72, 0), // 270
            new AprilTagLocation(16, -72, -48, 90) // 90
    )); */
    final static List<AprilTagLocation> blueAprilTagPoses = new ArrayList<>(Arrays.asList(
            new AprilTagLocation(16, -72, 48, 270), // 270
            new AprilTagLocation(15, 0, -72, 180), // 180
            new AprilTagLocation(14, 72, -48, 90), // 90
            new AprilTagLocation(11, -72, 48, 270), // 270
            new AprilTagLocation(12, 0, 72, 0), // 0
            new AprilTagLocation(13, 72, 48, 90) // 90
    ));

    // Red
    // Tag ID 16 - (0, 24)
    // Tag ID 15 - (72, 0)
    // Tag ID 14 - (144, 24)
    // Tag ID 11 - (0, 120)
    // Tag ID 12 - (72, 144)
    // Tag IG 13 - (144, 120)
    /* final static List<AprilTagLocation> redAprilTagPoses = new ArrayList<>(Arrays.asList(
        new AprilTagLocation(16, 0, 24, 270), // 270
        new AprilTagLocation(15, 72, 0, 180), // 180
        new AprilTagLocation(14, 144, 24, 90), // 90
        new AprilTagLocation(11, 0, 120, 270), // 270
        new AprilTagLocation(12, 72, 144, 0), // 0
        new AprilTagLocation(13, 144, 120, 90) // 90
    )); */
    final static List<AprilTagLocation> redAprilTagPoses = new ArrayList<>(Arrays.asList(
            new AprilTagLocation(16, -72, 48, 270), // 270
            new AprilTagLocation(15, 0, -72, 180), // 180
            new AprilTagLocation(14, 72, -48, 90), // 90
            new AprilTagLocation(11, -72, 48, 270), // 270
            new AprilTagLocation(12, 0, 72, 0), // 0
            new AprilTagLocation(13, 72, 48, 90) // 90
    ));

    public RobotBounds currentRobotBounds = new RobotBounds(
            new Coordinate(18, 18),
            new Coordinate(0, 18),
            new Coordinate(0, 0),
            new Coordinate(18, 0)
    );
    public DoubleCircularBuffer currentRangeCB = new DoubleCircularBuffer(30, true, false);
    public DoubleCircularBuffer currentATBearingCB = new DoubleCircularBuffer(30, true, false); // is bearing is false b/c this value is between -180 and 180 and we need to maintain the neg / positive
    public DoubleCircularBuffer currentATYawCB = new DoubleCircularBuffer(30, true, false);
    public double currentWebcamAngle = 0.0;
    private Coordinate currentCameraCenter = new Coordinate(10000, 10000);
    private double currentRobotBearing = 0.0;
    public AprilTagDetection currentAprilTag;


    public static final double ROBOT_SIDE_LENGTH = 18; // length of the side of the robot in inches
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final int INIT_APRIL_TAG_SEARCH_COUNT_MAX = 160000;
    private int searchCount = 0;
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    public AprilTagProcessor aprilTag;

    public VisionPortal visionPortal;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        searchCount = 0;
        if (autonomousMode) {
            // initVision();
            /*while (!aprilTagDetected()) {
                searchStateMachine();
                updateTelemetry(true);
                searchCount += 1;
                if (searchCount > INIT_APRIL_TAG_SEARCH_COUNT_MAX) {
                    break;
                }
            }
            while (
                cameraPoses.getStdDevX() > 0.4
                || cameraPoses.getStdDevY() > 0.4
                || cameraPoses.getStdDevHeading() > 0.01
            ) {
                if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                    searchStateMachine();
                }
                updateTelemetry(true);
                searchCount += 1;
                if (searchCount > INIT_APRIL_TAG_SEARCH_COUNT_MAX) {
                    break;
                }
            } */
            // centerWebcamOnAprilTag();
            // odo.resetPosAndIMU();
            // odo.setPosition(new Pose2D(DistanceUnit.INCH, cameraPoses.getAverageX(), cameraPoses.getAverageY(), AngleUnit.DEGREES, startAngle));
            // updateTelemetry(true);
            searchState = SearchState.WAITING_FOR_START;
        }
    }

    protected void workers() throws InterruptedException {
        super.workers();
        /* if (
                autonomousMode
                && Boolean.TRUE.equals(MAP_COMPONENT.get(ComponentEnum.CAMERA))
        ) {
            searchStateMachine();
            updateTelemetry(false);
        } */
    }

    public void initVision() throws InterruptedException {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(539.0239404,539.0239404, 316.450283269, 236.364794005)
                // ... these parameters are fx, fy, cx, cy.
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "ArduCam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(false);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessors(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        visionPortal.getCameraState();
    }

    private void updateTelemetry(boolean updateTelemetry) throws InterruptedException {
        // telemetry.addData("currentRobotBounds", currentRobotBounds);
        if (autonomousMode) {
            telemetry.addData("Search State", searchState);
            telemetry.addData("Vision State", visionPortal.getCameraState());
            if (!cameraPoses.isEmpty()) {
                // telemetry.addData("Camera Pose (Location Latest)", "x:" + cameraPoses.getLatestX() + " y:" + cameraPoses.getLatestY() + " heading:" + cameraPoses.getLatestHeading());
                telemetry.addData("Camera Pose (Location Avg)", "x:" + cameraPoses.getAverageX() + " y:" + cameraPoses.getAverageY() + " heading:" + cameraPoses.getAverageHeading());
                telemetry.addData("Camera Pose (StdDev)", "xStdDev:" + cameraPoses.getStdDevX() + " yStdDev:" + cameraPoses.getStdDevY() + " headingStdDev:" + cameraPoses.getStdDevHeading());
                telemetry.addData("Pose (ODO)", "X:" + odo.getPosition().getX(DistanceUnit.INCH) + " Y:" + odo.getPosition().getY(DistanceUnit.INCH) + " Heading(non-norm):" + Math.toDegrees(odo.getHeading()) + " Heading(norm):" +odo.getPosition().getHeading(AngleUnit.DEGREES));
            /* telemetry.addData("Camera Pose (Size)", cameraPoses.size());
            telemetry.addData("Camera Pose (X values)", Arrays.toString(cameraPoses.getXCoordinates()));
            telemetry.addData("Camera Pose (Y values)", Arrays.toString(cameraPoses.getYCoordinates()));
            telemetry.addData("Camera Pose (Headings)", Arrays.toString(cameraPoses.getHeadings())); */
            }
            // telemetry.addData("ATRangeCB Avg", currentRangeCB.getAverage());
            // telemetry.addData("ATBearingCB Avg", currentATBearingCB.getAverage());
            // telemetry.addData("ATYawCB Avg", currentATYawCB.getAverage());
            telemetry.addData("currentRobotBearing", currentRobotBearing);
            telemetry.addData("currentWebcamAngle", currentWebcamAngle);
            // telemetry.addData("currentWebcamServo", webcamServoPosition);
            telemetry.addData("Search Iteration Count", searchCount);
            telemetry.addData("Team", teamColor);
            if (aprilTag != null) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                telemetry.addData("# AprilTags Detected", currentDetections.size());
            }
            if (updateTelemetry) {
                telemetry.update();
            }
        }
    }

    private void updateCurrentWebcamAngle() throws InterruptedException {
        switch (teamColor) {
            case BLUE:
                currentWebcamAngle = (webcamServoPosition * BLUE_ANGLE_DIFF) + BLUE_ANGLE_MIN;
                break;
            case RED:
                currentWebcamAngle = (webcamServoPosition * RED_ANGLE_DIFF) + RED_ANGLE_MIN;
                break;
        }
    }

    private double getServoDiff(double bearingDegrees) throws InterruptedException {
        double servoDiffMin = -1.0;
        double servoDiffMax = 1.0;

        double angleDiffMin = 0.0;
        double angleDiffMax = 0.0;
        switch (teamColor) {
            case BLUE:
                angleDiffMin = -(BLUE_ANGLE_MAX - BLUE_ANGLE_MIN);
                angleDiffMax = BLUE_ANGLE_MAX - BLUE_ANGLE_MIN;
                break;
            case RED:
                angleDiffMin = -(RED_ANGLE_MAX - RED_ANGLE_MIN);
                angleDiffMax = RED_ANGLE_MAX - RED_ANGLE_MIN;
                break;
        }

        return ((bearingDegrees - angleDiffMin) * (servoDiffMax - servoDiffMin) / (angleDiffMax - angleDiffMin)) + servoDiffMin;
    }

    private void updateAprilTagBearing() throws InterruptedException {
        if (currentAprilTag != null) {
            Optional<AprilTagLocation> aprilTagLocationOptional = getAprilTagLocation(currentAprilTag);
            if (aprilTagLocationOptional.isPresent()) {
                double bearing = currentAprilTag.ftcPose.bearing;
                currentATBearingCB.addAndCalculate(bearing, 1);
            }
        }
    }

    private void updateAprilTagYaw() throws InterruptedException {
        if (currentAprilTag != null) {
            Optional<AprilTagLocation> aprilTagLocationOptional = getAprilTagLocation(currentAprilTag);
            if (aprilTagLocationOptional.isPresent()) {
                double yaw = currentAprilTag.ftcPose.yaw;
                currentATYawCB.addAndCalculate(yaw, 1);
            }
        }
    }

    private void updateCurrentRobotBearing() throws InterruptedException {
        if (currentAprilTag != null) {
            Optional<AprilTagLocation> aprilTagLocationOptional = getAprilTagLocation(currentAprilTag);
            if (aprilTagLocationOptional.isPresent()) {
                AprilTagLocation aprilTagLocation = aprilTagLocationOptional.get();
                double bearing = aprilTagLocation.angleCorrection - currentWebcamAngle - currentATYawCB.getAverage();
                // Normalize to [0, 360) range
                double normalizedBearing = (bearing + 360) % 360;
                currentRobotBearing = normalizedBearing;
                // robotState.bearingCB.addAndCalculate(normalizedBearing, 1);
            }
        }
    }

    private void updateCurrentRobotRange() throws InterruptedException {
        if (currentAprilTag != null) {
            double range = currentAprilTag.ftcPose.range;
            currentRangeCB.addAndCalculate(range, 0);
        }
    }

    private Optional<AprilTagLocation> getAprilTagLocation(AprilTagDetection aprilTag) throws InterruptedException {
        Optional<AprilTagLocation> aprilTagLocation;
        switch (teamColor) {
            case RED:
                aprilTagLocation = redAprilTagPoses.stream()
                        .filter(at -> at.id == aprilTag.id)
                        .findFirst();
                break;
            case BLUE:
                aprilTagLocation = blueAprilTagPoses.stream()
                        .filter(at -> at.id == aprilTag.id)
                        .findFirst();
                break;
            default:
                return Optional.empty();
        }
        return aprilTagLocation;
    }

    private void updateCurrentCameraCenter() throws InterruptedException {
        if (currentAprilTag != null) {
            Optional<AprilTagLocation> aprilTagLocationOptional = getAprilTagLocation(currentAprilTag);
            if (aprilTagLocationOptional.isPresent()) {
                AprilTagLocation aprilTagLocation = aprilTagLocationOptional.get();
                double xAprilTag = aprilTagLocation.x;
                double yAprilTag = aprilTagLocation.y;
                double yaw = currentATYawCB.getAverage();
                double bearing = currentATBearingCB.getAverage();
                double theta = yaw - bearing;
                if (teamColor == Enum.TeamColor.BLUE) {
                    theta += 180;
                }
                double thetaRadians = Math.toRadians(theta);

                double currentX = 0;
                double currentY = 0;

                int BLUE_LEFT_ANGLE = 90;
                int BLUE_FRONT_ANGLE = 180;
                int BLUE_RIGHT_ANGLE = 270;
                int BLUE_BACK_ANGLE = 360;
                int RED_LEFT_ANGLE = -90;
                int RED_FRONT_ANGLE = 0;
                int RED_RIGHT_ANGLE = 90;
                int RED_BACK_ANGLE = 180;
                switch (teamColor) {
                    case BLUE:
                        switch (currentAprilTag.id) {
                            case 11:
                            case 16:
                                currentX = xAprilTag - (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                currentY = yAprilTag - (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                break;
                            case 13:
                            case 14:
                                currentX = xAprilTag + (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                currentY = yAprilTag + (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                break;
                            case 15:
                                if (currentWebcamAngle >= BLUE_FRONT_ANGLE && currentWebcamAngle < BLUE_RIGHT_ANGLE) {
                                    currentX = xAprilTag + (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                    currentY = yAprilTag - (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                } else if (currentWebcamAngle >= BLUE_RIGHT_ANGLE && currentWebcamAngle <= BLUE_BACK_ANGLE) {
                                    currentX = xAprilTag - (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                    currentY = yAprilTag - (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                } else if (currentWebcamAngle < BLUE_FRONT_ANGLE && currentWebcamAngle > BLUE_LEFT_ANGLE) {
                                    currentX = xAprilTag + (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                    currentY = yAprilTag - (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                }
                                break;
                            case 12:
                                if (currentWebcamAngle >= BLUE_FRONT_ANGLE && currentWebcamAngle < BLUE_RIGHT_ANGLE) {
                                    currentX = xAprilTag - (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                    currentY = yAprilTag + (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                } else if (currentWebcamAngle >= BLUE_RIGHT_ANGLE && currentWebcamAngle <= BLUE_BACK_ANGLE) {
                                    currentX = xAprilTag + (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                    currentY = yAprilTag + (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                } else if (currentWebcamAngle < BLUE_FRONT_ANGLE && currentWebcamAngle > BLUE_LEFT_ANGLE) {
                                    currentX = xAprilTag - (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                    currentY = yAprilTag + (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                }
                                break;

                        }
                        break;
                    case RED:
                        switch (currentAprilTag.id) {
                            case 13:
                            case 14:
                                currentX = xAprilTag - (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                currentY = yAprilTag - (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                break;
                            case 11:
                            case 16:
                                currentX = xAprilTag + (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                currentY = yAprilTag + (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                break;
                            case 12:
                                if (currentWebcamAngle >= RED_FRONT_ANGLE && currentWebcamAngle < RED_RIGHT_ANGLE) {
                                    currentX = xAprilTag + (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                    currentY = yAprilTag - (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                } else if (currentWebcamAngle >= RED_RIGHT_ANGLE && currentWebcamAngle <= RED_BACK_ANGLE) {
                                    currentX = xAprilTag - (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                    currentY = yAprilTag - (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                } else if (currentWebcamAngle < RED_FRONT_ANGLE && currentWebcamAngle > RED_LEFT_ANGLE) {
                                    currentX = xAprilTag + (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                    currentY = yAprilTag - (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                }
                                break;
                            case 15:
                                if (currentWebcamAngle >= RED_FRONT_ANGLE && currentWebcamAngle < RED_RIGHT_ANGLE) {
                                    currentX = xAprilTag - (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                    currentY = yAprilTag + (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                } else if (currentWebcamAngle >= RED_RIGHT_ANGLE && currentWebcamAngle <= RED_BACK_ANGLE) {
                                    currentX = xAprilTag + (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                    currentY = yAprilTag + (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                } else if (currentWebcamAngle < RED_FRONT_ANGLE && currentWebcamAngle > RED_LEFT_ANGLE) {
                                    currentX = xAprilTag - (currentRangeCB.getAverage() * Math.sin(thetaRadians));
                                    currentY = yAprilTag + (currentRangeCB.getAverage() * Math.cos(thetaRadians));
                                }
                                break;

                        }
                        break;
                }
                Log.d("FTIC-23217-CameraCore", "updateCurrentCameraCenter: id:" + currentAprilTag.id + " team:" + teamColor + " xAT:" + xAprilTag + " yAT:" + yAprilTag + " theta:" + theta + " thetaRad: " + thetaRadians + " currRange:" + currentRangeCB.getAverage() + " curX:" + currentX + " curY:" + currentY);

                currentCameraCenter = new Coordinate(currentX, currentY);
            }
        }
    }

    /* private void updateCurrentRobotBounds() throws InterruptedException {
        // currentCameraCenter is the location of the camera.
        // need to account for the rest of the robot.
        // x0,y0 = top right (camera location) (known calculated)
        // x1,y1 = top left
        // x2,y2 = bottom left
        // x3,y3 = bottom right
        // R = robot side size (18 inches normal) (Known constant)
        // θ = angle diff from 0 degree north on field (Known calculated)

        double x1 = 0.0, x2 = 0.0, x3 = 0.0, y1 = 0.0, y2 = 0.0, y3 = 0.0;
        double hyp = Math.sqrt(Math.pow(ROBOT_SIDE_LENGTH, 2) + Math.pow(ROBOT_SIDE_LENGTH, 2));
        double currentBearing = robotState.bearingCB.getAverage();

        if (currentBearing >= 0 && currentBearing < 90) { // looking N to E (NE Quad)
            // x1 = x0 - R * cos θ
            x1 = robotState.cameraCenter.x - ROBOT_SIDE_LENGTH * Math.cos(Math.toRadians(currentBearing));
            // y1 = y0 + R * cos θ
            y1 = robotState.cameraCenter.y + ROBOT_SIDE_LENGTH * Math.sin(Math.toRadians(currentBearing));

            // x2 = x0 - sqr(R^2+R^2) * sin (θ + 45)
            x2 = robotState.cameraCenter.x - hyp * Math.sin(Math.toRadians(currentBearing + 45));
            // y2 = y0 - sqr(R^2+R^2) * cos (θ + 45)
            y2 = robotState.cameraCenter.y - hyp * Math.cos(Math.toRadians(currentBearing + 45));

            // x3 = x0 - R * sin θ
            x3 = robotState.cameraCenter.x - ROBOT_SIDE_LENGTH * Math.sin(Math.toRadians(currentBearing));
            // y3 = y0 - R * cos θ
            y3 = robotState.cameraCenter.y - ROBOT_SIDE_LENGTH * Math.cos(Math.toRadians(currentBearing));
        } else if (currentBearing >= 270 && currentBearing <= 360) { // looking W to N (NW Quad)
            // x1 = x0 - R * cos θ
            x1 = robotState.cameraCenter.x - ROBOT_SIDE_LENGTH * Math.cos(Math.toRadians(360 - currentBearing));
            // y1 = y0 - R * cos θ
            y1 = robotState.cameraCenter.y - ROBOT_SIDE_LENGTH * Math.sin(Math.toRadians(360 - currentBearing));

            // x2 = x0 - sqr(R^2+R^2) * sin (θ + 45)
            x2 = robotState.cameraCenter.x - hyp * Math.sin(Math.toRadians(currentBearing + 45));
            // y2 = y0 - sqr(R^2+R^2) * cos (θ + 45)
            y2 = robotState.cameraCenter.y - hyp * Math.cos(Math.toRadians(currentBearing + 45));

            // x3 = x0 + R * sin θ
            x3 = robotState.cameraCenter.x + ROBOT_SIDE_LENGTH * Math.sin(Math.toRadians(360 - currentBearing));
            // y3 = y0 - R * cos θ
            y3 = robotState.cameraCenter.y - ROBOT_SIDE_LENGTH * Math.cos(Math.toRadians(360 - currentBearing));
        } else if (currentBearing >= 180 && currentBearing < 270) { // looking S to W (SW Quad)
            // x1 = x0 - R * cos θ
            x1 = robotState.cameraCenter.x + ROBOT_SIDE_LENGTH * Math.sin(Math.toRadians(360 - currentBearing));
            // y1 = y0 - R * cos θ
            y1 = robotState.cameraCenter.y - ROBOT_SIDE_LENGTH * Math.cos(Math.toRadians(360 - currentBearing));

            // x2 = x0 - sqr(R^2+R^2) * sin (θ + 45)
            x2 = robotState.cameraCenter.x + hyp * Math.cos(Math.toRadians(currentBearing + 45));
            // y2 = y0 - sqr(R^2+R^2) * cos (θ + 45)
            y2 = robotState.cameraCenter.y - hyp * Math.sin(Math.toRadians(currentBearing + 45));

            // x3 = x0 + R * sin θ
            x3 = robotState.cameraCenter.x + ROBOT_SIDE_LENGTH * Math.cos(Math.toRadians(360 - currentBearing));
            // y3 = y0 - R * cos θ
            y3 = robotState.cameraCenter.y + ROBOT_SIDE_LENGTH * Math.sin(Math.toRadians(360 - currentBearing));
        } else if (currentBearing >= 90 && currentBearing < 180) { // looking E to S (SE Quad)
            // x1 = x0 - R * cos θ
            x1 = robotState.cameraCenter.x - ROBOT_SIDE_LENGTH * Math.sin(Math.toRadians(360 - currentBearing));
            // y1 = y0 - R * cos θ
            y1 = robotState.cameraCenter.y + ROBOT_SIDE_LENGTH * Math.cos(Math.toRadians(360 - currentBearing));

            // x2 = x0 - sqr(R^2+R^2) * sin (θ + 45)
            x2 = robotState.cameraCenter.x - hyp * Math.cos(Math.toRadians(currentBearing + 45));
            // y2 = y0 - sqr(R^2+R^2) * cos (θ + 45)
            y2 = robotState.cameraCenter.y + hyp * Math.sin(Math.toRadians(currentBearing + 45));

            // x3 = x0 + R * sin θ
            x3 = robotState.cameraCenter.x - ROBOT_SIDE_LENGTH * Math.cos(Math.toRadians(360 - currentBearing));
            // y3 = y0 - R * cos θ
            y3 = robotState.cameraCenter.y - ROBOT_SIDE_LENGTH * Math.sin(Math.toRadians(360 - currentBearing));
        }

        currentRobotBounds = new RobotBounds(
                new Coordinate(robotState.cameraCenter.x, robotState.cameraCenter.y),
                new Coordinate(x1, y1),
                new Coordinate(x2, y2),
                new Coordinate(x3, y3)
        );
    } */

    private void searchForAprilTag() throws InterruptedException {
        if (aprilTagDetected()) {
            searchState = SearchState.CHECKING_FOR_APRILTAG;
        } else {
            // TODO: Put better logic here on where to search next based on current location information.
            // TODO: Maybe create an algorithm that calculates closest april tag within view and then moves to it.
            searchState = previousSearchState;
        }
        // double currentBearing = bearingCB.getAverage();
        // go through specific search locations based on current x,y location
        /* switch (teamColor) {
            case RED:
                if (currentBearing < 90 && robotState.cameraCenter.x >= 72 && robotState.cameraCenter.y <= 72) {
                    // SE Quad, facing north to east (0 to 90)
                    // id 14 main, id 13 backup
                } else if (currentBearing < 90 && robotState.cameraCenter.x < 72 && robotState.cameraCenter.y <= 72) {
                    // SW Quad, facing north to east (0 to 90)
                    // id 15 main, id 14 backup
                } else if (currentBearing > 90 && currentBearing < 180 && robotState.cameraCenter.x >= 72 && robotState.cameraCenter.y <= 72) {
                    // SE Quad, facing east to south (90 to 180)
                    // id 15 main, id 16 backup
                } else if (currentBearing > 90 && currentBearing < 180 && robotState.cameraCenter.x < 72 && robotState.cameraCenter.y <= 72) {
                    // SW Quad, facing east to south (90 to 180)
                    // id 16 main, id 11 backup
                }
                break;
            case BLUE:
                break;
        } */
    }

    private void updateBestAprilTag() throws InterruptedException {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double minRange = 100000;
        currentAprilTag = null;

        List<AprilTagDetection> safeCurrentDetections = currentDetections.stream()
                .filter(at -> at.metadata != null)
                .collect(Collectors.toList());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : safeCurrentDetections) {
            if (detection.ftcPose.range < minRange) {
                currentAprilTag = detection;
                minRange = detection.ftcPose.range;
            }
        }
    }

    protected boolean aprilTagDetected() throws InterruptedException {
        if (aprilTag != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            return !currentDetections.isEmpty();
        }
        return false;
    }

    /* private double getYVel() throws InterruptedException {
        return currentLinearVelocity.speed * Math.cos(Math.toRadians(currentLinearVelocity.bearing));
    }
    private double getXVel() throws InterruptedException {
        return currentLinearVelocity.speed * Math.sin(Math.toRadians(currentLinearVelocity.bearing));
    } */

    private void centerWebcamOnAprilTag() throws InterruptedException {
        // TODO: Rewrite this code to better handle while the camera is moving.
        // TODO: Use the current velocity to adjust where we want to look. We may need to look ahead a little bit to handle delays in processing.
        if (!this.currentATBearingCB.isEmpty()) {
            double bearingCBNewest = this.currentATBearingCB.newestPeek();
            double servoDiff = getServoDiff(bearingCBNewest);

            double newServoPosition = webcamServoPosition + servoDiff;
            Log.d("FTC-23217-CameraCore", "centerWebcamOnAprilTag: bearingCBNewest:" + bearingCBNewest + " servoDiff:" + servoDiff + " webcamPosition:" + webcamServoPosition + " newWebcamPosition:" + newServoPosition);

            if (newServoPosition < 0) {
                webcamServoPosition = 0.0;
                searchTimer.reset();
                searchState = SearchState.MOVE_LEFT;
            } else if (newServoPosition > 1) {
                webcamServoPosition = 1.0;
                searchTimer.reset();
                searchState = SearchState.MOVE_RIGHT;
            } else {
                webcamServoPosition = newServoPosition;
                webcamServo.setPosition(webcamServoPosition);
                searchTimer.reset();
                searchState = SearchState.CHECKING_FOR_APRILTAG;
            }
        }
    }

    private void updateRobotState() throws InterruptedException{
        updateBestAprilTag();
        updateAprilTagBearing();
        updateAprilTagYaw();
        updateCurrentWebcamAngle();
        updateCurrentRobotRange();
        updateCurrentRobotBearing();
        updateCurrentCameraCenter();
        Pose2D currentLocation = new Pose2D(
                DistanceUnit.INCH,
                currentCameraCenter.x,
                currentCameraCenter.y,
                AngleUnit.DEGREES,
                currentRobotBearing
        );
        if (currentCameraCenter.x != 10000) { // ignore initial values
            cameraPoses.addAndCalculate(
                currentLocation
            );
        }
        // odo.setPosition(currentLocation);
    }


    private void searchStateMachine() throws InterruptedException {
        // Log.d("FTC-23217-CameraCore", "searchStateMachine: Start! searchState:" + searchState + " searchTimer:" + searchTimer.seconds() + " webcam:" + webcamServoPosition + " april tag detected:" + aprilTagDetected());

        updateCurrentWebcamAngle();
        switch (searchState) {
            case WAITING_FOR_START:
                searchState = SearchState.CHECKING_FOR_APRILTAG;
                break;
            case CHECKING_FOR_APRILTAG:
                Log.d("FTC-23217-CameraCore", "searchStateMachine: Start! searchState:" + searchState + " searchTimer:" + searchTimer.seconds() + " webcam:" + webcamServoPosition + " april tag detected:" + aprilTagDetected());
                if (aprilTagDetected()) {
                    updateRobotState();
                    // updateCurrentRobotBounds();
                    // centerWebcamOnAprilTag();
                } else {
                    searchTimer.reset();
                    searchForAprilTag();
                }
                break;
            case SEARCHING_LEFT:
                if (aprilTagDetected()) {
                    previousSearchState = SearchState.SEARCHING_LEFT;
                    searchState = SearchState.CHECKING_FOR_APRILTAG;
                } else {
                    if (searchTimer.seconds() > SEARCH_WAIT_TIME) {
                        if (aprilTagDetected()) {
                            searchState = SearchState.CHECKING_FOR_APRILTAG;
                        } else {
                            searchState = SearchState.MOVE_LEFT;
                        }
                    }
                }
                break;
            case SEARCHING_RIGHT:
                if (aprilTagDetected()) {
                    previousSearchState = SearchState.SEARCHING_RIGHT;
                    searchState = SearchState.CHECKING_FOR_APRILTAG;
                } else {
                    if (searchTimer.seconds() > SEARCH_WAIT_TIME) {
                        if (aprilTagDetected()) {
                            searchState = SearchState.CHECKING_FOR_APRILTAG;
                        } else {
                            searchState = SearchState.MOVE_RIGHT;
                        }
                    }
                }
                break;
            case MOVE_LEFT:
                Log.d("FTC-23217-CameraCore", "searchStateMachine: Start! searchState:" + searchState + " searchTimer:" + searchTimer.seconds() + " webcam:" + webcamServoPosition + " april tag detected:" + aprilTagDetected());
                if (aprilTagDetected()) {
                    previousSearchState = SearchState.SEARCHING_LEFT;
                    searchState = SearchState.CHECKING_FOR_APRILTAG;
                } else {
                    webcamServoPosition -= WEBCAM_SEARCH_STEP;
                    if (webcamServoPosition < WEBCAM_SERVO_MIN) {
                        webcamServoPosition = WEBCAM_SERVO_MIN;
                        webcamServo.setPosition(webcamServoPosition);
                        searchTimer.reset();
                        searchState = SearchState.SEARCHING_RIGHT;
                    } else {
                        webcamServo.setPosition(webcamServoPosition);
                        searchTimer.reset();
                        searchState = SearchState.SEARCHING_LEFT;

                    }
                }
                break;
            case MOVE_RIGHT:
                Log.d("FTC-23217-CameraCore", "searchStateMachine: Start! searchState:" + searchState + " searchTimer:" + searchTimer.seconds() + " webcam:" + webcamServoPosition + " april tag detected:" + aprilTagDetected());
                if (aprilTagDetected()) {
                    previousSearchState = SearchState.SEARCHING_RIGHT;
                    searchState = SearchState.CHECKING_FOR_APRILTAG;
                } else {
                    webcamServoPosition += WEBCAM_SEARCH_STEP;
                    if (webcamServoPosition > WEBCAM_SERVO_MAX) {
                        webcamServoPosition = WEBCAM_SERVO_MAX;
                        searchTimer.reset();
                        searchState = SearchState.SEARCHING_LEFT;
                    } else {
                        webcamServo.setPosition(webcamServoPosition);
                        searchTimer.reset();
                        searchState = SearchState.SEARCHING_RIGHT;
                    }
                }
                break;
            default:

        }
    }
}
