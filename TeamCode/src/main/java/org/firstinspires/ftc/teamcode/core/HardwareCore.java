package org.firstinspires.ftc.teamcode.core;

// import com.qualcomm.hardware.rev.RevTouchSensor;
// import com.qualcomm.hardware.dfrobot.HuskyLens;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;


import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.library.I2CXLMaxSonarEZ4;

@Disabled
public class HardwareCore extends BaseCore {
    final Double SAFE_CAR_WASH_SERVO_POSITION = 0.83;
    final Double SAFE_CAR_WASH_SLIDE_SERVO_POSITION = 0.59;
    protected DcMotor frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor, slideMotor;
    protected Servo liftPivotServo, liftPivotServoReverse, clawPivotServo, clawServo, webcamServo;
    protected Double liftPivotServoPosition, liftPivotServoReversePosition, clawPivotServoPosition, clawServoPosition, webcamServoPosition;
    protected I2CXLMaxSonarEZ4 frontUltraSonicSensor;
    protected Rev2mDistanceSensor slideTofSensor, frontTofSensor;
    protected RevTouchSensor rightTouchSensor, leftTouchSensor;
    protected RevColorSensorV3 clawColorSensor;
    protected GoBildaPinpointDriver odo;

    protected Enum.TeamColor teamColor;

    protected double oldTime = 0;

    // public IMU imu;


    public void runOpMode(boolean autonomousMode) throws InterruptedException {
        Log.d("FTC-23217", "HardwareCore Start.");
        super.runOpMode();
        setupMotors();
        setupServos();
        initializeServos();
        if (autonomousMode) {
            moveServosToStart();
        }
        setupSensors();
    }

    protected void workers() throws InterruptedException {
    }

    private void setupMotors() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        // Wheel Motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        rearLeftMotor = hardwareMap.dcMotor.get("rearLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        rearRightMotor = hardwareMap.dcMotor.get("rearRightMotor");

        // Function Motors
        slideMotor = hardwareMap.dcMotor.get("slideMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setupServos() throws InterruptedException {
        liftPivotServo = hardwareMap.servo.get("liftPivotServo");
        liftPivotServoReverse = hardwareMap.servo.get("liftPivotServoReverse");
        clawPivotServo = hardwareMap.servo.get("clawPivotServo");
        clawServo = hardwareMap.servo.get("clawServo");
        webcamServo = hardwareMap.servo.get("webcamServo");
    }

    private void initializeServos() throws InterruptedException {
        if (liftPivotServoPosition == null || liftPivotServoPosition == 0.0) {
           liftPivotServoPosition =  0.495;
        }
        if (liftPivotServoReversePosition == null || liftPivotServoReversePosition == 0.0) {
            liftPivotServoReversePosition = 0.49;
        }
        if (clawPivotServoPosition == null || clawPivotServoPosition == 0.0) {
            clawPivotServoPosition = 0.49;
        }
        if (clawServoPosition == null || clawServoPosition == 0.0) {
            clawServoPosition = 0.75;
        }
        if (webcamServoPosition == null || webcamServoPosition == 0.0) {
            webcamServoPosition = 0.6;
        }
    }

    private void moveServosToStart() throws InterruptedException {
        double SERVO_LIFT_PIVOT_START = 0.56;
        double SERVO_LIFT_PIVOT_REVERSE_START = 0.43;
        double SERVO_CLAW_PIVOT_START = 0.14;
        double SERVO_CLAW_START = 0.75;
        double SERVO_WEBCAM_START = 0.4;

        liftPivotServoPosition = SERVO_LIFT_PIVOT_START;
        liftPivotServoReversePosition = SERVO_LIFT_PIVOT_REVERSE_START;
        liftPivotServo.setPosition(liftPivotServoPosition);
        liftPivotServoReverse.setPosition(liftPivotServoReversePosition);

        clawPivotServoPosition = SERVO_CLAW_PIVOT_START;
        clawPivotServo.setPosition(clawPivotServoPosition);

        clawServoPosition = SERVO_CLAW_START;
        clawServo.setPosition(clawServoPosition);

        webcamServoPosition = SERVO_WEBCAM_START;
        webcamServo.setPosition(webcamServoPosition);
    }

    private void setupSensors() throws InterruptedException {
        frontUltraSonicSensor = hardwareMap.get(I2CXLMaxSonarEZ4.class, "frontUltraSonicSensor" );
        frontUltraSonicSensor.initialize();
        slideTofSensor = hardwareMap.get(Rev2mDistanceSensor.class, "slideTofSensor");
        slideTofSensor.initialize();
        frontTofSensor = hardwareMap.get(Rev2mDistanceSensor.class, "frontTofSensor");
        frontTofSensor.initialize();
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.resetPosAndIMU();
        clawColorSensor = hardwareMap.get(RevColorSensorV3.class, "clawColorSensor");
        leftTouchSensor = hardwareMap.get(RevTouchSensor.class, "leftTouchSensor");
        rightTouchSensor = hardwareMap.get(RevTouchSensor.class, "rightTouchSensor");
        // imu = hardwareMap.get(IMU.class, "imu");

    }

    private void setupCameras() throws InterruptedException {
        /* huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.initialize();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING); */
    }

}