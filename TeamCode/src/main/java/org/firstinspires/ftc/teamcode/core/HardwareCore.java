package org.firstinspires.ftc.teamcode.core;

// import com.qualcomm.hardware.rev.RevTouchSensor;
// import com.qualcomm.hardware.dfrobot.HuskyLens;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.robot.Robot;
// import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.RobotState;
import org.firstinspires.ftc.teamcode.library.VL53L4CD;

@Disabled
public class HardwareCore extends BaseCore {
    final Double SAFE_CAR_WASH_SERVO_POSITION = 0.83;
    final Double SAFE_CAR_WASH_SLIDE_SERVO_POSITION = 0.59;
    public DcMotor frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor, slideMotor;
    public Servo liftPivotServo, liftPivotServoReverse, clawPivotServo, clawServo, webcamServo;
    public Double liftPivotServoPosition, liftPivotServoReversePosition, clawPivotServoPosition, clawServoPosition, webcamServoPosition;
    public Rev2mDistanceSensor slideTofSensor;

    public Enum.TeamColor teamColor;
    public RobotState robotState = new RobotState(telemetry);
    protected boolean autonomousMode = false;

    private static double SERVO_LIFT_PIVOT_START = 0.56;
    private static double SERVO_LIFT_PIVOT_REVERSE_START = 0.43;
    private static double SERVO_CLAW_PIVOT_START =  0.14;
    private static double SERVO_CLAW_START = 0.7;
    private static double SERVO_WEBCAM_START = 0.6;
    // public IMU imu;


    public void runOpMode(boolean autonomousMode) throws InterruptedException {
        Log.d("FTC-23217", "HardwareCore Start.");
        super.runOpMode();
        this.autonomousMode = autonomousMode;
        setupMotors();
        setupServos();
        if (autonomousMode) {
            moveServosToStart();
        }
        setupSensors();
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

    private void moveServosToStart() throws InterruptedException {
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
        // leftUltrasonicSensor = hardwareMap.get(I2CXLMaxSonarEZ4.class, "leftUltrasonicSensor" );
        // rightUltrasonicSensor =  hardwareMap.get(I2CXLMaxSonarEZ4.class, "rightUltrasonicSensor" );
        slideTofSensor = hardwareMap.get(Rev2mDistanceSensor.class, "slideTofSensor");
        slideTofSensor.initialize();
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        // imu = hardwareMap.get(IMU.class, "imu");

    }

    private void setupCameras() throws InterruptedException {
        /* huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.initialize();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING); */
    }

}