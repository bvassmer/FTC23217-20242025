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
// import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.teamcode.Enum;
import org.firstinspires.ftc.teamcode.library.VL53L4CD;

@Disabled
public class HardwareCore extends LinearOpMode {
    final Double SAFE_CAR_WASH_SERVO_POSITION = 0.83;
    final Double SAFE_CAR_WASH_SLIDE_SERVO_POSITION = 0.59;
    public DcMotor frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor, slideMotor;
    public Servo liftPivotServo, liftPivotServoReverse, clawPivotServo, clawServo, webcamServo;
    public Double liftPivotServoPosition, liftPivotServoReversePosition, clawPivotServoPosition, clawServoPosition, webcamServoPosition;
    public Rev2mDistanceSensor slideTofSensor;
    // public HuskyLens huskyLens;
    public boolean robotIsMoving;
    public Enum.TeamColor teamColor;
    // public IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        Log.d("FTC-23217", "HardwareCore Start.");
        setupMotors();
        setupServos();
        setupSensors();
        setupCameras();
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
        liftPivotServo.setPosition(0.56);
        liftPivotServoReverse.setPosition(0.43);
        liftPivotServoPosition = liftPivotServo.getPosition();

        clawPivotServo = hardwareMap.servo.get("clawPivotServo");
        clawPivotServo.setPosition(0.14);
        clawPivotServoPosition = clawPivotServo.getPosition();
        clawServo = hardwareMap.servo.get("clawServo");
        clawServo.setPosition(0.7);
        clawServoPosition = clawServo.getPosition();

        webcamServo = hardwareMap.servo.get("webcamServo");
        webcamServo.setPosition(0.6);
        webcamServoPosition = webcamServo.getPosition();
    }

    private void setupSensors() throws InterruptedException {
        // leftUltrasonicSensor = hardwareMap.get(I2CXLMaxSonarEZ4.class, "leftUltrasonicSensor" );
        // rightUltrasonicSensor =  hardwareMap.get(I2CXLMaxSonarEZ4.class, "rightUltrasonicSensor" );
        slideTofSensor = hardwareMap.get(Rev2mDistanceSensor.class, "slideTofSensor");
        slideTofSensor.initialize();
        // imu = hardwareMap.get(IMU.class, "imu");

    }

    private void setupCameras() throws InterruptedException {
        /* huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.initialize();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING); */
    }

}