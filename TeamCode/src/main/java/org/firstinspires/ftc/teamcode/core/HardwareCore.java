package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.library.VL53L4CD;

@Disabled
public class HardwareCore extends LinearOpMode {
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, leftSlideMotor, rightSlideMotor, liftMotor, carWashMotor;
    public int leftSlideMotorPosition, rightSlideMotorPosition;
    public Servo clawServo, rampServo, carWashLiftServo, hookServo, rightClawPivotServo, leftClawPivotServo, rightClawGripServo, leftClawGripServo, rightClawDoorServo, leftClawDoorServo, droneServo;
    // public I2CXLMaxSonarEZ4 leftUltrasonicSensor, rightUltrasonicSensor, frontUltrasonicSensor;
    public VL53L4CD frontTofSensor;
    public Rev2mDistanceSensor slideTofSensor, angryMantisTofSensor;
    public DcMotor rightOdoEncoder, leftOdoEncoder, rearOdoEncoder;
    public TouchSensor angryMantisTouchSensor;
    public boolean robotIsMoving;


    @Override
    public void runOpMode() throws InterruptedException {
        setupMotors();
        setupServos();
        setupSensors();
        setupOdometry();
    }

    private void setupMotors() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        // Wheel Motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("rearLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("rearRightMotor");

        // Function Motors
        leftSlideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
        rightSlideMotor = hardwareMap.dcMotor.get("rightSlideMotor");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        carWashMotor = hardwareMap.dcMotor.get("carWashMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setupServos() throws InterruptedException {
        carWashLiftServo = hardwareMap.servo.get("carWashLiftServo");
        hookServo = hardwareMap.servo.get("hookServo");
        rightClawPivotServo = hardwareMap.servo.get("rightClawPivotServo");
        rightClawGripServo = hardwareMap.servo.get("rightClawGripServo");
        leftClawGripServo = hardwareMap.servo.get("leftClawGripServo");
        rightClawDoorServo = hardwareMap.servo.get("rightClawDoorServo");
        leftClawDoorServo = hardwareMap.servo.get("leftClawDoorServo");
        droneServo = hardwareMap.servo.get("droneServo");
    }

    private void setupSensors() throws InterruptedException {
        // leftUltrasonicSensor = hardwareMap.get(I2CXLMaxSonarEZ4.class, "leftUltrasonicSensor" );
        // rightUltrasonicSensor =  hardwareMap.get(I2CXLMaxSonarEZ4.class, "rightUltrasonicSensor" );
        slideTofSensor = hardwareMap.get(Rev2mDistanceSensor.class, "slideTofSensor");
        slideTofSensor.initialize();
        angryMantisTofSensor = hardwareMap.get(Rev2mDistanceSensor.class, "angryMantisTofSensor");
        angryMantisTofSensor.initialize();
        angryMantisTouchSensor = hardwareMap.get(RevTouchSensor.class, "angryMantisTouchSensor");
    }

    public void setupOdometry() throws InterruptedException {
        rightOdoEncoder = hardwareMap.dcMotor.get("frontLeftMotor");
        leftOdoEncoder = hardwareMap.dcMotor.get("rearRightMotor");
        rearOdoEncoder = hardwareMap.dcMotor.get("rearLeftMotor");
    }

    /*
    ElapsedTime servoSensorTimer = new ElapsedTime();
    public enum SweepState {
        SWEEP_FORWARD,
        WAIT_FORWARD,
        SWEEP_REVERSE,
        WAIT_REVERSE,
    };

    protected SweepState servoSensorSweepState = SweepState.SWEEP_FORWARD;

    protected void startHardwareTimers() {
        servoSensorTimer.startTime();
    }

    protected void servoSensorSweepStateMachine() {
        final double SERVO_TURN_TIME = 0.7; // this should be enough time to move the servo to the new 45 degree away position.
        final double SERVO_WAIT_TIME = 0.3; // this should be enough time to request a reading, and get it from the ultrasonic sensors
        final double REAR_LEFT_SERVO_START_POSITION = 0.1; // rear left Ultrasonic faces XXX when here
        final double REAR_LEFT_SERVO_END_POSITION = 0.55; // rear left Ultrasonic faces XXX when here
        final double REAR_RIGHT_SERVO_START_POSITION = 0.45; // rear right Ultrasonic faces XXX when here
        final double REAR_RIGHT_SERVO_END_POSITION = 0.0; // rear right Ultrasonic faces XXX when here
        final double FRONT_LEFT_SERVO_START_POSITION = 0.0; // front left Ultrasonic faces XXX when here
        final double FRONT_LEFT_SERVO_END_POSITION = 0.0; // front left Ultrasonic faces XXX when here
        final double FRONT_RIGHT_SERVO_START_POSITION = 0.0; // front right Ultrasonic faces XXX when here
        final double FRONT_RIGHT_SERVO_END_POSITION = 0.0; // front right Ultrasonic faces XXX when here
        // TODO: Add Front Servo Sensors

        switch (servoSensorSweepState) {
            case SWEEP_FORWARD:
                if (servoSensorTimer.seconds() > SERVO_TURN_TIME) {
                    rearLeftSensorServo.setPosition(REAR_LEFT_SERVO_END_POSITION);
                    rearRightSensorServo.setPosition(REAR_RIGHT_SERVO_END_POSITION);
                    servoSensorSweepState = SweepState.WAIT_FORWARD;
                    servoSensorTimer.reset();
                }
                break;
            case WAIT_FORWARD:
                if (servoSensorTimer.seconds() > SERVO_WAIT_TIME) {
                    servoSensorSweepState = SweepState.SWEEP_REVERSE;
                    servoSensorTimer.reset();
                }
                break;
            case SWEEP_REVERSE:
                if (servoSensorTimer.seconds() > SERVO_TURN_TIME) {
                    rearLeftSensorServo.setPosition(REAR_LEFT_SERVO_START_POSITION);
                    rearRightSensorServo.setPosition(REAR_RIGHT_SERVO_START_POSITION);
                    servoSensorSweepState = SweepState.WAIT_REVERSE;
                    servoSensorTimer.reset();
                }
                break;
            case WAIT_REVERSE:
                if (servoSensorTimer.seconds() > SERVO_WAIT_TIME) {
                    servoSensorSweepState = SweepState.SWEEP_FORWARD;
                    servoSensorTimer.reset();
                }
                break;
            default:
                // shouldn't get here.
        }
    }
     */
}