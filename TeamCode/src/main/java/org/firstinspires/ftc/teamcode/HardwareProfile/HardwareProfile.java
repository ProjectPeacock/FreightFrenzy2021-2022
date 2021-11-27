package org.firstinspires.ftc.teamcode.HardwareProfile;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareProfile {

    /*
     * Constants
     */
    public final int ARM_THREAD_SLEEP=100; //arm thread sleepTime

    public final double DRIVE_MULTIPLIER=0.50;
    public final double TURN_POWER=0.35;
    //intake deployment servos
    public final double BLUE_ZERO = 0.5; //IntakeDeployBlue zero pos
    public final double PINK_ZERO = 0.5; //IntakeDeployPink zero pos
    public final double INTAKE_DEPLOY_BLUE = 0.3; //Distance to deploy intake
    public final double INTAKE_DEPLOY_PINK = INTAKE_DEPLOY_BLUE+0.13; // distance to deploy pink intake
    public final double INTAKE_OUTTAKE = 0.05; //Distance to deploy intake for outtaking
    public final double INTAKE_TILT_INPUT= 0.72;
    public final double INTAKE_TILT_OUTPUT= 0.5;
    //chainsaw power
    public final double CHAIN_POW = 0.4; //motorChainsaw power

    //intake power
    public final double INTAKE_POW = 1; //intaking power
    public final double INTAKE_IDLE = 1; //intake idling power (for using intake as outtake)
    public final double INTAKE_REVERSE_POW = -0.8; //intake reverse power (for using intake as outtake)

    //arm scoring positions
    public final int HIGH_PLATFORM=1700;
    public final int MID_PLATFORM=2350;
    public final int LOW_PLATFORM=2350;

    //arm intake positions
    public final int ARM_1_INTAKE=1100;
    public final int ARM_2_INTAKE=45;
    /*
     * Hardware devices
     */
    //Motors
    public DcMotor motorR1 = null;  // Right Front Drive Motor
    public DcMotor motorL1 = null;  // Left Front Drive Motor
    public DcMotor motorL2 = null;  // Left Rear  Drive Motor
    public DcMotor motorR2 = null;  // Right Rear Drive Motor
    public DcMotorEx motorArmAngle2 = null; // Arm upper angle motor
    public DcMotorEx motorArmAngle1 = null; // Arm base angle motor
    public DcMotor motorIntake = null; // Intake motor
    public DcMotor motorChainsaw = null; // Chainsaw motor

    public BNO055IMU imu;       // Internal accelerometer / Gyro sensor

    //Servos
    public Servo intakeDeployBlue = null; //Right intake deploy servo
    public Servo intakeDeployPink = null; //Left intake deploy servo
    public Servo intakeTilt = null; //Intake ramp position servo
    public Servo bucketDump = null; //Bucket turning servo
    //public WebcamName webcam;
    public DistanceSensor sensorDistPink=null;
    public DistanceSensor sensorDistBlue=null;
    public RevBlinkinLedDriver LEDPort;

    /* Constructor */
    public HardwareProfile() {

    }   // end of HardwareProfile method

    public void init(HardwareMap hwMap) {

//        HardwareMap hwMap = ahwMap;

        /*
         * Initialize Motors
         */


        motorL1 = hwMap.dcMotor.get("motorL1");
        motorL1.setDirection(DcMotor.Direction.REVERSE);
        motorL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL1.setPower(0);

        motorL2 = hwMap.dcMotor.get("motorL2");
        motorL2.setDirection(DcMotor.Direction.REVERSE);
        motorL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL2.setPower(0);

        motorR1 = hwMap.dcMotor.get("motorR1");
        motorR1.setDirection(DcMotor.Direction.FORWARD);
        motorR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR1.setPower(0);

        motorR2 = hwMap.dcMotor.get("motorR2");
        motorR2.setDirection(DcMotor.Direction.FORWARD);
        motorR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR2.setPower(0);

        motorIntake = hwMap.dcMotor.get("motorIntake");
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntake.setPower(0);

        motorChainsaw = hwMap.dcMotor.get("motorChainsaw");
        motorChainsaw.setDirection(DcMotor.Direction.FORWARD);
        motorChainsaw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorChainsaw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorChainsaw.setPower(0);

        intakeDeployBlue = hwMap.servo.get("intakeDeployBlue");
        intakeDeployPink = hwMap.servo.get("intakeDeployPink");
        intakeTilt = hwMap.servo.get("intakeTilt");
        bucketDump = hwMap.servo.get("bucketDump");

        motorArmAngle1 = hwMap.get(DcMotorEx.class,"motorArmAngle1");
        motorArmAngle1.setDirection(DcMotor.Direction.REVERSE);
        motorArmAngle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmAngle1.setTargetPosition(0);
        motorArmAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmAngle1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmAngle1.setPower(0);

        motorArmAngle2 = hwMap.get(DcMotorEx.class, "motorArmAngle2");
        motorArmAngle2.setDirection(DcMotor.Direction.REVERSE);
        motorArmAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmAngle2.setTargetPosition(0);
        motorArmAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmAngle2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmAngle2.setPower(0);
/*
        motorOdometry = hwMap.dcMotor.get("motorOdometry");
        motorOdometry.setDirection(DcMotor.Direction.FORWARD);
        motorOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOdometry.setPower(0);

        /*
         * Initialize LED Controller
         */
  //      LEDPort = hwMap.get(RevBlinkinLedDriver.class, "LEDPort");
  //      LEDPort.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        /*
         * Initialize Sensors
         */
        sensorDistBlue=hwMap.get(DistanceSensor.class, "sensorDistBlue");
        sensorDistPink=hwMap.get(DistanceSensor.class, "sensorDistPink");
        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        /* Webcam device will go here */
//        webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }   // end of init() method

}