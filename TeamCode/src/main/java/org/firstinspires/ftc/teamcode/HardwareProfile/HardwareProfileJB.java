package org.firstinspires.ftc.teamcode.HardwareProfile;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class HardwareProfileJB {

    /*
     * Constants
     */
//        public final double COUNTS_PER_INCH = 307.699557;
    public final double PIVOT_SPEED = 0.5;
    //    public final double USD_COUNTS_PER_ROTATION = 360;  // US Digital encoder value - Odometry
    //        public final double USD_COUNTS_PER_INCH = 76.59556369;  // US Digital encoder value - Odometry
    public final double USD_COUNTS_PER_INCH = 307.699557;  // US Digital encoder value - Odometry
    public final double GB_COUNTS_PER_ROTATION = 28;    // goBilda encoder value


    public final int FOUR_RING_THRESHOLD = 137;
    public final int ONE_RING_THRESHOLD = 130;

    /*
     *  Constants & variables for wheel parameters
     */
    private final double DRIVE_TICKS_PER_ROTATION = 400;       // 3 x 4 x 1.5 x 28
    private final double WHEEL_RADIUS = 1.968505;         // 100mm wheel in inches
    private final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;    // circumference in inches
    public final double DRIVE_TICKS_PER_INCH = DRIVE_TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;

    /*
     *  Constants & variables for different shooting positions
     */
    public final double SHOOTER_TARGET_RPM= 4550;       // Target RPM is normal game shooting
    public final double SHOOTER_PS_RPM = 4150;          // PS_RPM is for power shoot shooting
    public final double SHOOTER_AUTO_RPM = 4500;        // AUTO_RPM is for shooting position for auto QUAD shooting
    public final double SHOOTER_IDLE_RPM = 3500;        // IDLE_RPM is for
    public final double SHOOTER_AUTO_FIELD_RPM = 4000;

    /*
     * Servo Position parameters
     */
    public final double SERVO_RING_UP = 0.10;
    public final double SERVO_RING_DOWN = 0.53;
    public final double SERVO_TRANSFER_UP = 0.80;
    public final double SERVO_TRANSFER_DOWN = 0.6;
    public final double SERVO_KICK_OUT = 0.43;
    public final double SERVO_KICK_IN = 0.68;
    public final double SERVO_WOBBLE_GRAB_OPEN = 0.4;
    public final double SERVO_WOBBLE_GRAB_CLOSE = 0.00;
    public final double SERVO_INTAKE_EXTEND = 0.65;
    public final double SERVO_INTAKE_RETRACT = 0.20;
    public final double SERVO_WOBBLE_ARM_EXTEND = 0.8;
    public final double SERVO_WOBBLE_ARM_STOW = 0.22;
    public final double SERVO_WOBBLE_ARM_UP = 0.4;
    public final long SERVO_RING_DOWN_DELAY = 175;          // delay in milliseconds
    public final long SERVO_TRANSFER_UP_DELAY = 100;        // delay in milliseconds

    /*
     * Hardware devices
     */
    public DcMotor motorR1 = null;  // Right Front Drive Motor
    public DcMotor motorL1 = null;  // Left Front Drive Motor
    public DcMotor motorL2 = null;  // Left Rear  Drive Motor
    public DcMotor motorR2 = null;  // Right Rear Drive Motor
    public DcMotor motorArmBase = null; // Pivoting base for arm
    public DcMotor motorArmPivot = null; // Pivoting elbow for arm
    public DcMotor motorBeater = null; // motor for beater bar
    public DcMotor motorChainsaw = null; // motor for chainsaw
//    public DcMotor motorArm = null; //Arm angle idk
//    public DcMotor motorTurretTurner = null; // Motor that turns the turret
//    public DcMotor motorTSEArm = null; // Team Scoring Element Arm

//    public DcMotor motorChainsaw = null;  // Duck Carousel motor

    public Servo servoBucket;
    public Servo  servoTurrentTurn1;
    public Servo  servoTurrentTurn2;
    public Servo servoIntakeDeploy1;
    public Servo servoIntakeDeploy2;
    public Servo servoIntakePivot;

    public TouchSensor limitSwicth1;
    public TouchSensor limitSwicth2;

    public BNO055IMU imu;       // Internal accelerometer / Gyro sensor

//    public WebcamName webcam;

    //public Servo servoRing;

    //public TouchSensor touchWobble;


    public RevBlinkinLedDriver LEDPort;

    /* Constructor */
    public HardwareProfileJB() {

    }   // end of HardwareProfile method

    public void init(HardwareMap hwMap) {

//        HardwareMap hwMap = ahwMap;

        /*
         * Initialize Motors
         */


        motorL1 = hwMap.dcMotor.get("motorL1");
        motorL1.setDirection(DcMotor.Direction.REVERSE);
        motorL1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL1.setPower(0);

        motorL2 = hwMap.dcMotor.get("motorL2");
        motorL2.setDirection(DcMotor.Direction.REVERSE);
        motorL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL2.setPower(0);

        motorR1 = hwMap.dcMotor.get("motorR1");
        motorR1.setDirection(DcMotor.Direction.FORWARD);
        motorR1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR1.setPower(0);

        motorR2 = hwMap.dcMotor.get("motorR2");
        motorR2.setDirection(DcMotor.Direction.FORWARD);
        motorR2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR2.setPower(0);

//        motorChainsaw = hwMap.get(DcMotor.class,"motorChainsaw");
//        motorChainsaw.setDirection(DcMotor.Direction.FORWARD);
     //   motorChainsaw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //   motorChainsaw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorChainsaw.setPower(0);

/*        motorBeater = hwMap.dcMotor.get("motorBeater");
        motorBeater.setDirection(DcMotor.Direction.FORWARD);
        motorBeater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBeater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBeater.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBeater.setPower(0);

        motorArm = hwMap.dcMotor.get("motorArm");
        motorArm.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setPower(0);

        motorTurretTurner = hwMap.dcMotor.get("motorTurretTurner");
        motorTurretTurner.setDirection(DcMotor.Direction.FORWARD);
        motorTurretTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTurretTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTurretTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTurretTurner.setPower(0);

        motorTSEArm = hwMap.dcMotor.get("motorTSEArm");
        motorTSEArm.setDirection(DcMotor.Direction.FORWARD);
        motorTSEArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTSEArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorTSEArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTSEArm.setPower(0);
*/
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
/*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
*/
        /* Webcam device will go here */
//        webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }   // end of init() method

}