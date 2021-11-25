package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ArmTesting", group = "Competition")
//  @Disabled

public class ArmTesting extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public ArmTesting(){

    }   // end of BrokenBotTS constructor

    public void runOpMode(){



        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);

        /*
         * Initialize the drive class
         */

        /*
         * Calibrate / initialize the gyro sensor
         */

        telemetry.addData("Robot state = ", "INITIALIZED");
//        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
//        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        int angle1Pos=0;
        int angle2Pos=0;
        telemetry.addData("Arm Angle 1:",robot.motorArmAngle1.getCurrentPosition());
        telemetry.addData("Arm Angle 2:",robot.motorArmAngle2.getCurrentPosition());
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Arm Angle 1 = ", robot.motorArmAngle1.getCurrentPosition());
            telemetry.addData("Arm Angle 2 = ", robot.motorArmAngle2.getCurrentPosition());
            telemetry.update();

            if(gamepad2.dpad_up){

                angle2Pos=-950;
            } else if(gamepad2.dpad_down){
                angle2Pos=0;
            } else{

            }
            robot.motorArmAngle2.setTargetPosition(angle2Pos);
            robot.motorArmAngle2.setPower(1);

            if(gamepad2.a){
                robot.bucketDump.setPosition(-1);
            } else robot.bucketDump.setPosition(0.5);


            /**
             * #################################################################################
             * #################################################################################
             * #################      PROVIDE USER FEEDBACK    #################################
             * #################################################################################
             * #################################################################################
             *
            telemetry.addData("Motor Shooter1 Encoder = ", robot.motorShooter1.getCurrentPosition());
            telemetry.addData("Motor Shooter2 Encoder = ", robot.motorShooter2.getCurrentPosition());
            telemetry.addData("Motor RF Encoder = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("Motor LF Encoder = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("Motor RR Encoder = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("Motor LR Encoder = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("Calculated Distance = ", drive.calcDistance(0,0,0,0,0));
            telemetry.addData("Shooter RPM = ", (robot.motorShooter1.getVelocity() / 28 * 60));
            telemetry.update(); */
        }   // end of while opModeIsActive()
    }   // end of runOpMode method

}   // end of TeleOp.java class


