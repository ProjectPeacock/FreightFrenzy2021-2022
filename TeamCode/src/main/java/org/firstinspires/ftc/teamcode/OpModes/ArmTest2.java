package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ArmTest2", group = "Competition")
//  @Disabled

public class ArmTest2 extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public ArmTest2(){

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
        telemetry.update();
//        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
//        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);

        // declare arm control constants
        double motorArm1Power = 0.02;
        double motorArm2Power = 0.02;
        int angle1Increment = 25;
        int angle2Increment = 25;
        int angle1Threshold = 10;
        int angle2Threshold = 10;
        int angle1Min = -250;
        int angle1Max = 250;

        // declare arm control variables
        int angle1TargetPos=0;
        int angle2TargetPos=0;
        int angle1StartPos=0;
        int angle2StartPos=0;

        // set both arm motors to position zero
        robot.motorArmAngle1.setTargetPosition(angle1TargetPos);
        robot.motorArmAngle2.setTargetPosition(angle2TargetPos);

        // position arm1
        robot.motorArmAngle1.setPower(motorArm1Power);
        while(Math.abs(angle1TargetPos - robot.motorArmAngle1.getCurrentPosition())>angle1Threshold){
            telemetry.addData("Arm1","initializing...");
            telemetry.update();
            sleep(250);
        }
        // stop arm1
        robot.motorArmAngle1.setPower(0);

        // position arm1
        robot.motorArmAngle2.setPower(motorArm2Power);
        while(Math.abs(angle2TargetPos - robot.motorArmAngle2.getCurrentPosition())>angle2Threshold){
            telemetry.addData("Arm2","initializing...");
            telemetry.update();
            sleep(250);
        }
        // stop arm2
        robot.motorArmAngle2.setPower(0);

        // save starting arm positions
        angle1StartPos = robot.motorArmAngle1.getCurrentPosition();
        angle2StartPos = robot.motorArmAngle2.getCurrentPosition();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Arm Angle 1 current  = ", robot.motorArmAngle1.getCurrentPosition());
            telemetry.addData("Arm Angle 1 target = ", robot.motorArmAngle1.getTargetPosition());
            telemetry.addData("Arm Angle 2 current = ", robot.motorArmAngle2.getCurrentPosition());
            telemetry.addData("Arm Angle 2 target = ", robot.motorArmAngle2.getTargetPosition());
            telemetry.update();

            // check for left stick movement
            if(gamepad1.left_stick_y<=-0.5){
                angle1TargetPos += angle1Increment;
            }else if(gamepad1.left_stick_y>=0.5){
                angle1TargetPos -= angle1Increment;
            }else{
            }

            // ensure arm1 stays within the limits
            Range.clip(angle1TargetPos,angle1Min,angle1Max);

            // change position if arm1 value changed or if we drifted
            if (Math.abs(angle1TargetPos - robot.motorArmAngle1.getCurrentPosition())>angle1Threshold) {
                robot.motorArmAngle1.setPower(motorArm1Power);
                while(Math.abs(angle1TargetPos - robot.motorArmAngle1.getCurrentPosition())>angle1Threshold){
                    telemetry.addData("Arm1 moving to: ",angle1TargetPos);
                    telemetry.update();
                    sleep(250);
                }
                robot.motorArmAngle1.setPower(0);
            }

            // check for right stick movement
            if(gamepad1.right_stick_y<=-0.5){
                angle2TargetPos += angle2Increment;
            }else if(gamepad1.right_stick_y>=0.5){
                angle2TargetPos -= angle2Increment;
            }else{
            }

            // change position if arm2 value changed or if we drifted
            if (Math.abs(angle2TargetPos - robot.motorArmAngle2.getCurrentPosition())>angle2Threshold) {
                robot.motorArmAngle2.setPower(motorArm2Power);
                while(Math.abs(angle1TargetPos - robot.motorArmAngle2.getCurrentPosition())>angle2Threshold){
                    telemetry.addData("Arm2 moving to: ",angle2TargetPos);
                    telemetry.update();
                    sleep(250);
                }
                robot.motorArmAngle2.setPower(0);
            }

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


