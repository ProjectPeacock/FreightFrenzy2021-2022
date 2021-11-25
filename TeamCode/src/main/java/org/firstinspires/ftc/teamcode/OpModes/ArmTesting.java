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
        double bucketAngle=0.5;
        int bumpCount=0;
        boolean toggleReadyDown=false;
        boolean toggleReadyUp=false;
        telemetry.addData("Arm Angle 1:",robot.motorArmAngle1.getCurrentPosition());
        telemetry.addData("Arm Angle 2:",robot.motorArmAngle2.getCurrentPosition());
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Arm Angle 1 = ", robot.motorArmAngle1.getCurrentPosition());
            telemetry.addData("Arm Angle 2 = ", robot.motorArmAngle2.getCurrentPosition());
            telemetry.update();

            //move arm to intake position
            if(gamepad2.right_bumper){
                angle2Pos=45;
                robot.motorArmAngle2.setTargetPosition(angle2Pos);
                while (robot.motorArmAngle2.getCurrentPosition()>800){
                }
                angle1Pos=1150;
                bumpCount=0;
            }else{

            }
            //allows for toggling between arm positions and not only going to lowest one because of button being held
            if(gamepad2.left_bumper==false){
                toggleReadyDown=true;
            }
            if(gamepad2.dpad_up==false){
                toggleReadyUp=true;
            }

            //move arm to scoring position
            if(gamepad2.left_bumper && toggleReadyDown){
                toggleReadyDown=false;
                if(bumpCount<3) {
                    bumpCount += 1;
                }
            }

            if(gamepad2.dpad_up && toggleReadyUp){
                toggleReadyUp=false;
                if(bumpCount>1) {
                    bumpCount -= 1;
                }
            }

            //scoring positions
            if(bumpCount==1){
                angle1Pos=0;
                robot.motorArmAngle1.setTargetPosition(angle1Pos);
                while(robot.motorArmAngle1.getCurrentPosition()>750){

                }
                angle2Pos=robot.HIGH_PLATFORM;
            }else if(bumpCount==2){
                angle1Pos=0;
                angle2Pos=robot.MID_PLATFORM;
            }else if(bumpCount==3){
                angle1Pos=-500;
                angle2Pos=robot.LOW_PLATFORM;
            }

            if(gamepad2.a){
                bucketAngle=-1;
            } else{
                bucketAngle=0.5;
            }

            if(gamepad2.dpad_down){
                bumpCount=0;
                angle1Pos=0;
                angle2Pos=0;
            }else{

            }

            robot.motorArmAngle2.setTargetPosition(angle2Pos);
            robot.motorArmAngle2.setPower(1);
            robot.motorArmAngle1.setTargetPosition(angle1Pos);
            robot.motorArmAngle1.setPower(1);
            robot.bucketDump.setPosition(bucketAngle);



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


