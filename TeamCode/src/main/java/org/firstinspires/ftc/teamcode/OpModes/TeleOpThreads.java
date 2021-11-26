package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.ArmControlLibrary;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpThreads", group = "Competition")
//  @Disabled

public class TeleOpThreads extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public TeleOpThreads(){

    }   // end of BrokenBotTS constructor

    public void runOpMode(){
        ArmControlLibrary armControl = new ArmControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread armController = new Thread(armControl);
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
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_TILT_INPUT);
        double bucketAngle=0.5;
        int bumpCount=0;
        boolean toggleReadyDown=false;
        boolean toggleReadyUp=false;
        boolean isDeployed=false;
        waitForStart();
        armController.start();

        while(opModeIsActive()) {
            /*
            DRIVE CONTROLS:
            Left Stick - forward/backward
            Right Stick - turn
            A - intake
            B - reverse intake
            X - toggle scoring position down
            Y - intake position
            Dpad up - toggle scoring position up
            Dpad down - reset arm to zero
            Dpad right - dump bucket
            Left bumper - chainsaw direction 1
            Right bumper - chainsaw direction 2

             *Drive Control section
            */
            telemetry.addData("Test = ", robot.motorArmAngle1.getCurrentPosition());
            telemetry.addData("Arm #2 angle = ", robot.motorArmAngle2.getCurrentPosition());
            telemetry.addData("tilt = ", robot.intakeTilt.getPosition());
            telemetry.update();
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            double left  = drive + turn;
            double right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            robot.motorL1.setPower(left);
            robot.motorL2.setPower(left);
            robot.motorR1.setPower(right);
            robot.motorR2.setPower(right);

            //intake control
            if(gamepad1.a) {
                //turns intake on
                armControl.intakeOn(isDeployed);
            }else if(gamepad1.b) {
                //reverses intake
                robot.motorIntake.setPower(robot.INTAKE_REVERSE_POW);
            }else{
                //turns intake off
                armControl.intakeOff(isDeployed);
            }

            //intake ramp controls (GP1, Dpad)
            /*
            if(gamepad1.dpad_left){
                robot.intakeTilt.setPosition(robot.INTAKE_TILT_INPUT);
            } else if(gamepad1.dpad_right){
                robot.intakeTilt.setPosition(robot.INTAKE_TILT_OUTPUT);
            } else{
            }
            */


            //chainsaw controls (GP1, Bumpers)
            if(gamepad1.left_bumper){
                robot.motorChainsaw.setPower(robot.CHAIN_POW);
            }else if(gamepad1.right_bumper){
                robot.motorChainsaw.setPower(-robot.CHAIN_POW);
            }else{
                robot.motorChainsaw.setPower(0);
            }


            //arm control section
            telemetry.addData("Arm Angle 1 = ", robot.motorArmAngle1.getCurrentPosition());
            telemetry.addData("Arm Angle 2 = ", robot.motorArmAngle2.getCurrentPosition());
            telemetry.update();

            //allows for toggling between arm positions and not only going to lowest one because of button being held
            if(!gamepad1.x){
                toggleReadyDown=true;
            }
            if(!gamepad1.dpad_up){
                toggleReadyUp=true;
            }

            //adds 1 to bumpCount if x isn't held down
            if(gamepad1.x && toggleReadyDown){
                toggleReadyDown=false;
                if(bumpCount<3) {
                    bumpCount += 1;
                }
            }

            //removes 1 from bumpCount if dpad up isn't held down
            if(gamepad1.dpad_up && toggleReadyUp){
                toggleReadyUp=false;
                if(bumpCount>1) {
                    bumpCount -= 1;
                }
            }

            //counts how many times x has been pressed (what position to go to to score)
            if(bumpCount>0){
                isDeployed=true;
            }

            //move arm to scoring positions
            if(bumpCount==1){
                armControl.scoringPos1();
            }else if(bumpCount==2){
                armControl.scoringPos2();
            }else if(bumpCount==3){
                armControl.scoringPos3();
            }

            //reset arm to zero with or without scoring
            if(gamepad1.dpad_down){
                bumpCount=0;
                isDeployed=false;
                armControl.moveToZero();
            }else{

            }

            //reset arm to zero AFTER SCORING
            if(gamepad1.y){
                if(isDeployed){
                    armControl.resetArm();
                    bumpCount=0;
                    isDeployed=false;
                }
            }

            //bucket control
            if(gamepad1.dpad_right){
                bucketAngle=-1;
            } else{
                bucketAngle=0.5;
            }
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
        armControl.stop();
    }   // end of runOpMode method

}   // end of TeleOp.java class


