package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Threads.MechControlLibrary;
import org.firstinspires.ftc.teamcode.Threads.TurretControlLibrary;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Anthony's Super Duper Extra Super Special Limited Edition TeleOp", group = "Competition")
//  @Disabled

public class TeleOpAnthonyControls extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public TeleOpAnthonyControls(){

    }   // end of BrokenBotTS constructor

    public void runOpMode(){
        //mechanism control thread
        MechControlLibrary mechControl = new MechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread mechController = new Thread(mechControl);
        //turret control thread
        TurretControlLibrary turretControl = new TurretControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread turretController = new Thread(turretControl);
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
        double bucketAngle=0.5;
        int bumpCount=0;
        int chainsawMode=0;
        boolean toggleReadyDown=false;
        boolean toggleReadyUp=false;
        boolean isDeployed=false;
        boolean intakeDown=false;
        boolean toggleIntake=false;
        boolean turretToggle=false;

        boolean TSEMode=false;
        boolean TSEtoggle=false;

        double chainsawPower=1;
        boolean chainsawToggle=false;

        int turretPreset=0;
        double turn, drive, left, right, max;
        int turretPosition=0;
        int turretThreshold=2;

        waitForStart();
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_STARTING_POS);
        mechController.start();
        turretController.start();

        while(opModeIsActive()) {
            /*
            DRIVE CONTROLS:
            GP1/
            Left Stick - forward/backward
            Right Stick - turn
            A - intake / retract intake
            L/R bumpers - chainsaw
            L/R triggers - fast chainsaw
            */

//drive control section (GP1, Joysticks)
            drive = -gamepad1.left_stick_y*robot.DRIVE_MULTIPLIER;
            if(drive<0){
                drive*=robot.REVERSE_MULTIPLIER;
            }
            turn  =  gamepad1.right_stick_x*robot.TURN_MULTIPLIER;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            if(Math.abs(left)>1){
                left/=Math.abs(left);
            }
            if(Math.abs(right)>1){
                left/=Math.abs(right);
            }
            //100% power forward
            if(-gamepad1.left_stick_y>0.95&&-gamepad1.right_stick_y>0.95){
                left=1;
                right=1;
            }
            //100% power in reverse
            if(-gamepad1.left_stick_y<0-.95&&-gamepad1.right_stick_y<-0.95){
                left=-1;
                right=-1;
            }

            // Output the safe vales to the motor drives.
            robot.motorL1.setPower(left);
            robot.motorL2.setPower(left);
            robot.motorR1.setPower(right);
            robot.motorR2.setPower(right);
//end of drive controls

//intake control section (GP1, Right Bumper)
            //allows for intake toggle and not button hold down
            if(!gamepad1.right_bumper){
                toggleIntake=true;
            }

            //if intake isn't deployed, deploy it & vice versa

            if(gamepad1.right_bumper&&toggleIntake){
                toggleIntake=false;
                intakeDown=!intakeDown;
                isDeployed=false;
                bumpCount=0;
            }
            //check if intake needs to be reversed and then deploy or retract
            if (!gamepad1.left_bumper) {
                if (intakeDown) {
                    if(Math.abs(robot.turrentEncoder.getCurrentPosition())<=5) {
                        mechControl.intakeOn(isDeployed);
                    }
                } else {
                    mechControl.intakeOff(isDeployed);
                }
            } else {
                robot.motorIntake.setPower(robot.INTAKE_REVERSE_POW);
            }

//end of intake controls

//chainsaw control section (GP1, dpad up/down, Triggers)
            if(!gamepad1.dpad_up&&!gamepad1.dpad_down){
                chainsawToggle=true;
            }

            //adjust chainsaw power
            if(gamepad1.dpad_up&&chainsawToggle&&chainsawPower<1){
                chainsawToggle=false;
                chainsawPower+=0.2;
            }else if(gamepad1.dpad_down&&chainsawToggle&&chainsawPower>0.4){
                chainsawToggle=false;
                chainsawPower-=0.2;
            }

            //reset chainsaw power to 1
            if(gamepad1.a){
                chainsawPower=1;
            }

            //apply chainsaw power
            if(gamepad1.right_trigger>0.5){
                robot.motorChainsaw.setPower(chainsawPower);
            }
            else if(gamepad1.left_trigger>0.5){
                robot.motorChainsaw.setPower(-chainsawPower);
            }else{
                robot.motorChainsaw.setPower(0);
            }

//end of chainsaw controls

//arm control section (GP2, X, Dpad Down)
            //allows for toggling between arm positions and not only going to lowest one because of button being held
            if(!gamepad2.x){
                toggleReadyDown=true;
            }
            if(!gamepad2.dpad_up){
                toggleReadyUp=true;
            }
            //end of arm toggle checks

            //adds 1 to bumpCount if x isn't held down
            if(gamepad2.x && toggleReadyDown){
                toggleReadyDown=false;
                if(bumpCount<3) {
                    bumpCount += 1;
                }
            }

            //removes 1 from bumpCount if dpad up isn't held down
            if(gamepad2.dpad_up && toggleReadyUp){
                toggleReadyUp=false;
                if(bumpCount>1) {
                    bumpCount -= 1;
                }
            }
            if(!gamepad2.y){
                TSEtoggle=true;
            }
            if(gamepad2.y&&TSEtoggle&&!isDeployed){
                TSEtoggle=false;
                TSEMode=!TSEMode;
            }

            //counts how many times x has been pressed (what position to go to to score)
            if(bumpCount>0){
                isDeployed=true;
                mechControl.resetIntake();
                intakeDown=false;
            }

            if(TSEMode) {
                //mode to pick up TSE
                if (bumpCount == 1) {
                    mechControl.TSEDown();
                } else if (bumpCount == 2) {
                    mechControl.TSEresting();
                } else if (bumpCount == 3) {
                    mechControl.TSEtop();
                }
            }else{
                //move arm to score
                if (bumpCount == 1) {
                    mechControl.scoringPos1();
                } else if (bumpCount == 2) {
                    mechControl.scoringPos2();
                } else if (bumpCount == 3) {
                    mechControl.scoringPos3();
                }
            }
            //reset arm to zero with or without scoring
            if (gamepad2.dpad_left) {
                bumpCount = 0;
                isDeployed = false;
                mechControl.moveToZero();
                turretPreset=0;
            }
//end of arm controls

//turret control section (GP2, left stick)
            if(!intakeDown){
                if(!gamepad2.left_bumper&&!gamepad2.right_bumper){
                    turretToggle=true;
                }
                if(turretToggle&&gamepad2.left_bumper&&turretPreset<robot.TURRET_INCREMENTS){
                    turretToggle=false;
                    turretPreset++;
                }else if(turretToggle&&gamepad2.right_bumper&&turretPreset>-robot.TURRET_INCREMENTS){
                    turretToggle=false;
                    turretPreset--;
                }
                if(gamepad2.right_stick_button){
                    turretPreset=0;
                }
            }
            turretPosition=turretPreset*robot.TURRET_STEP;
            //apply angle to turret PID
            if(turretPosition>robot.TURRET_MAX_POSITION){
                turretPosition=robot.TURRET_MAX_POSITION;
            }else if(turretPosition<-robot.TURRET_MAX_POSITION){
                turretPosition=-robot.TURRET_MAX_POSITION;
            }
            turretControl.setTargetPosition(turretPosition);
//end of turret control section

//bucket control section (GP2, Dpad Right)
            if(gamepad2.dpad_right&&!TSEMode){
                if(bumpCount==1){
                    bucketAngle=0.15;
                }else if(bumpCount==2){
                    bucketAngle=0.05;
                }else {
                    bucketAngle = 0.25;
                }
            }else if(gamepad2.dpad_right&&bumpCount==3&&TSEMode) {
                bucketAngle = 0;
            }else{
                if(intakeDown){
                    bucketAngle=0.4;
                }else if(bumpCount==1&&!TSEMode&&robot.motorArmAngle1.getCurrentPosition()<750) {
                    bucketAngle = 0.6;
                }else if(bumpCount==2&&!TSEMode){
                    bucketAngle=0.55;
                }else if(bumpCount==3&&!TSEMode) {
                    bucketAngle = 0.75;
                }else if(bumpCount==1&&TSEMode){
                    bucketAngle=0.4;
                }else if(bumpCount==3&&TSEMode){
                    bucketAngle=0.7;
                }else{
                    if(robot.motorArmAngle1.getCurrentPosition()<500) {
                        bucketAngle = 0.5;
                    }
                }
            }
            robot.bucketDump.setPosition(bucketAngle);
//end of bucket controls

            telemetry.addData("TSE MODE: ",TSEMode);
            telemetry.addData("","");
            telemetry.addData("Last Intake Servo Pos",robot.bucketDump.getPosition());
            telemetry.addData("Turret Current Angle: ",robot.turrentEncoder.getCurrentPosition());
            telemetry.addData("Turret Target Angle: ",turretPosition);
            telemetry.addData("Turret Preset: ",turretPreset);
            telemetry.addData("Left Power: ",left);
            telemetry.addData("Right Power: ",right);
            telemetry.addData("Chainsaw Power: ",chainsawPower);
            telemetry.addData("Happy Driving ",")");
            telemetry.update();
        }   // end of while opModeIsActive()
        //stops mechanism thread
        mechControl.stop();
        turretControl.stop();
    }   // end of runOpMode method

}   // end of TeleOp.java class