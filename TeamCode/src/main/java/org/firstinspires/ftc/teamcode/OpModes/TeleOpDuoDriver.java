package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Threads.MechControlLibrary;
import org.firstinspires.ftc.teamcode.Threads.TurretControlLibrary;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp 2 Driver", group = "Competition")
//  @Disabled

public class TeleOpDuoDriver extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public TeleOpDuoDriver(){

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
        boolean chainsawReady=false;
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

//intake control section (GP1, A)
            //allows for intake toggle and not button hold down
            if(!gamepad2.a){
                toggleIntake=true;
            }

            //if intake isn't deployed, deploy it & vice versa

            if(gamepad2.a&&toggleIntake){
                toggleIntake=false;
                intakeDown=!intakeDown;
                isDeployed=false;
                bumpCount=0;
            }
            //check if intake needs to be reversed and then deploy or retract
            if(Math.abs(robot.turrentEncoder.getCurrentPosition())<=turretThreshold) {
                if (!gamepad2.b) {
                    if (intakeDown) {
                        mechControl.intakeOn(isDeployed);
                    } else {
                        mechControl.intakeOff(isDeployed);
                    }
                } else {
                    robot.motorIntake.setPower(robot.INTAKE_REVERSE_POW);
                }
            }
//end of intake controls

//chainsaw control section (GP1, Bumpers, Triggers)
            if(gamepad1.right_bumper){
                robot.motorChainsaw.setPower(0.45);
            }
            else if(gamepad1.left_bumper){
                robot.motorChainsaw.setPower(-0.45);
            }
            else if(gamepad1.right_trigger>0.5){
                robot.motorChainsaw.setPower(1);
            }
            else if(gamepad1.left_trigger>0.5){
                robot.motorChainsaw.setPower(-1);
            }
            else{ robot.motorChainsaw.setPower(0);
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

            //counts how many times x has been pressed (what position to go to to score)
            if(bumpCount>0){
                isDeployed=true;
                mechControl.resetIntake();
                intakeDown=false;
            }

            //move arm to scoring positions
            if(bumpCount==1){
                mechControl.scoringPos1();
            }else if(bumpCount==2){
                mechControl.scoringPos2();
            }else if(bumpCount==3){
                mechControl.scoringPos3();
            }

            //reset arm to zero with or without scoring
            if(gamepad2.dpad_down){
                bumpCount=0;
                isDeployed=false;
                mechControl.moveToZero();
            }else{

            }
//end of arm controls

//turret control section (GP2, left stick)
            if(!intakeDown){
                turretPosition=(int)(gamepad2.left_stick_x*robot.TURRET_MAX_POSITION);
            }else{
                turretPosition=0;
            }
            //apply angle to turret PID
            turretControl.setTargetPosition(turretPosition);

//end of turret control section

//bucket control section (GP2, Dpad Right)
            if(gamepad2.dpad_right){
                bucketAngle=-1;
            } else{
                bucketAngle=0.5;
                if(bumpCount==3){
                    bucketAngle=0.75;
                }
            }
            robot.bucketDump.setPosition(bucketAngle);
//end of bucket controls

            telemetry.addData("Turret Current Angle: ",robot.turrentEncoder.getCurrentPosition());
            telemetry.addData("Turret Target Angle: ",turretPosition);
            telemetry.addData("Turret power blue: ",robot.turretServoBlue.getPower());
            telemetry.addData("Turret power pink: ",robot.turretServoPink.getPower());
            telemetry.addData("Turret direction blue: ",robot.turretServoBlue.getDirection());
            telemetry.addData("Turret direction pink: ",robot.turretServoPink.getDirection());
            telemetry.update();
        }   // end of while opModeIsActive()
        //stops mechanism thread
        mechControl.stop();
        turretControl.stop();
    }   // end of runOpMode method

}   // end of TeleOp.java class


