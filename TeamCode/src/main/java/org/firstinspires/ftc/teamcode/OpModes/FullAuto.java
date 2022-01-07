/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// change comments
/*
12/07/21 added  back code to drive to goal, changed bucket dump setting,
         added SCORING_POSITION state to initialization. use dpad to set level
*/
package org.firstinspires.ftc.teamcode.OpModes;

//
// import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.vuforia.State;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.ArmControlCLass;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;
import org.firstinspires.ftc.teamcode.Threads.MechControlLibrary;


@Autonomous(name="FullAuto", group="Competition")
//@Disabled
public class FullAuto extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareProfile robot   = new HardwareProfile();
    private LinearOpMode opMode = this;
    private ElapsedTime   runtime = new ElapsedTime();
    private State setupState = State.ALLIANCE_SELECT;

    @Override
    public void runOpMode() {

        boolean autoReady = false;
        boolean running = true;
        double startTime;
        String oldalliance = "";
        String startPosition = "";
        String goalPosition = "";
        long startDelay = 0;
        double timeElapsed;
        double positionFactor = 1;
        int scorePosition=1;
        double forwardSpeed = -0.32;

        double forwardDistance = 26.0;
        double hubDistance = 10.0;
        double goalAdjust = 0;

        double turnAngle = 65;
        double parkDistance = 35;

        double turnError = 0.5;
        double bucketAngle = -1.0;

        //red if false, blue if true
        boolean alliance = false;

        //carousel if false, warehouse if true
        boolean position = false;


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_TILT_INPUT);
        robot.bucketDump.setPosition(.5);

        DriveClass drive = new DriveClass(robot, opMode);
        // arm control
        ArmControlCLass armControl = new ArmControlCLass(robot, robot.ARM_THREAD_SLEEP);

        timeElapsed = runtime.time();   // initialize timeElapsed to confirm button press time
        while (!autoReady){

            switch (setupState) {
                case ALLIANCE_SELECT:
                    telemetry.addData("Drive Motor Encoders:",robot.motorR1.getCurrentPosition());
                    telemetry.addData("Which Alliance are you on?", "");
                    telemetry.addData("Press X  == ", " BLUE Alliance");
                    telemetry.addData("Press Y  == ", " RED Alliance");
                    telemetry.update();

                    if(gamepad1.x || gamepad1.y){
                        if (gamepad1.x){
                            alliance = true;
                        }  else {
                            alliance = false;
                        }
                        setupState = State.DELAY_LENGTH;    // give option to add delay
                        telemetry.addData("Goal on ==  ", positionFactor);
                        telemetry.update();
                    }   // end of if(gamepad1.x...
                    break;

                case DELAY_LENGTH:
                    telemetry.addData("Add a Delay", "");
                    telemetry.addData("Press DPAD_UP  ", " To Increase Delay");
                    telemetry.addData("Press DPAD_DOWN ", " To Reduce Delay");
                    telemetry.addData("Current Delay = ", startDelay);

                    telemetry.addData("Press A to continue", "");
                    telemetry.update();

                    if(gamepad1.dpad_up && (runtime.time()-timeElapsed) >0.3) {
                        if(startDelay < 10){       // limit the max delay to 10 seconds
                            startDelay = startDelay + 1;
                        }
                        timeElapsed = runtime.time();
                    } //end of if(gamepad1.dpad_up)

                    if(gamepad1.dpad_down && (runtime.time()-timeElapsed) >0.3) {
                        if(startDelay > 0){       // confirm that the delay is not negative
                            startDelay = startDelay - 1;
                        } else startDelay = 0;
                        timeElapsed = runtime.time();
                    } //end of if(gamepad1.dpad_up)

                    if(gamepad1.a){         // exit the setup
                        setupState = State.POSITION_SELECT;
                        startDelay = startDelay * 1000;
                    }   // end of if(gamepad1.x...
                    break;

                case POSITION_SELECT:
                    telemetry.addData("What is the position of the robot?", "");
                    telemetry.addData("Press DPAD Left  == ", " CAROUSEL Side");
                    telemetry.addData("Press DPAD Right == ", " WAREHOUSE Side");
                    telemetry.update();

                    if(gamepad1.dpad_left || gamepad1.dpad_right){
                        if (gamepad1.dpad_left){
                            position=false;
                            //sleep(1000);
                        }  else {
                            position=true;
                        }
                        setupState = State.SCORING_POSITION;
                    }   // end of if(gamepad1.dpad_left...
                    break;

                case SCORING_POSITION:
                    telemetry.addData("Set Scoring Position", "");
                    telemetry.addData("Press DPAD_UP  ", " To Increase ");
                    telemetry.addData("Press DPAD_DOWN ", " To Decrease");
                    telemetry.addData("Current Scoring Position = ", scorePosition);

                    telemetry.addData("Press A to continue", "");
                    telemetry.update();

                    if(gamepad1.dpad_up && (runtime.time()-timeElapsed) >0.3) {
                        if(scorePosition < 3){       // limit the max delay to 10 seconds
                            scorePosition = scorePosition + 1;
                        }
                        timeElapsed = runtime.time();
                    } //end of if(gamepad1.dpad_up)

                    if(gamepad1.dpad_down && (runtime.time()-timeElapsed) >0.3) {
                        if(scorePosition > 1){       // confirm that the delay is not negative
                            scorePosition = scorePosition - 1;
                        } else scorePosition = 1;
                        timeElapsed = runtime.time();
                    } //end of if(gamepad1.dpad_up)

                    if(gamepad1.a){         // exit the setup
                        setupState = State.VERIFY_CONFIG;
                    }   // end of Scoring_position...
                    break;

                case VERIFY_CONFIG:
                    if (positionFactor==1) {
                        goalPosition="Right";
                    }else{
                        goalPosition="Left";
                    }
                    telemetry.addData("Verify the setup", "");
                    telemetry.addData("Alliance          == ", alliance);
                    telemetry.addData("Start Delay == ", startDelay);
                    telemetry.addData("Starting Position ==  ", startPosition);
                    telemetry.addData("Goal position ==  ", goalPosition);
                    telemetry.addData("Goal on ==  ", positionFactor);
                    telemetry.addData("Scoring Position ==  ", scorePosition);
                    telemetry.addData("","");
                    telemetry.addData("Press A to Confirm or B to start over","");
                    telemetry.update();

                    if(gamepad1.b || gamepad1.a){
                        if (gamepad1.b){
                            autoReady = false;
                            startDelay = 0;     // reset the start delay
                            setupState = State.ALLIANCE_SELECT;
                        }  else {
                            autoReady = true;
                        }
                    }   // end of if(gamepad1.right_bumper...
                    break;
            }   // end of switch(setupState)
        }   // end of while(autoReady)

        //red carousel
        if(!alliance&&!position){
            positionFactor=1;
        //blue carousel
        }else if(alliance&&!position){
            hubDistance+=5;
            positionFactor=-1;
        //red warehouse
        }else if(!alliance&&position){
            forwardDistance=28;
            hubDistance-=6;
            positionFactor=-1;
            parkDistance=40;
        //blue warehouse
        }else{
            forwardDistance=28;
            hubDistance-=6;
            positionFactor=1;
            parkDistance=40;
        }
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        startTime = runtime.time() + startDelay;

        // start arm controller thread
        //    armController.start();

        // sleep for set delay time
        sleep(startDelay);

        while (opModeIsActive() && (running)) {

            // Step 1
            //      drive.driveStraight(backwards to goal
            drive.driveStraight(forwardSpeed, forwardDistance);
            sleep(250);

            //   drive.driveTurn(turn towards goal);
            drive.driveTurn(turnAngle * positionFactor, turnError);

            //move arm to scoring positions
            if(scorePosition==1){
                armControl.scoringPos1();
                goalAdjust = 2;
            }else if(scorePosition==2){
                armControl.scoringPos2();
                goalAdjust = 1;
            }else if(scorePosition==3){
                armControl.scoringPos3();
                goalAdjust = 0;

            }
            //    idle();
            sleep(250);

            //  drive towards goal
            drive.driveStraight(forwardSpeed, hubDistance);
            sleep(500);

            // score element in high goal
            //move arm to scoring positions. initailly always go high, later will use
            // recognition to set scorePosition


            // dump bucket
            robot.bucketDump.setPosition(bucketAngle);
            sleep(500);
            //reset arms
            robot.bucketDump.setPosition(0.5);
            armControl.moveToZero();

            // reverse direction to drive forward to park
            forwardSpeed = forwardSpeed * -1;

            //back up turn amount
            if(!alliance&&position){
                forwardSpeed=1;
            }else if(alliance&&position){
                forwardSpeed=1;
            }
            drive.driveStraight(forwardSpeed, parkDistance);
            sleep(500);

            // reposition angle to park
            turnAngle = turnAngle * -1;
            //  drive.driveTurn(turnAngle * positionFactor, turnError);

            //go to carousel, red
            if(!alliance&&!position){
                drive.driveTurn(0,turnError);
                sleep(350);
                while(robot.frontDistanceSensor.getDistance(DistanceUnit.CM)>30) {
                    drive.setDrivePower(forwardSpeed, forwardSpeed, forwardSpeed, forwardSpeed);
                    robot.motorChainsaw.setPower(-robot.CHAIN_POW*0.75);
                }
                drive.setDrivePower(0.1,0.1,0.1,0.1);
                sleep(2500);
                drive.driveTurn(0,turnError);
                robot.motorChainsaw.setPower(0);
                drive.driveStraight(-forwardSpeed,9);
                sleep(250);
                drive.driveTurn(-90,turnError);
                sleep(7750-startDelay);
                drive.driveStraight(1,100);
            }

            //go to carousel, blue
            if(alliance&&!position){
                drive.driveTurn(0,turnError);
                sleep(350);
                while(robot.frontDistanceSensor.getDistance(DistanceUnit.CM)>30) {
                    drive.setDrivePower(forwardSpeed, forwardSpeed, forwardSpeed, forwardSpeed);
                    robot.motorChainsaw.setPower(robot.CHAIN_POW*0.75);
                }

                drive.setDrivePower(0.1,0.1,0.1,0.1);
                sleep(2500);
                drive.driveTurn(0,turnError);
                robot.motorChainsaw.setPower(0);
                drive.driveStraight(-forwardSpeed,9);
                sleep(250);
                drive.driveTurn(90,turnError);
                sleep(7750-startDelay);
                drive.driveStraight(1,100);
            }

            // Step 2 - just stop for now
            drive.motorsHalt();
            running = false;
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

    } // end of opmode

    /*
     * Enumerate the states of the machine
     */
    enum State {
        ALLIANCE_SELECT, POSITION_SELECT, VERIFY_CONFIG, DELAY_LENGTH,SCORING_POSITION,
        FIELD_PARK, HALT
    }   // end of enum State
}
