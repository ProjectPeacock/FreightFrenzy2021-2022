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

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.vuforia.State;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;
import org.firstinspires.ftc.teamcode.Threads.MechControlLibrary;


@Autonomous(name="AutoParkDriveClassJG", group="Competition")
public class AutoParkDriveClassJG extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareProfile robot   = new HardwareProfile();
    private LinearOpMode opMode = this;
    private ElapsedTime   runtime = new ElapsedTime();
    private State setupState = State.ALLIANCE_SELECT;

    @Override
    public void runOpMode() {
        // arm control thread
        MechControlLibrary mechControl = new MechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread armController = new Thread(mechControl);

        boolean autoReady = false;
        double startTime;
        String alliance = "";
        String startPosition = "";
        String goalPosition = "";
        long startDelay = 0;
        double timeElapsed;
        int positionFactor = 1;
        int scorePosition=3;
        double forwardSpeed = -0.4;
        double goalDistance = 26.0;
        double turnDistance = 2.0;
        double turnAngle = 60;
        double parkDistance = 26.0;
        double parkAdjust = 24.0;
        double turnError = 0.5;
        double bucketAngle=0.75;

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        DriveClass drive = new DriveClass(robot, opMode);

        timeElapsed = runtime.time();   // initialize timeElapsed to confirm button press time
        while (!autoReady){

            switch (setupState) {
                case ALLIANCE_SELECT:
                    telemetry.addData("Which Alliance are you on?", "");
                    telemetry.addData("Press X  == ", " BLUE Alliance");
                    telemetry.addData("Press Y  == ", " RED Alliance");
                    telemetry.update();

                    if(gamepad1.x || gamepad1.y){
                        if (gamepad1.x){
                            alliance = "BLUE";
                            positionFactor = 1;
                            robot.LEDPort.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        }  else {
                            alliance = "RED";
                            positionFactor = -1;
                            robot.LEDPort.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        }
                        setupState = State.DELAY_LENGTH;    // give option to add delay
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
                    telemetry.addData("Press DPAD Left  == ", " WALL Side");
                    telemetry.addData("Press DPAD Right == ", " FIELD Side");
                    telemetry.update();

                    if(gamepad1.dpad_left || gamepad1.dpad_right){
                        if (gamepad1.dpad_left){
                            startPosition = "WALL";
                            if(alliance.equals("BLUE")){
                                robot.LEDPort.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
                            } else robot.LEDPort.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
                            sleep(1000);
                        }  else startPosition = "FIELD";
                        positionFactor = positionFactor * -1;
                        setupState = State.VERIFY_CONFIG;
                    }   // end of if(gamepad1.dpad_left...
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
                    telemetry.addData("Goal on ==  ", goalPosition);
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

        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);

        waitForStart();
        startTime = runtime.time() + startDelay;

        // start arm controller thread
        armController.start();

        // sleep for set delay time
        sleep(startDelay);
        timeElapsed = runtime.time() - startTime;

        while (timeElapsed < 30) {

            timeElapsed = runtime.time() - startTime;

            // Step 1
            //      drive.driveStraight(backwards to goal
            drive.driveStraight(forwardSpeed, goalDistance);

            //   drive.driveTurn(turn towards goal);
            drive.driveTurn(turnAngle * positionFactor, turnError);

            //pause for 1 second
            sleep(1000);

            //  drive towards goal
            drive.driveStraight(forwardSpeed, turnDistance);

            // score element in high goal
            //move arm to scoring positions. initailly always go high, later will use
            // recognition to set scorePosition
            if (scorePosition == 1) {
                mechControl.scoringPos1();
            } else if (scorePosition == 2) {
                mechControl.scoringPos2();
            } else if (scorePosition == 3) {
                mechControl.scoringPos3();
            }

            // dump bucket
            robot.bucketDump.setPosition(bucketAngle);

            //reset arms
            mechControl.resetArm();

            // reverse direction to drive forward to park
            forwardSpeed = forwardSpeed * -1;

            //back up turn amount
            drive.driveStraight(forwardSpeed, turnDistance);

            // reposition angle to park
            turnAngle = 90 - turnAngle;
            drive.driveTurn(turnAngle * positionFactor, turnError);

            // adjust park distance if needed
            if (startPosition.equals("FIELD")) {
                parkDistance = parkDistance + parkAdjust;
            }
            drive.driveStraight(forwardSpeed, parkDistance);

            // Step 2 - just stop for now
        }
        drive.motorsHalt();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

        //stops mechanism thread
        mechControl.stop();

    } // end of opmode

    /*
     * Enumerate the states of the machine
     */
    enum State {
        ALLIANCE_SELECT, POSITION_SELECT, VERIFY_CONFIG, DELAY_LENGTH,
        DEPOT_PARK, FIELD_PARK, HALT
    }   // end of enum State
}
