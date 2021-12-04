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


@Autonomous(name="AutoParkDriveClassJG", group="Competition")
public class AutoParkDriveClassJG extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareProfile robot   = new HardwareProfile();
    private LinearOpMode opMode = this;
    private ElapsedTime   runtime = new ElapsedTime();
    private State setupState = State.ALLIANCE_SELECT;

    @Override
    public void runOpMode() {
        boolean autoReady = false;
        String alliance = "";
        String startPosition = "";
        long startDelay = 0;
        double timeElapsed;
        long positionFactor = 1;
        double forwardSpeed = -0.4;
        double forwardDistance = 26.0;
        double turnAngle = 90;

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
                    telemetry.addData("Verify the setup", "");
                    telemetry.addData("Alliance          == ", alliance);
                    telemetry.addData("Start Delay == ", startDelay);
                    telemetry.addData("Starting Position ==  ", startPosition);
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
        sleep(startDelay);

        // Step 1
        //      drive.driveStraight(backwards to goal
        drive.driveStraight(forwardSpeed, forwardDistance);
        
        //   drive.driveTurn(-90, 0.5);
        drive.driveTurn(turnAngle * positionFactor, 0.5);

        // drive forward to park
        forwardSpeed = forwardSpeed * -1;
        forwardDistance = 26;
        if (startPosition.equals("FIELD")){
            forwardDistance = forwardDistance + 24;
        }
        drive.driveStraight(forwardSpeed, forwardDistance);

        telemetry.addData("motor L1 = ", robot.motorL1.getCurrentPosition());
        telemetry.addData("motor L2 = ", robot.motorL2.getCurrentPosition());
        telemetry.addData("motor R1 = ", robot.motorR1.getCurrentPosition());
        telemetry.addData("motor R2 = ", robot.motorR2.getCurrentPosition());
        telemetry.update();


        // Step 2

        drive.motorsHalt();

     //   telemetry.addData("Path", "Complete");
     //   telemetry.update();
        sleep(1000);
    }

    /*
     * Enumerate the states of the machine
     */
    enum State {
        ALLIANCE_SELECT, POSITION_SELECT, VERIFY_CONFIG, DELAY_LENGTH,
        DEPOT_PARK, FIELD_PARK, HALT
    }   // end of enum State
}
