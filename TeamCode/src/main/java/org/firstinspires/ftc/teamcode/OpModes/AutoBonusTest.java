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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.ArmControlCLass;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;


@Autonomous(name="Test Bonus Auto", group="Competition")
//@Disabled
public class AutoBonusTest extends LinearOpMode {

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

        // warehouse park
        boolean warehousePark = true;
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.turrentEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_TILT_INPUT);
        robot.bucketDump.setPosition(.5);

        DriveClass drive = new DriveClass(robot, opMode);
        // arm control
        ArmControlCLass armControl = new ArmControlCLass(robot, robot.ARM_THREAD_SLEEP);

        timeElapsed = runtime.time();   // initialize timeElapsed to confirm button press time

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        armControl.intakeOn();
        sleep(350);
        robot.motorIntake.setPower(1);
        telemetry.addData("intake power:",robot.motorIntake.getPower());

        while(opModeIsActive() && (running)) {

        }


        if (opModeIsActive() && (running)) {

            // Step 1
            //drive forward and push TSE out of the way
            drive.driveTime(1, 0.65);

            //turn towards scoring elements
            drive.driveTurn(45, turnError);

            armControl.intakeOn();
            sleep(350);
            robot.motorIntake.setPower(1);

            drive.driveTime(0.3, 1);
            robot.motorIntake.setPower(-1);
            drive.driveTime(-0.3, 1);
            robot.motorIntake.setPower(0);

            armControl.intakeOff();

            // turn towards barrier
            drive.driveTurn(0, turnError);

            drive.driveTime(-0.3, 0.5);
            // drive over barrier
            drive.driveTime(-1, 0.65);

            // turn towards hub
            drive.driveTurn(45, turnError);

            // drive towards hub
            drive.driveStraight(0.3, 10);

            //dump bucket
            robot.bucketDump.setPosition(bucketAngle);
            sleep(500);

            //reset arms
            robot.bucketDump.setPosition(0.5);
            armControl.moveToZero();

            // reverse direction to drive forward to park
            forwardSpeed = forwardSpeed * -1;
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
        FIELD_PARK, HALT, SELECT_PARK
    }   // end of enum State
}
