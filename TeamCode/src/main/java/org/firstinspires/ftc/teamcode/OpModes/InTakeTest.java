package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.ArmControlCLass;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;
import org.firstinspires.ftc.teamcode.Threads.MechControlLibrary;
import org.firstinspires.ftc.teamcode.Threads.TurretControlThread;

@Autonomous(name = "MOTOR INTAKE Test", group = "Competition")
//  @Disabled

public class InTakeTest extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public InTakeTest(){

    }   // end of BrokenBotTS constructor

    public void runOpMode(){

        double forwardSpeed = -0.32;
        MechControlLibrary mechControl = new MechControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread mechController = new Thread(mechControl);
        TurretControlThread turretControl = new TurretControlThread(robot, robot.ARM_THREAD_SLEEP);
        Thread turretController = new Thread(turretControl);
        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);
/**
        robot.turrentEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_TILT_INPUT);
        robot.bucketDump.setPosition(.5);

        DriveClass drive = new DriveClass(robot, opMode);
        // arm control
        ArmControlCLass armControl = new ArmControlCLass(robot, robot.ARM_THREAD_SLEEP);
**/

        waitForStart();

        while(opModeIsActive()) {

            robot.motorIntake.setPower(1);

        }   // end of while opModeIsActive()
        //stops mechanism thread
        mechControl.stop();
        turretControl.stop();
    }   // end of runOpMode method

}   // end of TeleOp.java class


