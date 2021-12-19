//  change comments
/*
    IMPORTANT NOTE: This program is strictly to be used to refine the turret control library and
    for general turret thread testing.
 */
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;
import org.firstinspires.ftc.teamcode.Threads.TurretControlLibrary;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Turret testing & debugging", group = "Dev")

public class TurretTestCTSDELETE extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public TurretTestCTSDELETE(){

    }   // end of BrokenBotTS constructor

    public void runOpMode(){
        TurretControlLibrary turretControl = new TurretControlLibrary(robot, robot.ARM_THREAD_SLEEP);
        Thread turretController = new Thread(turretControl);
        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);

        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();
        int armPositionCounter =0;
        boolean toggleReadyDown=false;
        boolean toggleReadyUp=false;
        boolean isDeployed=false;
        boolean intakeDown=false;
        boolean toggleIntake=false;
        double currentTurretPower = 0.0;
        int turretPosition = 0;

        waitForStart();
        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
        robot.intakeTilt.setPosition(robot.INTAKE_STARTING_POS);
//        turretController.start();

        while(opModeIsActive()) {
            /*
            DRIVE CONTROLS:
            Dpad up - reset turret
            Dpad down - reset turret
            Dpad right - turret to right side of the robot
            Dpad Left - turret to the left side of the robot
            */

            // reset the position of the turret to 0 position
            if(gamepad1.dpad_down || gamepad2.dpad_down || gamepad1.dpad_up || gamepad2.dpad_up){
                turretPosition = 0;
            }

            // move the turret to the right side of the robot using preset values
            if(gamepad1.dpad_right || gamepad2.dpad_right){
                turretPosition = robot.TURRET_RIGHT_POSITION;
            }

            // move the turret to the left side of the robot using preset values
            if(gamepad1.dpad_left || gamepad2.dpad_left){
                turretPosition = robot.TURRET_LEFT_POSITION;
            }

            // manually control the rotation of the turret
            if (gamepad2.right_trigger > 0) {
//                turretPosition = turretControl.currentTurretPosition() + 1;
                if (turretPosition > robot.TURRET_RIGHT_POSITION)
                    turretPosition = robot.TURRET_RIGHT_POSITION;
            } else if (gamepad2.left_trigger > 0) {
//                turretPosition = turretControl.currentTurretPosition() - 1;
                if (turretPosition < robot.TURRET_LEFT_POSITION)
                    turretPosition = robot.TURRET_LEFT_POSITION;
            }

            if(gamepad2.right_stick_x != 0){
//                turretControl.setTurretRotation(gamepad2.right_stick_x);
                robot.turretServoBlue.setPower(gamepad2.right_stick_x/10);
                robot.turretServoPink.setPower(gamepad2.right_stick_x/10);
            } else {
                robot.turretServoBlue.setPower(0);
                robot.turretServoPink.setPower(0);
            }

            // apply the changes to the turret
 //           turretControl.setTargetPosition(turretPosition);

            /**
             * #################################################################################
             * #################################################################################
             * #################      PROVIDE USER FEEDBACK    #################################
             * #################################################################################
             * #################################################################################
             */

//            telemetry.addData("Current Turret Encoder = ", turretControl.currentTurretPosition());
            telemetry.addData("Turret Encoder = ", robot.motorIntake.getCurrentPosition());
            telemetry.addData("Target Turret Encoder = ", turretPosition);
            telemetry.update();

        }   // end of while opModeIsActive()

        //stops mechanism thread
//        turretControl.stop();

    }   // end of runOpMode method

}   // end of TeleOp.java class


