//  change comments
/*
    IMPORTANT NOTE: This program is strictly to be used to refine the turret control library and
    for general turret thread testing.
 */
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

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
        double powerMultiplier=0.1;
        int turretPosition = 0;
        int lastTurretPosition=0;

        waitForStart();
//        robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
//        robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
//        robot.intakeTilt.setPosition(robot.INTAKE_STARTING_POS);
        turretController.start();

        while(opModeIsActive()) {
            /*
            DRIVE CONTROLS:
            Dpad up - reset turret
            Dpad down - reset turret
            Dpad right - turret to right side of the robot
            Dpad Left - turret to the left side of the robot
            */

            // reset the position of the turret to 0 position
            if (gamepad1.dpad_down || gamepad2.dpad_down || gamepad1.dpad_up || gamepad2.dpad_up) {
                turretPosition=0;
            }

            // move the turret to the right side of the robot using preset values
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                turretPosition = robot.TURRET_RIGHT_POSITION;
            }

            // move the turret to the left side of the robot using preset values
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                turretPosition = robot.TURRET_LEFT_POSITION;
            }

            if(gamepad2.right_stick_x>0){
                turretPosition+=(int)gamepad2.right_stick_x*5;
            }else if(gamepad2.right_stick_x<0){
                turretPosition+=(int)gamepad2.right_stick_x*5;
            }

            if(turretPosition>robot.TURRET_MAX_POSITION) {
                turretPosition = robot.TURRET_MAX_POSITION;
            }else if(turretPosition<-robot.TURRET_MAX_POSITION) {
                turretPosition = -robot.TURRET_MAX_POSITION;
            }

            if(turretPosition!=lastTurretPosition) {
                turretControl.setTargetPosition(turretPosition);
            }

            lastTurretPosition=turretPosition;




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
            robot.bucketDump.setPosition(0.5);
        }   // end of while opModeIsActive()

        //stops mechanism thread
        turretControl.stop();

    }   // end of runOpMode method

    /*
    private void turretControlTest(int targetPosition){
        double integral = 0;
        double Cp = 0.0012;
        double Ci = 0.002;
        double Cd = 0.007;
        double maxSpeed = 1;
        double minSpeed =0.065;
        double rotationSpeed;
        double derivative = 0, lastError = 0;

        double error = targetPosition - robot.motorIntake.getCurrentPosition();

        // limit the turn position of the turret to the TURRET_MAX_POSITION to avoid
        // damaging the robot
        if(targetPosition > robot.TURRET_MAX_POSITION){
            targetPosition = robot.TURRET_MAX_POSITION;
        } else if(targetPosition < -robot.TURRET_MAX_POSITION){
            targetPosition = -robot.TURRET_MAX_POSITION;
        }

        // nested while loops are used to allow for a final check of an overshoot situation
        while ((Math.abs(error) > 2) && opModeIsActive()) {
            derivative = lastError - error;
            rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative)) * maxSpeed;

            // Clip servo speed
            rotationSpeed = Range.clip(rotationSpeed, -maxSpeed, maxSpeed);

            // make sure the servo speed doesn't drop to a level where it is no longer able
            // to rotate
            if ((rotationSpeed < 0) && (rotationSpeed > -minSpeed)) {
                rotationSpeed = -minSpeed;
            } else if ((rotationSpeed > 0) && (rotationSpeed < minSpeed)) {
                rotationSpeed = minSpeed;
            }

            setTurretRotation(rotationSpeed);
            lastError = error;

            error = targetPosition - robot.motorIntake.getCurrentPosition();

            telemetry.addData("rotationSpeed = ", rotationSpeed);
            telemetry.addData("Calculated error = ", error);
            telemetry.addData("Turret Encoder = ", robot.motorIntake.getCurrentPosition());
            telemetry.addData("Target Turret Position = ", targetPosition);
            telemetry.update();
        }   // end of while Math.abs(error)

        setTurretRotation(0);       // stop the turrets
    }   // end of turretControl() method

    public void setTurretRotation(double rotationSpeed){
        robot.turretServoBlue.setPower(-rotationSpeed);
        robot.turretServoPink.setPower(-rotationSpeed);
    }
*/
}   // end of TeleOp.java class


