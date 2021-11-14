package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Competition")
//  @Disabled

public class TeleOp extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public TeleOp(){

    }   // end of BrokenBotTS constructor

    public void runOpMode(){



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

        waitForStart();

        while(opModeIsActive()) {

            /*
             * Mecanum Drive Control section
            */
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

            //intake controls (GP1, A button and Y Button)
            if(gamepad1.a) {
                robot.motorIntake.setPower(robot.INTAKE_POW);
                robot.intakeDeployBlue.setPosition(robot.BLUE_DEPLOY);
                robot.intakeDeployPink.setPosition(robot.PINK_DEPLOY);
            }else if(gamepad1.y){
                robot.motorIntake.setPower(robot.INTAKE_REVERSE_POW);
                robot.intakeDeployBlue.setPosition(robot.BLUE_OUTTAKE);
                robot.intakeDeployPink.setPosition(robot.PINK_OUTTAKE);
            }  else {
                robot.motorIntake.setPower(robot.INTAKE_IDLE);
                robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
                robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
            }

            //intake ramp controls (GP1, Dpad)
            if(gamepad1.dpad_left){
                double angle=robot.intakeTilt.getPosition();
                robot.intakeTilt.setPosition(angle+0.05);
            } else if(gamepad1.dpad_right){
                double angle=robot.intakeTilt.getPosition();
                robot.intakeTilt.setPosition(angle-0.05);
            } else{
            }
            
            //chainssaw controls (GP1, Bumpers)
            if(gamepad1.left_bumper){
                robot.motorChainsaw.setPower(robot.CHAIN_POW);
            }else if(gamepad1.right_bumper){
                robot.motorChainsaw.setPower(-robot.CHAIN_POW);
            }else{
                robot.motorChainsaw.setPower(0);
            }
            if(gamepad1.x){
                robot.motorIntake.setPower(0);
            }else {
            }

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
    }   // end of runOpMode method

}   // end of TeleOp.java class


