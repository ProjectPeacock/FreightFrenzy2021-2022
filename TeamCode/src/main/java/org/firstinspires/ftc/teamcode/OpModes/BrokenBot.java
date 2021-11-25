package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

    @TeleOp(name = "Broken Bot TS", group = "Troubleshooting")
  //  @Disabled

    public class BrokenBot extends LinearOpMode {

        private final static HardwareProfile robot = new HardwareProfile();
        private LinearOpMode opMode = this;

        public BrokenBot(){

        }   // end of BrokenBotTS constructor

        public void runOpMode(){
            double currentTick, currentTime, currentRPM;
            boolean servoGrabFlag=false, servoKickFlag=false, servoTransferFlag=false, servoIntakeFlag=false, servoRingFlag=false;
            double armPosition = 0.5;


            boolean intake = false,             // tracks whether the user has enabled or disabled the intake system
                    intakeForward = true,       // tracks which direction the intake system should go
                    grabOpen = false,           // tracks whether the grabber should be open or closed
                    intakeDeployed = false;     // tracks whether the intake should be out or in
            ElapsedTime runTime = new ElapsedTime();
            boolean shooter = false;
            double shooterPower = 0.80;
            double buttonPress = 0;
            boolean readyToShoot = false;
            boolean motorIntakeFlag = false;

            /*
             * ServoRing in the up and ready to receive position:   robot.servoRing.setPosition(0.2);
             * ServoRing in the down and holding rings position:    robot.servoRing.setPosition(0.5);
             * servoTransfer up towards shooter position:           robot.servoTransfer.setPosition(0.7);
             * servoTransfer down to receive rings position:        robot.servoTransfer.setPosition(0.46);
             * servoKick up towards shooter position:               robot.servoKick.setPosition(0.43);
             * servoKick down to receive rings position:            robot.servoKick.setPosition(0.68);
             * servoIntake deployed:                                robot.servoIntake.setPosition(0.65);
             * servoIntake stored:                                  robot.servoIntake.setPosition(0.28);
             */

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
            robot.intakeDeployBlue.setPosition(0.5);
            robot.intakeDeployPink.setPosition(0.5);
            robot.intakeTilt.setPosition(0.5);
            robot.bucketDump.setPosition(0.5);

            telemetry.addData("Robot state = ", "INITIALIZED");
            telemetry.update();

            waitForStart();

            while(opModeIsActive()) {
                currentTime = runTime.time();

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

                /**
                 * ##############################################################################
                 * ##############################################################################
                 * ################    GAMEPAD 1 NORMAL CONTROLS   #############################
                 * ##############################################################################
                 * ##############################################################################
                 */


                /**
                 * ##############################################################################
                 * ##############################################################################
                 * ################    GAMEPAD 2 TESTING CONTROLS   #############################
                 * ##############################################################################
                 * ##############################################################################
                 */

                if (gamepad2.dpad_down) {
                    robot.motorL2.setPower(1);
                    telemetry.addData("Motor = ", "MotorL2");
                }

                if (gamepad2.dpad_up) {
                    robot.motorL1.setPower(1);
                    telemetry.addData("Motor = ", "MotorL1");
                }

                if (gamepad2.dpad_right) {
                    robot.motorR2.setPower(1);
                    telemetry.addData("Motor = ", "MotorR2");
                }

                if(gamepad2.dpad_left) {
                    robot.motorR1.setPower(1);
                    telemetry.addData("Motor = ", "MotorR1");
                }

                //intake controls (GP1, A button and Y Button)
                if(gamepad1.a) {
                    robot.motorIntake.setPower(robot.INTAKE_POW);
                    robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO-robot.INTAKE_DEPLOY_BLUE); //subtracting because it needs to rotate counterclockwise
                    robot.intakeDeployPink.setPosition(robot.PINK_ZERO+robot.INTAKE_DEPLOY_PINK); //adding because it needs to rotate clockwise
                }else if(gamepad1.y) {
                    robot.motorIntake.setPower(robot.INTAKE_REVERSE_POW);
                    robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO - robot.INTAKE_OUTTAKE); //counterclockwise
                    robot.intakeDeployPink.setPosition(robot.PINK_ZERO + robot.INTAKE_OUTTAKE); //clockwise
                }else if(gamepad1.x) {
                    robot.motorIntake.setPower(0);
                }else{
                    robot.intakeDeployBlue.setPosition(robot.BLUE_ZERO);
                    robot.intakeDeployPink.setPosition(robot.PINK_ZERO);
                }


                /*
                 * Toggle the intake forward and reverse based on user pressing gamepad1.y
                 */
                if (gamepad2.right_trigger > 0.2 && (currentTime - buttonPress) >= 0.3){
                    motorIntakeFlag = true;
                    intakeForward = true;      // returns opposite of flag setting
                    buttonPress = currentTime;      // update the time that the button was pressed
                }   else if (gamepad2.left_trigger > 0.2 && (currentTime - buttonPress) >= 0.3) {
                    motorIntakeFlag = true;
                    intakeForward = false;
                    buttonPress = currentTime;      // update the time that the button was pressed
                }   // end of if(gamepad2.right_trigger)

                if (gamepad2.left_bumper && (currentTime - buttonPress) >= 0.3){
                    shooter = toggleFlag(shooter);
                }


                if (gamepad2.right_stick_y > 0.2){
                    armPosition = armPosition + 0.05;
                } else if (gamepad2.right_stick_y < -0.2) {
                    armPosition = armPosition - 0.05;
                }



                /**
                 * ##############################################################################
                 * ##############################################################################
                 * #####################    APPLY CONTROL ACTIONS   #############################
                 * ##############################################################################
                 * ##############################################################################
                 */

                /*
                 * Control the shooting motors based on the user button input
                 */



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

        boolean toggleFlag(boolean flag){
            if (flag) return false;
            else return true;
        }   // end of toggleFlag function
    }   // end of BrokenBot.java class


