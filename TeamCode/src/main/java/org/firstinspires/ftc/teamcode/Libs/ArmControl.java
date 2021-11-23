package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

public class ArmControl {

    private HardwareProfile robot;
    public LinearOpMode opMode;

    /**
     * Arm Control Constructor
     */
    public ArmControl(HardwareProfile myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;
    }  // closes ArmControl constructor Method


    /**
     * Method: moveToPosition
     * Parameters:
     * @param targetMotor       -> Motor to apply motion to
     * @param targetPosition    -> Position to move the motor to
     * @param armPower          -> Power to use to move motor => + or - changes direction
     */
    public void moveToPosition(DcMotorEx targetMotor, int targetPosition, double armPower) {
        double integral = 0;
        double error;
        double Cp = 0.06;
        double Ci = 0.0003;
        double Cd = 0.0001;
        double maxPower = 0.5;
        double rotationSpeed;
        double derivative = 0, deltaError, lastError=0;
        double currentPosition = targetMotor.getCurrentPosition();

        // check to see...
        error = currentPosition - targetPosition;

        // nested while loops are used to allow for a final check of an overshoot situation
        while ((Math.abs(error) >= 0) && opMode.opModeIsActive()) {
            deltaError = lastError - error;

            rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative)) * armPower;

            // Clip motor speed
            rotationSpeed = Range.clip(rotationSpeed, -maxPower, maxPower);

            targetMotor.setPower(rotationSpeed);

            lastError = error;

            opMode.telemetry.addData("targetPosition  = ", targetPosition);
            opMode.telemetry.addData("Current Postion  = ", targetMotor.getCurrentPosition());
            opMode.telemetry.addData("Theta/lastError Value= ", lastError);
            opMode.telemetry.addData("CurrentZ/Error Value = ", error);
            opMode.telemetry.addData("zCorrection/derivative Value = ", derivative);
            opMode.telemetry.update();

            error = currentPosition - targetPosition;

        }   // end of while Math.abs(error)

    }   // closes moveToPosition method

}   // closes ArmControl class
