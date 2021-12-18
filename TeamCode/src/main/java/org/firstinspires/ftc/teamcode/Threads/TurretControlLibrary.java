// change comments

/*
  12/09/21 added rotateTurret & resetTurret method stubs
 */
package org.firstinspires.ftc.teamcode.Threads;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

public class TurretControlLibrary implements Runnable{
    //takes in HardwareProfile to be able to access motors & servos
    public HardwareProfile localRobot=null;
    private ElapsedTime runtime = new ElapsedTime();

    private int angle1=0;
    private int angle2=0;
    private double arm1Power=1;
    private double arm2Power=1;
    private int sleepTime;
    private boolean isRunning=true;
    private double turretPower = 0.02;
    private int turretTargetPosition = 0;
    private int turretThreshold = 5;
    private int turretLimit = 330;
    private double timeElapsed;
    private int turretServoPower=0;

    //constructor
    public TurretControlLibrary(HardwareProfile robotIn, int threadSleepDelay){
        this.localRobot=robotIn;
        this.sleepTime=threadSleepDelay;
    }

// rotate turret
    public void rotateTurret(double power){
        // ensure turret within limit
        if (Math.abs(localRobot.turrentEncoder.getCurrentPosition())<turretLimit){
            localRobot.turretServoBlue.setPower(power);
            localRobot.turretServoPink.setPower(power);
        }
    }   // end of rotateTurret

    // reset turret - returns turret to 'home' position
    public void resetTurret(){
        turretControl(0);
    }   // end of resetTurret

    public void positionTurret(double targetPosition){
        turretControl(targetPosition);
    }   // end method positionTurret

    public int currentTurretPosition(){
        return localRobot.motorIntake.getCurrentPosition();
    }

    private void turretControl(double targetPosition){
        double integral = 0;
        double Cp = 0.06;
        double Ci = 0.0003;
        double Cd = 0.0001;
        double maxSpeed = 0.5;
        double rotationSpeed;
        double derivative = 0, lastError = 0;

        double error = targetPosition - currentTurretPosition();

        // nested while loops are used to allow for a final check of an overshoot situation
        while (Math.abs(error) >= targetPosition) {
            derivative = lastError - error;
            rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative)) * maxSpeed;

            // Clip servo speed
            rotationSpeed = Range.clip(rotationSpeed, -maxSpeed, maxSpeed);

            // make sure the servo speed doesn't drop to a level where it is no longer able
            // to rotate
            if ((rotationSpeed > -0.05) && (rotationSpeed < 0)) {
                rotationSpeed = -0.05;
            } else if ((rotationSpeed < 0.05) && (rotationSpeed > 0)) {
                rotationSpeed = 0.05;
            }

            lastError = error;

            error = targetPosition - currentTurretPosition();
        }   // end of while Math.abs(error)

        setTurretRotation(0);       // stop the turrets

    }   // end of turretControl() method

    public void setTurretRotation(double rotationSpeed){
        localRobot.turretServoBlue.setPower(rotationSpeed);
        localRobot.turretServoPink.setPower(rotationSpeed);
    }
//method that runs whenever thread is running
    public void activeTurretControl(){
    }
//end of default running method

    //thread stop method
    public void stop(){
        isRunning=false;
    }

//thread run method
    @Override
    public void run() {
        while(isRunning){
            activeTurretControl();
            try{
                sleep(sleepTime);
            } catch(InterruptedException e){
                e.printStackTrace();
            }   // end of try{
        }   // end of while(isRunning)
    }   // end of run() method
}   // end of TurretControlLibrary Class
//end of thread run method