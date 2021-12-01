package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

public class ArmControlLibrary implements Runnable{
    public HardwareProfile localRobot=null;
    private int angle1=0;
    private int angle2=0;
    private double arm1Power=1;
    private double arm2Power=1;
    private int sleepTime;
    private boolean isRunning=true;
    public ArmControlLibrary(HardwareProfile robotIn, int threadSleepDelay){
        this.localRobot=robotIn;
        this.sleepTime=threadSleepDelay;
    }
    public void setPower(double a, double b){
        arm1Power=a;
        arm2Power=b;
    }
    public void intakeOn(boolean deployed){
        if(!deployed) {
            localRobot.motorIntake.setPower(localRobot.INTAKE_POW);
            localRobot.intakeDeployBlue.setPosition(localRobot.BLUE_ZERO - localRobot.INTAKE_DEPLOY_BLUE);
            localRobot.intakeDeployPink.setPosition(localRobot.PINK_ZERO + localRobot.INTAKE_DEPLOY_PINK);
            localRobot.intakeTilt.setPosition(localRobot.INTAKE_TILT_INPUT);
            angle1=localRobot.ARM_1_INTAKE;
            angle2=localRobot.ARM_2_INTAKE;
        }
    }
    //intake controls
    public void intakeOff(boolean deployed){
        localRobot.motorIntake.setPower(0);
        if(!deployed){
            angle1=0;
            angle2=0;
        }
        if(localRobot.motorArmAngle1.getCurrentPosition()<900){
            localRobot.intakeDeployBlue.setPosition(localRobot.BLUE_ZERO);
            localRobot.intakeDeployPink.setPosition(localRobot.PINK_ZERO);
            localRobot.intakeTilt.setPosition(localRobot.INTAKE_STARTING_POS);
        }
    }
    //high platform scoring, default
    public void scoringPos1(){
        angle1=-300;
        while(localRobot.motorArmAngle1.getCurrentPosition()>750){

        }
        angle2=localRobot.HIGH_PLATFORM;
    }
    //mid platform scoring
    public void scoringPos2(){
        angle1=-300;
        angle2=localRobot.MID_PLATFORM;
    }
    //low platform scoring
    public void scoringPos3(){
        angle1=-1000;
        angle2=localRobot.LOW_PLATFORM;
    }
    //moves arm to new zero (through the bottom)
    public void resetArm(){
        angle1=0;
        angle2=2786; //moves arm 2 to one full rotation
        while(localRobot.motorArmAngle2.getCurrentPosition()!=angle2){

        }
        localRobot.motorArmAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        localRobot.motorArmAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetIntake(){
        localRobot.motorIntake.setPower(0);
        if(localRobot.motorArmAngle1.getCurrentPosition()<900){
            localRobot.intakeDeployBlue.setPosition(localRobot.BLUE_ZERO);
            localRobot.intakeDeployPink.setPosition(localRobot.PINK_ZERO);
            localRobot.intakeTilt.setPosition(localRobot.INTAKE_STARTING_POS);
        }
    }
    //moves arm to 0 (over the top)
    public void moveToZero(){
        angle1=0;
        angle2=0;
    }
    //stops thread
    public void stop(){
        isRunning=false;
    }
    //return motorArmAngle1 position for auto
    public int getEncoderAngle1(){
        return localRobot.motorArmAngle1.getCurrentPosition();
    }
    //return motorArmAngle2 position for auto
    public int getEncoderAngle2(){
        return localRobot.motorArmAngle2.getCurrentPosition();
    }
    //set custom position for motors
    public void setAutoPosition(int a, int b){
        localRobot.motorArmAngle1.setTargetPosition(a);
        localRobot.motorArmAngle2.setTargetPosition(b);
    }

    public void incrementUp(){
        angle2+=5;
    }
    public void incrementDown(){
        angle2-=5;
    }
    //method that runs whenever thread is running
    public void activeArmControl(){
        localRobot.motorArmAngle1.setPower(arm1Power);
        localRobot.motorArmAngle2.setPower(arm2Power);
        localRobot.motorArmAngle1.setTargetPosition(angle1);
        localRobot.motorArmAngle2.setTargetPosition(angle2);
    }

    @Override
    public void run() {
        while(isRunning){
            activeArmControl();
            try{
                Thread.sleep(sleepTime);
            } catch(InterruptedException e){
                e.printStackTrace();
            }
        }
    }
}
