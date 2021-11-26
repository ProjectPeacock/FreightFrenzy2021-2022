package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareProfile.HardwareProfile;

public class ArmControlLibrary implements Runnable{
    private HardwareProfile r=null;
    private int angle1=0;
    private int angle2=0;
    public ArmControlLibrary(HardwareProfile robotIn){
        r=robotIn;
    }
    public void setPower(double a, double b){
        r.motorArmAngle1.setPower(a);
        r.motorArmAngle2.setPower(b);
    }
    public void intakeOn(boolean deployed){
        if(!deployed) {
            r.motorIntake.setPower(r.INTAKE_POW);
            r.intakeDeployBlue.setPosition(r.BLUE_ZERO - r.INTAKE_DEPLOY_BLUE);
            r.intakeDeployPink.setPosition(r.PINK_ZERO + r.INTAKE_DEPLOY_PINK);
            r.intakeTilt.setPosition(r.INTAKE_TILT_INPUT);
            r.motorArmAngle1.setTargetPosition(r.ARM_1_INTAKE);
            r.motorArmAngle2.setTargetPosition(r.ARM_2_INTAKE);
        }
    }
    public void intakeOff(boolean deployed){
        r.motorIntake.setPower(0);
        if(!deployed){
            r.motorArmAngle1.setTargetPosition(0);
            r.motorArmAngle2.setTargetPosition(0);
        }
        if(r.motorArmAngle1.getCurrentPosition()<900){
            r.intakeDeployBlue.setPosition(r.BLUE_ZERO);
            r.intakeDeployPink.setPosition(r.PINK_ZERO);
        }
        r.intakeTilt.setPosition(0.6);
    }
    public void scoringPos1(){
        r.motorArmAngle1.setTargetPosition(0);
        while(r.motorArmAngle1.getCurrentPosition()>750){

        }
        r.motorArmAngle2.setTargetPosition(r.HIGH_PLATFORM);
    }
    public void scoringPos2(){
        r.motorArmAngle1.setTargetPosition(0);
        r.motorArmAngle2.setTargetPosition(r.MID_PLATFORM);
    }
    public void scoringPos3(){
        r.motorArmAngle1.setTargetPosition(-500);
        r.motorArmAngle2.setTargetPosition(r.LOW_PLATFORM);
    }
    public void resetArm(){
        r.motorArmAngle1.setTargetPosition(0);
        r.motorArmAngle2.setTargetPosition(2786); //moves arm 2 to one full rotation
        while(r.motorArmAngle2.getCurrentPosition()!=2786){

        }
        r.motorArmAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.motorArmAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveToZero(){
        r.motorArmAngle1.setTargetPosition(0);
        r.motorArmAngle2.setTargetPosition(0);
    }

    @Override
    public void run() {

    }
}
