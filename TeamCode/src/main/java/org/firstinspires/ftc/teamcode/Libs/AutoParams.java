package org.firstinspires.ftc.teamcode.Libs;

public class AutoParams {

    public double forwardSpeed = 0;
    public double forwardDistance = 0;
    public double hubDistance1 = 0;
    public double hubDistance2 = 0;
    public double hubDistance3 = 0;
    public double tseDistance = 0;
    public double TSEreturnDist = 0;
    public double bucketAngle1= 0;
    public double bucketAngle2= 0;
    public double bucketAngle3= 0;

    public double hubDistance = 10.0;
    public double turnAngle = 65;
    public double parkDistance = 35;
    public double warehouseParkDistance = 100;
    public double turnError = 2;
    public double bucketAngle = 0.3;

    public AutoParams(){

    }
    public void initParams(boolean blueAlliance, boolean warehouseSide){

        if(blueAlliance){
            if (warehouseSide){
                blueWarehouse();
            } else {
                blueCarousel();
            }
        } else {
            if(warehouseSide){
                redWarehouse();
            } else {
                redCarousel();
            }
        }
    }   // end AutoParams constructor

    private void redCarousel(){
        this.forwardSpeed = -0.32;
        this.forwardDistance = 20.0;
        this.hubDistance1 = 5;
        this.hubDistance2 = 6;
        this.hubDistance3 = 7;
        this.bucketAngle1 = 0.3;
        this.bucketAngle2 = 0.3;
        this.bucketAngle3 = 0.3;
        this.tseDistance = 34;
        this.TSEreturnDist = 14;

        this.turnAngle = 50;
        this.parkDistance = 35;
        this.warehouseParkDistance = 100;
        this.turnError = 2;
        this.bucketAngle = 0.3;

    }   // end method redCarousel

    private void redWarehouse() {
        this.forwardDistance = 0.0;
        this.hubDistance1 = 14;
        this.hubDistance2 = 14;
        this.hubDistance3 = 14;
        this.bucketAngle1 =-0.55;
        this.bucketAngle2 =-0.55;
        this.bucketAngle3 =-0.55;
        this.tseDistance = 34;
        this.TSEreturnDist = 14;

        this.forwardSpeed = 0;
        this.turnAngle = 65;
        this.parkDistance = 35;
        this.warehouseParkDistance = 100;
        this.turnError = 2;
        this.bucketAngle = 0.3;
    }   // end method redWarehouse

    private void blueCarousel(){
        this.forwardSpeed = -0.32;
        this.forwardDistance = 20.0;
        this.hubDistance1 = 5;
        this.hubDistance2 = 6;
        this.hubDistance3 = 7;
        this.bucketAngle1 = 0.3;
        this.bucketAngle2 = 0.3;
        this.bucketAngle3 = 0.3;
        this.tseDistance = 34;
        this.TSEreturnDist = 14;

        this.turnAngle = -50;
        this.parkDistance = 35;
        this.warehouseParkDistance = 100;
        this.turnError = 2;
        this.bucketAngle = 0.3;
    }   // end method blueCarousel

    private void blueWarehouse() {
        this.forwardDistance = 0.0;
        this.hubDistance1 = 14;
        this.hubDistance2 = 14;
        this.hubDistance3 = 14;
        this.bucketAngle1 =-0.55;
        this.bucketAngle2 =-0.55;
        this.bucketAngle3 =-0.55;
        this.tseDistance = 34;
        this.TSEreturnDist = 14;

        this.forwardSpeed = 0;
        this.turnAngle = 65;
        this.parkDistance = 35;
        this.warehouseParkDistance = 100;
        this.turnError = 2;
        this.bucketAngle = 0.3;
    }   // end method blueWarehouse

}   // end AutoParams class
