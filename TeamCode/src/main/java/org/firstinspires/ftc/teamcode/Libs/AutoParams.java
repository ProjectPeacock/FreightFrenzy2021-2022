package org.firstinspires.ftc.teamcode.Libs;

public class AutoParams {

    public double forwardSpeed = 0;
    public double forwardDistance = 0;
    public double hubDistance1 = 0;
    public double hubDistance2 = 0;
    public double hubDistance3 = 0;
    public double TSEreturnDist = 0;
    public double bucketAngle1= 0;
    public double bucketAngle2= 0;
    public double bucketAngle3= 0;

    public double hubDistance = 10.0;
    public double turnAngle = 65;
    //                TSEturnAngle = 20;
    //                TSEturnFactor = 1;
    public double parkDistance = 35;
    public double warehouseParkDistance = 100;
    public double turnError = 0.75;
    public double bucketAngle = 0.3;


    public void AutoParams(String alliance, String side){
        // alliance options are "Red" or "Blue"
        // side options are "Carousel" or "Warehouse"

    }   // end AutoParams constructor

    private void redCarousel(){
        this.forwardDistance = 0.0;
        this.hubDistance1 = 14;
        this.hubDistance2 = 14;
        this.hubDistance3 = 14;
        this.TSEreturnDist = 0;
        this.bucketAngle1 =-0.55;
        this.bucketAngle2 =-0.55;
        this.bucketAngle3 =-0.55;

    }   // end method redCarousel

    private void redWarehouse() {

    }   // end method redWarehouse

    private void blueCarousel(){
        forwardSpeed = -0.32;
        forwardDistance = 20.0;
        hubDistance = 10.0;
        turnAngle = 65;
        TSEreturnDist = 16;
        //                TSEturnAngle = 20;
        //                TSEturnFactor = 1;
        parkDistance = 35;
        warehouseParkDistance = 100;
        turnError = 0.75;
        bucketAngle = 0.3;
    }   // end method blueCarousel

    private void blueWarehouse() {

    }   // end method blueWarehouse

}   // end AutoParams class
