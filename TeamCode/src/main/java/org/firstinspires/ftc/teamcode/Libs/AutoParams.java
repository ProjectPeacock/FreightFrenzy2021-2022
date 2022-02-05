package org.firstinspires.ftc.teamcode.Libs;

public class AutoParams {

    public double forwardDistance = 0;
    public double hubDistance1 = 0;
    public double hubDistance2 = 0;
    public double hubDistance3 = 0;
    public double TSEreturnDist = 0;
    public double bucketAngle1= 0;
    public double bucketAngle2= 0;
    public double bucketAngle3= 0;

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

    }   // end method blueCarousel

    private void blueWarehouse() {

    }   // end method blueWarehouse

}   // end AutoParams class
