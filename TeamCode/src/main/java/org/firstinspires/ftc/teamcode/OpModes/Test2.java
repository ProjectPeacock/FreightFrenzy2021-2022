package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Roadrunner_QuickStart.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous (name = "Roadrunner Blue Freight", group = "Autonomous Main")
public class Blue_Freight extends LinearOpMode {

    private Servo Arme = null;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AXhYkLf/////AAABmR7ld1TAyUtVj27BNC2kgi5v8M6kdi515bANfa7qogr3SfVwQcXRTNq/n76Je310IFU1Uq9kjtQ3f7FduLg62GljMRado4kF5BtOKg+L6lVhwqaFX/jDZVl6bsxbi3SIOZw0QDbiw1CZDmcdZWX695/fN8Bltc1JiD+ww6+1nIme4cdtIXY435E0OpHyd2KKG4DKXzj9dN/+XysQwR71atOSwASr5gj2SUaQJDbk5+4nWb5CyFh1DWtdvLIZSsRkg38Ta/A5ZT8bhoUHOayEAA2+nrkr0wR/P1VYDBTBf8/oepmsuW++VV+E3tI2wQPUQ9JuXfJTy5vc/MLVnT3QJBgE+HoW9pwbvgQznpJ2KpLi";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * Variables used for switching cameras.
     */
    private WebcamName webcam1;
    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    Hardware4 robot = new Hardware4();

    @Override
    public void runOpMode() {

        Arme = hardwareMap.get(Servo.class, "AA");
        initVuforia();
        initTfod();
        msStuckDetectInit = 10;

        float Object = 0;

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.2, 16.0 / 9.0);
        }

        robot.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Sets Starting place on the field
        Pose2d startPose = new Pose2d(-35, 65, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        //Middle
        Trajectory Place1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-15, 52), Math.toRadians(0))
                .build();

        Trajectory Duck1 = drive.trajectoryBuilder(Place1.end())
                .back(2)
                .splineToConstantHeading(new Vector2d(-57, 55), Math.toRadians(0))
                .build();

        Trajectory Park1 = drive.trajectoryBuilder(Duck1.end())
                .splineToConstantHeading(new Vector2d(-62, 42), Math.toRadians(180))
                .build();

        //Top
        Trajectory Place2 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-15, 52), Math.toRadians(0))
                .build();

        Trajectory Duck2 = drive.trajectoryBuilder(Place2.end())
                .back(2)
                .splineToConstantHeading(new Vector2d(-57, 55), Math.toRadians(0))
                .build();

        Trajectory Park2 = drive.trajectoryBuilder(Duck2.end())
                .splineToConstantHeading(new Vector2d(-64, 40), Math.toRadians(180))
                .build();

        //Bottom
        Trajectory Place3 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-20, 52), Math.toRadians(0))
                .build();

        Trajectory Duck3 = drive.trajectoryBuilder(Place2.end())
                .back(2)
                .splineToConstantHeading(new Vector2d(-57, 55), Math.toRadians(0))
                .build();

        Trajectory Park3 = drive.trajectoryBuilder(Duck2.end())
                .splineToConstantHeading(new Vector2d(-62, 40), Math.toRadians(180))
                .build();

        //Detects ducks and waits for the game to start
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    Object = recognition.getRight();
                    i++;
                }
            }
        }
        telemetry.addLine("Press Play To Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //Duck is in the middle
            if (Object < 300 && Object > 1) {
                telemetry.addLine("Duck = Middle");
                telemetry.update();
                Arme.setPosition(0.4);
                sleep(1000);
                robot.Down(0.345);
                robot.Off();
                drive.followTrajectory(Place1);
                robot.Grab(1.5);
                //robot.Down(0.4);
                //robot.Off();
                sleep(1000);
                robot.Grab(0.35);
                sleep(1000);
                drive.followTrajectory(Duck1);
                robot.Up(0.7);
                robot.Off();
                robot.Grab(1.5);
                sleep(1000);
                robot.Off();
                sleep(1000);
                drive.turn(Math.toRadians(-90));
                robot.LeftStrafe(0.11);
                robot.Off();
                robot.setShooterPower(-0.2);
                sleep(3000);
                robot.RightStrafe(0.1);
                drive.followTrajectory(Park1);
                Arme.setPosition(1);
                break;
            }
            //Duck is on the right
            else if ( Object > 300) {
                telemetry.addLine("Duck = Right");
                telemetry.update();
                sleep(1000);
                drive.followTrajectory(Place2);
                robot.Grab(1.5);
                robot.Down(0.4);
                robot.Off();
                sleep(1000);
                robot.Grab(0.35);
                sleep(1000);
                robot.Up(0.7);
                robot.Off();
                robot.Grab(1.5);
                sleep(1000);
                robot.Off();
                drive.followTrajectory(Duck2);
                sleep(1000);
                drive.turn(Math.toRadians(-90));
                robot.LeftStrafe(0.117);
                robot.Off();
                robot.setShooterPower(-0.2);
                sleep(3000);
                robot.RightStrafe(0.1);
                drive.followTrajectory(Park2);
                break;
            }
            //Duck is on the left
            else {
                telemetry.addLine("Duck = Left");
                telemetry.update();
                Arme.setPosition(0.654);
                sleep(1000);
                robot.Down(0.5);
                robot.Off();
                drive.followTrajectory(Place3);
                robot.Grab(1.5);
                //robot.Down(0.4);
                //robot.Off();
                sleep(1000);
                robot.Grab(0.35);
                sleep(1000);
                robot.Grab(0.6);
                sleep(1000);
                drive.followTrajectory(Duck3);
                robot.Up(0.7);
                robot.Off();
                robot.Grab(1.5);
                sleep(1000);
                robot.Off();
                sleep(1000);
                drive.turn(Math.toRadians(-90));
                robot.LeftStrafe(0.11);
                robot.Off();
                robot.setShooterPower(-0.2);
                sleep(3000);
                robot.RightStrafe(0.1);
                drive.followTrajectory(Park3);
                Arme.setPosition(1);
                break;
            }
        }
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}

