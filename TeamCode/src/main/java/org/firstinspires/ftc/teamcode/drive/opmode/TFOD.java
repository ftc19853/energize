package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@Autonomous(name = "DetectTFODAuto")
public class TFOD extends LinearOpMode {
    private double BestConfidence = 0.0;
    private VuforiaCurrentGame vuforiaPOWERPLAY = new VuforiaCurrentGame();
    private Tfod tfod = new Tfod();
    public static int type = 1;
    Recognition recognition;
    List<Recognition> recognitions;
    int index;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        SampleMecanumDrive RobotDrive = new SampleMecanumDrive(hardwareMap);
        RobotDrive.setPoseEstimate(new Pose2d(0,0,0));
        Trajectory trajSeqBolt = RobotDrive.trajectoryBuilder(new Pose2d(0,0,0))
                 .splineTo(new Vector2d(-24,0),0)
                 .splineTo(new Vector2d(-24,24),0)
                 .build();

        Trajectory trajSeqBulb = RobotDrive.trajectoryBuilder(new Pose2d(0,0,0))
                .splineTo(new Vector2d(-24,0),0)
                .build();

        Trajectory trajSeqPanel = RobotDrive.trajectoryBuilder(new Pose2d(0,0,0))
                .splineTo(new Vector2d(-24,0),0)
                .splineTo(new Vector2d(-24,-24),0)
                .build();

        Init();

        waitForStart();
        if (opModeIsActive()) {
            if (type == 1){
                RobotDrive.followTrajectory(trajSeqBolt);
            } else if(type == 2 ){
                RobotDrive.followTrajectory(trajSeqBulb);
            } else if(type == 3 ){
                RobotDrive.followTrajectory(trajSeqPanel);
            } else {}
            while (opModeIsActive()) {

                //Recognitions();

            }
        }
        // Deactivate TFOD.
        tfod.deactivate();

        vuforiaPOWERPLAY.close();
        tfod.close();
    }

    private void Recognitions(){

        recognitions = tfod.getRecognitions();
        // If list is empty, inform the user. Otherwise, go
        // through list and display info for each recognition.
        if (JavaUtil.listLength(recognitions) == 0) {
            telemetry.addData("TFOD", "No items detected.");
            //return false;
        } else {
            index = 0;
            // Iterate through list and call a function to
            // display info for each recognized object.
            for (Recognition recognition_item : recognitions) {
                recognition = recognition_item;
                // Display info.
                displayInfo(index);
                // Increment index.
                index = index + 1;

            }
            //return true;
        }

    }
    private void Init(){
        vuforiaPOWERPLAY.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        tfod.useDefaultModel();
        // Set min confidence threshold to 0.7
        tfod.initialize(vuforiaPOWERPLAY, (float) 0.7, true, true);
        // Initialize TFOD before waitForStart.
        // Activate TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfod.activate();
        // Enable following block to zoom in on target.
        tfod.setZoom(2, 16.0 / 9.0);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }
    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayInfo(int i) {


        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Left, Top " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        telemetry.addData("Right, Bottom " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));


        if(recognition.getConfidence() > BestConfidence){
            switch (recognition.getLabel()){
                case "1 Bolt":
                    type = 1;
                    BestConfidence = recognition.getConfidence();
                case "2 Bulb":
                    type = 2;
                    BestConfidence = recognition.getConfidence();
                case "3 Panel":
                    type = 3;
                    BestConfidence = recognition.getConfidence();
            }

        }
    }
}