package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "DetectTFODAuto")
public class TFOD_PP extends LinearOpMode {
    private double BestConfidence = 0.3;
    private VuforiaCurrentGame vuforiaPOWERPLAY = new VuforiaCurrentGame();
    private Tfod tfod = new Tfod();
    public static int type = 1;
    Recognition recognition;
    List<Recognition> recognitions;
    int index;
    ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        SampleMecanumDrive RobotDrive = new SampleMecanumDrive(hardwareMap);
        RobotDrive.setPoseEstimate(new Pose2d(0,0,0));

        TrajectorySequence trajSeqBolt = RobotDrive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .strafeLeft(40)
                .forward(30)
                .build();
        TrajectorySequence trajSeqBulb = RobotDrive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .forward(30)
                .build();
        TrajectorySequence trajSeqPanel = RobotDrive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .strafeRight(40)
                .forward(30)
                .build();
        TrajectorySequence trajSeqDefaultLeft = RobotDrive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .strafeLeft(40)
                .build();

        Init();

        waitForStart();
        boolean done = false;
        time.reset();
        type = 0;
        while(true) {
            type = Recognitions();
            //type = 1;
            if (time.seconds()>20.0) {
                RobotDrive.followTrajectorySequence(trajSeqDefaultLeft);
                break;
            }
            //telemetry.addData("Time elapsed", time.seconds());
            //telemetry.update();
            if (type!=0) break;
        }
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if(!done) {
                    switch (type) {
                        case 1:
                            RobotDrive.followTrajectorySequence(trajSeqBolt);
                            break;
                        case 2:
                            RobotDrive.followTrajectorySequence(trajSeqBulb);
                            break;
                        case 3:
                            RobotDrive.followTrajectorySequence(trajSeqPanel);
                            break;
                    }
                    done = true;
                }
            }
        }
        // Deactivate TFOD.
        tfod.deactivate();

        vuforiaPOWERPLAY.close();
        tfod.close();
    }

    private int Recognitions(){
        recognitions = tfod.getRecognitions();
        // If list is empty, inform the user.
        // Otherwise, go
        // through list and display info for each recognition.
        if (JavaUtil.listLength(recognitions) == 0) {
            telemetry.addData("TFOD", "No items detected.");
            //return false;
            telemetry.update();
        } else {
            index = 0;
            // Iterate through list and call a function to
            // display info for each recognized object.
            for (Recognition recognition_item : recognitions) {
                recognition = recognition_item;
                // Display info.
                displayInfo(index);

                if(recognition.getConfidence() > BestConfidence) {
                    switch (recognition.getLabel()) {
                        case "1 Bolt":
                            type = 1;
                            BestConfidence = recognition.getConfidence();
                            break;
                        case "2 Bulb":
                            type = 2;
                            BestConfidence = recognition.getConfidence();
                            break;
                        case "3 Panel":
                            type = 3;
                            BestConfidence = recognition.getConfidence();
                            break;
                    }

                }
                telemetry.addData("TFOD: type", type);
                // Increment index.
                index = index + 1;

            }
            //return true;
        }
        return type;

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
        // Activate TFOD here so the object detection labels are vTFOD_PPisible
        // in the Camera Stream preview window on the Driver Station.
        tfod.activate();
        // Enable following block to zoom in on target.
        tfod.setZoom(2.5, 4.0 / 3.0);
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

    }
}