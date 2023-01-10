package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "PowerPlayAuto")
public class PowerPlayAuto extends LinearOpMode {

    //private final Vector2d CONE = new Vector2d(0,24);

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        SampleMecanumDrive RobotDrive = new SampleMecanumDrive(hardwareMap);
        //PIDFcontroller controller = new PIDFcontroller();
        //controller.Calculate();
        Pose2d startPose = new Pose2d(0, 0, 0);

        RobotDrive.setPoseEstimate(startPose);
        //QuinticSpline CustomSpline = new QuinticSpline();


        /*TrajectorySequence trajSeq = RobotDrive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d())
                .build();

        */
        Trajectory trajSeq = RobotDrive.trajectoryBuilder(startPose)
                .forward(36)
                .build();
        waitForStart();

        if (opModeIsActive()) {
            // Put run blocks here.
            //RobotDrive.followTrajectory(traj1);
            //RobotDrive.followTrajectory(traj3);

            if (!isStopRequested())
                RobotDrive.followTrajectory(trajSeq);

            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.update();
            }
        }
    }
}