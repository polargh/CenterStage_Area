package team.spicyketchup.opmode.autos.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import team.spicyketchup.subsystem.Lift;
import team.spicyketchup.subsystem.tests.detector_2_ranges;

//import com.acmerobotics.roadrunner.geometry.Pose2d;


//import com.acmerobotics.roadrunner.trajectoryBuilder;


@Autonomous(name="chaintest", group="Auto")
@Disabled
public class splinechaintest extends LinearOpMode {
    OpenCvCamera webcam;
    Lift lift;
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detector_2_ranges detector = new detector_2_ranges(telemetry);
        //START_POSITION position = new CenterstageDetector(telemetry);
        webcam.setPipeline(detector);
        //private detector.getLocation location = detector.getLocation().LEFT;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                         }

                                         @Override
                                         public void onError(int errorCode) {
                                             //This will be called if the camera could not be opened
                                         }
                                     }

        );

        Pose2d startPose = new Pose2d(16.62, -63.42, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);


        Trajectory right = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(22.69, -43.41), Math.toRadians(90.00))
                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(13.22, -39.31), Math.toRadians(90.00))
                .build();

        Trajectory left = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(8.93, -39.13), Math.toRadians(140.00))
                .build();


        Trajectory backup_middle = drive.trajectoryBuilder(middle.end())
                .back(2)
                .build();
        Trajectory drop_middle = drive.trajectoryBuilder(backup_middle.end())
                .lineToLinearHeading(new Pose2d(44.86, -40.63, Math.toRadians(0.00)))
                .build();

        Trajectory backup_right = drive.trajectoryBuilder(right.end())
                .back(10)
                .build();
        Trajectory middle_park = drive.trajectoryBuilder(backup_middle.end())
                .strafeRight(25)
                .build();
        Trajectory right_park = drive.trajectoryBuilder(backup_right.end())
                .strafeRight(20)
                .build();
        Trajectory backup_left = drive.trajectoryBuilder(left.end())
                .back(30)
                .build();

        Trajectory left_strafe = drive.trajectoryBuilder(backup_left.end())
                .strafeRight(6)
                .build();
        Trajectory left_park = drive.trajectoryBuilder(left_strafe.end())
                .back(9)
                .build();
        TrajectorySequence trajright = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(22.69, -43.41), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(22.69, -49.4), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(50, -50), Math.toRadians(0.00))
               // .splineTo(new Vector2d(22.69, -45.4), Math.toRadians(90.00))

                .build();

        waitForStart();
        if (isStopRequested()) return;


        switch (detector.getLocation()) {
            case LEFT: //middle
                drive.followTrajectory(middle);
                drive.followTrajectory(backup_middle);
                drive.followTrajectory(drop_middle);
                //lift.moveToTarget(Lift.LiftPos.LOW);
                //drive.followTrajectory(middle_park);
                break;
            case RIGHT: //right
                drive.followTrajectorySequence(trajright);
                break;
            case NOT_FOUND: //left
                drive.followTrajectory(left);
                drive.followTrajectory(backup_left);
                drive.followTrajectory(left_strafe);
                drive.followTrajectory(left_park);
                break;
        }






        webcam.stopStreaming();
    }
}



