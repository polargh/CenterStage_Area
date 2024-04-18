package team.spicyketchup.opmode.autos.blue.untested;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import team.spicyketchup.subsystem.Arm;
import team.spicyketchup.subsystem.Intake;
import team.spicyketchup.subsystem.Lift;
import team.spicyketchup.subsystem.vision.tests.blueAudiencePipeline;

@Autonomous(name = "Hivemind Auto Blue 2+3", group = "tests")
public class Blue23TrussAudience extends LinearOpMode {

    SampleMecanumDrive drive;
    OpenCvWebcam webcam;

    Arm arm;
    Lift lift;
    Intake intake;

    public static double waitTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);

        int cameraIdx = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraIdx);

        blueAudiencePipeline pipeline = new blueAudiencePipeline(telemetry);

        blueAudiencePipeline.Location location = pipeline.getLocation();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int i) {
                System.out.println("Experienced error while attempting to scan prop. Id: " + i);
            }
        });

        /*
         * Send initial hardware calls
        */

        arm.intakePostele();
        arm.drop.setPosition(0.9);

        Pose2d startPose = new Pose2d(-35.5, 60, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        waitForStart();

        webcam.stopStreaming();

        /*
         * Trajectories
        */

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(waitTime)
                .splineTo(new Vector2d(-35, 35), Math.toRadians(0))
                .addTemporalMarker(1, () -> intake.intakewhile5())
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(waitTime)
                .splineTo(new Vector2d(-35.5, 35), Math.toRadians(-90))
                .addTemporalMarker(1, () -> intake.intakewhile5())
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(waitTime)
                .splineTo(new Vector2d(-44, 36), Math.toRadians(180))
                .addTemporalMarker(1, () -> intake.intakewhile5())
                .build();

        // Trajectories


        TrajectorySequence goToStackFromProp;
        TrajectorySequence splineToDepositPosition;
        TrajectorySequence backdropToIntake;
        TrajectorySequence intakeToBackdrop;

        while (!isStopRequested() && opModeIsActive()) {
            TrajectorySequence selected = null;

            switch (location) {
                case LEFT:
                    selected = left;
                    break;
                case RIGHT:
                    selected = right;
                    break;
                case NOT_FOUND:
                    selected = middle;
                    break;
            }

            goToStackFromProp = drive.trajectorySequenceBuilder(selected.end())
                    .lineToLinearHeading(new Pose2d(-60, 35, Math.toRadians(0)))
                    .addTemporalMarker(0, () -> {
                        // todo: intake logic
                    })
                    .build();

            splineToDepositPosition = drive.trajectorySequenceBuilder(goToStackFromProp.end())
                    .addTemporalMarker(0.1, () -> {
                        intake.stopIntake();
                        arm.drop.setPosition(.59);
                        arm.raxon.setPosition(.783);
                        arm.laxon.setPosition(.217);
                        arm.bendwrist.setPosition(.159);
                    })
                    .addTemporalMarker(0.1, () -> arm.grab())
                    .addTemporalMarker(0.1, () -> arm.aftergrab())
                    .splineTo(new Vector2d(-30.01, 58.96), Math.toRadians(0.00))
                    .lineTo(new Vector2d(24.30, 59.49))
                    .splineTo(new Vector2d(50, 35), Math.toRadians(0.00))
                    .addTemporalMarker(0, () -> {
                        // todo: deposit logic
                    })
                    .build();

            backdropToIntake = drive.trajectorySequenceBuilder(splineToDepositPosition.end())
                    .back(5)
                    .splineTo(new Vector2d(23.94, 58.96), Math.toRadians(180.00))
                    .splineTo(new Vector2d(-45.20, 51.63), Math.toRadians(200))
                    .splineTo(new Vector2d(-60, 35), Math.toRadians(180.00))
                    .addTemporalMarker(1.5, () -> {
                        // todo: intake logic
                    })
                    .build();

            intakeToBackdrop = drive.trajectorySequenceBuilder(backdropToIntake.end())
                    .splineTo(new Vector2d(-30.01, 58.96), Math.toRadians(0.00))
                    .lineTo(new Vector2d(24.30, 59.49))
                    .splineTo(new Vector2d(50, 35), Math.toRadians(0.00))
                    .addTemporalMarker(1.5, () -> {
                        // todo: deposit logic
                    })
                    .build();

            Trajectory park = drive.trajectoryBuilder(intakeToBackdrop.end())
                    .strafeLeft(24)
                    .addTemporalMarker(1.5, () -> {
                        // reset all mechanisms
                    })
                    .build();

            drive.followTrajectorySequence(selected);
            drive.followTrajectorySequence(goToStackFromProp);
            drive.followTrajectorySequence(splineToDepositPosition);
            drive.followTrajectorySequence(backdropToIntake);
            drive.followTrajectorySequence(intakeToBackdrop);
            drive.followTrajectory(park);
        }
    }
}
