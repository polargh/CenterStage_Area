package Autos;

import Hardware.Arm;
import Hardware.Intake;
import Hardware.Lift;
import Hardware.detector_2_ranges;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Disabled
@Autonomous(name="Close Truss 2+4 Backstage", group="Auto")
public class CloseRedTruss2Plus4Backstage extends LinearOpMode {

    TrajectorySequence toStack;
    TrajectorySequence toBoard;
    double LFLAPUP = .465;
    double LFLAPDOWN = .57;
    double RFLAPUP = .512;
    double RFLAPDOWN = .4309;

    SampleMecanumDrive drive;
    OpenCvCamera webcam;
    Lift lift;
    Arm arm;
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        initCam();
        arm.init();

        Pose2d startPose = new Pose2d(16.62, -63.42, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);

        Lift.LiftPos pos = Lift.LiftPos.LOW_AUTO;
        Pose2d yellowPose = new Pose2d(0, 0, 0);

        // TODO: change startpose to spike mark yellow path.end()


        Trajectory right = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(24.2, -40), Math.toRadians(90.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory middle = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(13, -35), Math.toRadians(90.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory middleback = drive.trajectoryBuilder(middle.end())
                .back(12)
                .build();
        Trajectory middledropyellow = drive.trajectoryBuilder(middleback.end())
                .splineTo(new Vector2d(42.8, -33.83), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(), Math.toRadians(0.00))

                .build();
        Trajectory left = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(10, -36), Math.toRadians(140.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory deposit_middle = drive.trajectoryBuilder(middledropyellow.end())
                .forward(7.5)
                .build();
        Trajectory away_middle = drive.trajectoryBuilder(middledropyellow.end())
                .back(5.7)
                .build();
        Trajectory backup_left = drive.trajectoryBuilder(left.end())
                .back(15)
                .build();
        Trajectory leftdropyellow = drive.trajectoryBuilder(backup_left.end())
                // .splineTo(new Vector2d(13, -35), Math.toRadians(0.00))
                .splineTo(new Vector2d(42.8, -25), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(50.1, -25), Math.toRadians(0.00))

                .build();
        Trajectory away_left = drive.trajectoryBuilder(leftdropyellow.end())
                .back(6)
                .build();
        Trajectory rightback = drive.trajectoryBuilder(right.end())
                .back(7)
                .build();
        Trajectory rightdropyellow = drive.trajectoryBuilder(rightback.end())
                .splineTo(new Vector2d(42.8, -40), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(50, -40), Math.toRadians(0.00))
                .build();
        Trajectory away_right = drive.trajectoryBuilder(rightdropyellow.end())
                .back(5.7)
                .build();


        // TODO: this should be ur spikepos
        toStack = drive.trajectorySequenceBuilder(yellowPose)
                .addTemporalMarker(() -> {
                    // RESET BOT
                    arm.lflap.setPosition(LFLAPDOWN);
                    arm.rflap.setPosition(RFLAPDOWN);
                    arm.intakePos();

                    // PUT LIFT DOWN, ARM IN, TRANSFER POSITION
                })

                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(12, -60), Math.PI)
                .splineTo(new Vector2d(-26, -60), Math.toRadians(180))
                .splineTo(new Vector2d(-60, -36), Math.toRadians(180))

                .addTemporalMarker(() -> {
                    // WHAT YOU DO AT STACK

                })
                .waitSeconds(.5)

                .build();

        toBoard = drive.trajectorySequenceBuilder(toStack.end())
                // transfer


                .setTangent(0)
                .splineTo(new Vector2d(-26, -60), Math.toRadians(0))
                .addTemporalMarker(0.5, () -> {
                    // PRE OUTTAKING STUFF
                    // ARM OUT

                })
                .splineTo(new Vector2d(43, -60), Math.PI)

                .addTemporalMarker(0.4,() -> {
                    // WHAT YOU DO AT BOARD

                })

                .waitSeconds(0.3)


                .build();




        // TODO: run tostack toboard
        // drive.followTrajectorySequence(spikemark paths)

        // drive.followTrajectorySequence(toStack);
        // drive.followTrajectorySequence(toBoard);
        // drive.followTrajectorySequence(toStack);
        // drive.followTrajectorySequence(toBoard);

        waitForStart();

      //  detector_2_ranges.Location location = detector.getLocation();

        while (opModeIsActive()) {

//            switch (location) {
//                case LEFT: //middle spike
////spike mark code
//
//
//                    break;
//                case NOT_FOUND: //left spike
////spike mark code
//
//
//                    break;
//                case RIGHT: //right spike
////spike mark code
//
//                    break;
//            }



            drive.update();
            lift.update(pos);
        }
    }

    private  void initCam() {
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
    }
}