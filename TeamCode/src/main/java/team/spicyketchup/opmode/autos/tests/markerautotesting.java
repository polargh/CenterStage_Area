package team.spicyketchup.opmode.autos.tests;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import team.spicyketchup.subsystem.Intake;
import team.spicyketchup.subsystem.Lift;
import team.spicyketchup.subsystem.tests.detector_2_ranges;
import team.spicyketchup.subsystem.Arm;
//import com.acmerobotics.roadrunner.geometry.Pose2d;


//import com.acmerobotics.roadrunner.trajectoryBuilder;

@Disabled
@Autonomous(name="markerauto2+4backstagered", group="Auto")
public class markerautotesting extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenCvCamera webcam;
    Lift lift;
    Arm arm;
    Intake intake;
    double LFLAPUP = .465;
    double LFLAPDOWN = .57;
    double RFLAPUP = .512;
    double RFLAPDOWN = .4309;
    double waitTime1 = 1.39;
    double waitTime2 = 0.6;
    double waitTime3 = 0.6;
    double waitTime4 = .35;
    double waitTime6 = .25;
    double waitTime7 = .25;
    double waitTime8 = .5;
    double waitTime9 = .75;
    double waitTime10 = .5;
    double waitTime11 = .5;
    private MultipleTelemetry tl;

    ElapsedTime waitTimer1 = new ElapsedTime();
    ElapsedTime waitTimer2 = new ElapsedTime();
    ElapsedTime waitTimer3 = new ElapsedTime();
    ElapsedTime waitTimer4 = new ElapsedTime();

    ElapsedTime waitTimer6 = new ElapsedTime();

    ElapsedTime waitTimer7 = new ElapsedTime();
    ElapsedTime waitTimer8 = new ElapsedTime();
    ElapsedTime waitTimer9 = new ElapsedTime();
    ElapsedTime waitTimer10 = new ElapsedTime();

    ElapsedTime waitTimer11 = new ElapsedTime();


    ElapsedTime runtime = new ElapsedTime();
//

    public enum START_POSITION {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    enum elbowDownState { //INTAKE
        START,
        MID,
        ALMOST,
        INTAKE,
        WRIST,
        AFTERINTAKE,
        IDLE

    }

    enum grab { //INTAKE
        START,

        DOWN,
        PICKPIXELS,
        UP,
        IDLE


    }


    enum elbowUpState { //OUTTAKE no lift
        START,
        OUTTAKE,
        WRIST,
        IDLE


    }

    enum Outtakelift { //OUTTAKE lift
        START,
        OUTTAKE,
        WRIST,
        IDLE


    }

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        Drop_RED.elbowUpState outtake = Drop_RED.elbowUpState.START;// no lift position
        Drop_RED.elbowDownState intakepos = Drop_RED.elbowDownState.START; // intake position
        Drop_RED.grab claw = Drop_RED.grab.START;// grab
        Drop_RED.Outtakelift deposityellow = Drop_RED.Outtakelift.START; //yellow

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
        arm.init();

        Pose2d startPose = new Pose2d(16.62, -63.42, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);
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
                .splineToConstantHeading(new Vector2d(51.3, -33.5), Math.toRadians(0.00))

                .build();

//        Trajectory left = drive.trajectoryBuilder(startPose)
//                .addTemporalMarker((0) -> {pos = Lift.LiftPos.LOW_AUTO;}
//                )
//                .splineTo(new Vector2d(10, -36), Math.toRadians(140.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(() -> {arm.bendwrist.setPosition(.148);}
//                )

               // .build();
        Trajectory deposit_middle = drive.trajectoryBuilder(middledropyellow.end())
                .forward(7.5)
                .build();
        Trajectory away_middle = drive.trajectoryBuilder(middledropyellow.end())
                .back(5.7)
                .build();
        TrajectorySequence middleplus2 = drive.trajectorySequenceBuilder(away_middle.end())

                .splineToConstantHeading(new Vector2d(25, -8.65), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-45.9, -8.3), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory middle_intakeforward = drive.trajectoryBuilder(middleplus2.end())
//                .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-57.45, -10.85), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory middle_intakebackward = drive.trajectoryBuilder(middle_intakeforward.end())
                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0.00))
                .build();
        TrajectorySequence middle_backstage2drop = drive.trajectorySequenceBuilder(middle_intakeforward.end())
                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(46.5, -13.99), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence middle_plus4 = drive.trajectorySequenceBuilder(middle_backstage2drop.end())
//                .splineToConstantHeading(new Vector2d(-43, -6), Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-47, -14), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-53.05, -9))

                .build();
        Trajectory middle_intakeforward4 = drive.trajectoryBuilder(middle_plus4.end())
//                .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .back(13.8, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .splineToConstantHeading(new Vector2d(-57.9, -11.5), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence middle_intakebackward4 = drive.trajectorySequenceBuilder(middle_intakeforward4.end())
                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(-52, -11), Math.toRadians(0.00))

                .build();
        TrajectorySequence middle_backstage4drop = drive.trajectorySequenceBuilder(middle_intakebackward4.end())
                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(46.5, -14.83), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory backup_middle = drive.trajectoryBuilder(middle.end())
                .back(3)
                .build();
        Trajectory drop_middle = drive.trajectoryBuilder(backup_middle.end())
                .lineToLinearHeading(new Pose2d(44.24, -39.75, Math.toRadians(0.00)))
                .build();


        Trajectory middle_park = drive.trajectoryBuilder(away_middle.end())
                //  .strafeRight(24)
                .strafeLeft(32)
                .build();
        Trajectory backup_right = drive.trajectoryBuilder(right.end())
                .back(3.6)
                .build();

        Trajectory right_drop = drive.trajectoryBuilder(backup_right.end())
                .lineToLinearHeading(new Pose2d(44.24, -45, Math.toRadians(0.00)))
                .build();
        Trajectory deposit_right = drive.trajectoryBuilder(right.end())
                .forward(6)
                .build();

//HEREEEEEEE
        Trajectory backup_left = drive.trajectoryBuilder(middle.end())
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

        TrajectorySequence leftplus2 = drive.trajectorySequenceBuilder(away_left.end())

                .splineToConstantHeading(new Vector2d(25, -7.95), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-45.9, -7.95), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-45.9, -11.2), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        Trajectory left_intakeforward = drive.trajectoryBuilder(leftplus2.end())
//                .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .back(12.695, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .splineToConstantHeading(new Vector2d(-56.95, -9.35), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence left_backstage2drop = drive.trajectorySequenceBuilder(left_intakeforward.end())
                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        // Runs 1 seconds after First spline so intake is stopped and brought up during second spline
                    //    .UNSTABLE_addTemporalMarkerOffset((1) ->  {intake.stopintake();
                              // intake.outtake(.25);
                               // intake.intake5(.6);
                                //intake.outtake2nd(.3);
                               // intake.outtake2ndon();})
              //  .splineToConstantHeading(new Vector2d(46.5, -13.83), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                 //       SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence left_plus4 = drive.trajectorySequenceBuilder(left_backstage2drop.end())
//                .splineToConstantHeading(new Vector2d(-43, -6), Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-47, -14), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-53.05, -9))

                .build();
        Trajectory left_intakeforward4 = drive.trajectoryBuilder(left_plus4.end())
//                .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .back(13.8, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .splineToConstantHeading(new Vector2d(-57.9, -10), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence left_intakebackward4 = drive.trajectorySequenceBuilder(left_intakeforward4.end())
                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(-52, -11), Math.toRadians(0.00))

                .build();
        TrajectorySequence left_backstage4drop = drive.trajectorySequenceBuilder(left_intakebackward4.end())
                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(46.5, -14.83), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
        TrajectorySequence rightplus2 = drive.trajectorySequenceBuilder(away_right.end())

                .splineToConstantHeading(new Vector2d(27.75, -8.4), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-45.9, -8.4), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory right_intakeforward = drive.trajectoryBuilder(rightplus2.end())
//                .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-57.9, -10.45), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory right_intakebackward = drive.trajectoryBuilder(right_intakeforward.end())
                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0.00))
                .build();
        TrajectorySequence right_backstage2drop = drive.trajectorySequenceBuilder(right_intakeforward.end())
                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(46.5, -14.83), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence right_plus4 = drive.trajectorySequenceBuilder(right_backstage2drop.end())
//                .splineToConstantHeading(new Vector2d(-43, -6), Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-47, -14), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-53.05, -9))

                .build();
        Trajectory right_intakeforward4 = drive.trajectoryBuilder(right_plus4.end())
//                .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .back(13.8, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .splineToConstantHeading(new Vector2d(-57.9, -11.5), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence right_intakebackward4 = drive.trajectorySequenceBuilder(right_intakeforward4.end())
                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(-52, -11), Math.toRadians(0.00))

                .build();
        TrajectorySequence right_backstage4drop = drive.trajectorySequenceBuilder(right_intakebackward4.end())
                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(46.5, -14.83), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        waitForStart();
        if (isStopRequested()) return;
        detector_2_ranges.Location location = detector.getLocation();
        Lift.LiftPos pos = Lift.LiftPos.LOW_AUTO;

        switch (location) {
            case LEFT: //middle

                // put state machine here


                break;

            case NOT_FOUND: //left

                //scoreLow(deposit_left, away_left, left_park);

                break;
            case RIGHT: //right
                break;

        }


    }
        }



