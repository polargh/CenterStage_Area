package team.spicyketchup.opmode.autos.red;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import team.spicyketchup.subsystem.Arm;
import team.spicyketchup.subsystem.Intake;
import team.spicyketchup.subsystem.Lift;
import team.spicyketchup.subsystem.tests.detector_2_ranges;
import team.spicyketchup.opmode.autos.tests.Drop_RED;

//import com.acmerobotics.roadrunner.geometry.Pose2d;


//import com.acmerobotics.roadrunner.trajectoryBuilder;


@Autonomous(name="Red_2+4_back_truss", group="Auto")
public class Red24TrussBackstage extends LinearOpMode {
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
                .splineToConstantHeading(new Vector2d(51.3, -33.1), Math.toRadians(0.00))

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
//        TrajectorySequence middleplus2 = drive.trajectorySequenceBuilder(away_middle.end())
//
//                .splineToConstantHeading(new Vector2d(25, -8.65), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-45.9, -8.3), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
        TrajectorySequence middleundertruss = drive.trajectorySequenceBuilder(away_middle.end())
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(12, -56), Math.PI)
                .splineTo(new Vector2d(-30, -56), Math.toRadians(180))
                .splineTo(new Vector2d(-56, -37), Math.toRadians(148))
                .build();

        Trajectory middle_intakeforward = drive.trajectoryBuilder(middleundertruss.end())
//                .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-58.75, -38.5), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory middle_intakebackward = drive.trajectoryBuilder(middle_intakeforward.end())
                .splineToConstantHeading(new Vector2d(-52, -40), Math.toRadians(0.00))
                .build();
        TrajectorySequence middle_backstage2drop = drive.trajectorySequenceBuilder(middle_intakeforward.end())
                .splineTo(new Vector2d(-29.48, -56), Math.toRadians(0.00))
                .splineTo(new Vector2d(40.48, -56), Math.toRadians(0.00))
                .build();

        TrajectorySequence middle_plus4 = drive.trajectorySequenceBuilder(middle_backstage2drop.end())
              .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(12, -56), Math.PI)
                .splineTo(new Vector2d(-30, -56), Math.toRadians(180))
                .splineTo(new Vector2d(-56, -37), Math.toRadians(148))

                .build();
        Trajectory middle_intakeforward4 = drive.trajectoryBuilder(middle_plus4.end())
//                .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .back(13.8, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .splineToConstantHeading(new Vector2d(-58.75, -38.5), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence middle_intakebackward4 = drive.trajectorySequenceBuilder(middle_intakeforward4.end())
                .splineToConstantHeading(new Vector2d(-52, -40), Math.toRadians(0.00))

                .build();
        TrajectorySequence middle_backstage4drop = drive.trajectorySequenceBuilder(middle_intakebackward4.end())
                .splineTo(new Vector2d(-29.48, -56), Math.toRadians(0.00))
                .splineTo(new Vector2d(44, -56), Math.toRadians(0.00))
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
        Trajectory backup_left = drive.trajectoryBuilder(left.end())
                .back(15)
                .build();
        Trajectory leftdropyellow = drive.trajectoryBuilder(backup_left.end())
                // .splineTo(new Vector2d(13, -35), Math.toRadians(0.00))
                .splineTo(new Vector2d(38.5, -25), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(48.5, -25), Math.toRadians(0.00))

                .build();
        Trajectory away_left = drive.trajectoryBuilder(leftdropyellow.end())
                .back(6)
                .build();

        TrajectorySequence leftplus2 = drive.trajectorySequenceBuilder(away_left.end())

                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(12, -56), Math.PI)
                .splineTo(new Vector2d(-30, -56), Math.toRadians(180))
                .splineTo(new Vector2d(-56, -37), Math.toRadians(148))
                .build();


        Trajectory left_intakeforward = drive.trajectoryBuilder(leftplus2.end())
//                .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .back(12.695, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .splineToConstantHeading(new Vector2d(-58.75, -38.5), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory left_intakebackward = drive.trajectoryBuilder(left_intakeforward.end())
                .splineToConstantHeading(new Vector2d(-52, -40), Math.toRadians(0.00))
                .build();
        TrajectorySequence left_backstage2drop = drive.trajectorySequenceBuilder(left_intakeforward.end())
                .splineTo(new Vector2d(-29.48, -56), Math.toRadians(0.00))
                .splineTo(new Vector2d(44, -56), Math.toRadians(0.00))
                .build();
        TrajectorySequence left_plus4 = drive.trajectorySequenceBuilder(left_backstage2drop.end())
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(12, -56), Math.PI)
                .splineTo(new Vector2d(-30, -56), Math.toRadians(180))
                .splineTo(new Vector2d(-56, -37), Math.toRadians(148))

                .build();
        Trajectory left_intakeforward4 = drive.trajectoryBuilder(left_plus4.end())
//                .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .back(13.8, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .splineToConstantHeading(new Vector2d(-58.78, -38.5), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence left_intakebackward4 = drive.trajectorySequenceBuilder(left_intakeforward4.end())
                .splineToConstantHeading(new Vector2d(-52, -40), Math.toRadians(0.00))

                .build();
        TrajectorySequence left_backstage4drop = drive.trajectorySequenceBuilder(left_intakebackward4.end())
                .splineTo(new Vector2d(-29.48, -56), Math.toRadians(0.00))
                .splineTo(new Vector2d(44, -56), Math.toRadians(0.00))
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

                .splineToConstantHeading(new Vector2d(27.75, -8.4), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-45.9, -8.4), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence rightundertruss = drive.trajectorySequenceBuilder(away_right.end())
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(12, -56), Math.PI)
                .splineTo(new Vector2d(-30, -56), Math.toRadians(180))
                .splineTo(new Vector2d(-56, -41), Math.toRadians(148))
                .build();
        Trajectory right_intakeforward = drive.trajectoryBuilder(rightundertruss.end())
//                .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-59.4, -39), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory right_intakebackward = drive.trajectoryBuilder(right_intakeforward.end())
                .splineToConstantHeading(new Vector2d(-52, -40), Math.toRadians(0.00))
                .build();
        TrajectorySequence right_backstage2drop = drive.trajectorySequenceBuilder(right_intakebackward.end())
//                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(46.5, -14.83), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-29.48, -56), Math.toRadians(0.00))
                .splineTo(new Vector2d(40.48, -56), Math.toRadians(0.00))

                .build();

        TrajectorySequence right_plus4 = drive.trajectorySequenceBuilder(right_backstage2drop.end())
               .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(12, -56), Math.PI)
                .splineTo(new Vector2d(-30, -56), Math.toRadians(180))
                .splineTo(new Vector2d(-56, -41), Math.toRadians(148))

                .build();
        Trajectory right_intakeforward4 = drive.trajectoryBuilder(right_plus4.end())
//                .splineToConstantHeading(new Vector2d(-55, -11), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .back(13.8, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .splineToConstantHeading(new Vector2d(-59.4, -39.5), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence right_intakebackward4 = drive.trajectorySequenceBuilder(right_intakeforward4.end())
                .splineToConstantHeading(new Vector2d(-52, -40), Math.toRadians(0.00))

                .build();
        TrajectorySequence right_backstage4drop = drive.trajectorySequenceBuilder(right_intakebackward4.end())
                .splineTo(new Vector2d(-29.48, -56), Math.toRadians(0.00))
                .splineTo(new Vector2d(44, -56), Math.toRadians(0.00))
                .build();





        waitForStart();
        if (isStopRequested()) return;
        detector_2_ranges.Location location = detector.getLocation();
        Lift.LiftPos pos = Lift.LiftPos.LOW_AUTO;

        switch (location) {
            case LEFT: //middle

                // put state machine here
                int state = 0; ///middle
                boolean done = false;
                while (opModeIsActive() && !isStopRequested() && !done) {
                    switch(state) { //middle
                        case 0:
                            pos = Lift.LiftPos.LOW_AUTO;
                            drive.followTrajectoryAsync(middle);
                            //  arm.drop.setPosition(.63);
                            arm.bendwrist.setPosition(.148);
                            state = 1;
                            break;
                        case 1:
                            if (!drive.isBusy()) {
                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.3);
                                arm.laxon.setPosition(.7);
                                drive.followTrajectoryAsync(middleback);
                                state = 2;
                            }
                            break;
                        case 2:
                            if (!drive.isBusy()) {
                                arm.rotwrist.setPosition(.7);
                                arm.bendwrist.setPosition(.705);
                                arm.drop.setPosition(.95);
                                drive.followTrajectoryAsync(middledropyellow);
                                state = 4;
                            }
                            break;
//                        case 3:
//                            if (!drive.isBusy()) {
//                               //
//                                arm.release();
//                                state = 4;
//                            }
//                            break;
                        case 4:
                            if (!drive.isBusy()) {
                                // TODO: Maybe add a small delay
                                arm.release();
                                drive.followTrajectoryAsync(away_middle);

                                state = 5;
                            }
                            break;
                        case 5:
                            if (!drive.isBusy()) {
/// change here to go thru truss
                                arm.intakePosafterscore();
                                arm.intakePos();
                                pos = Lift.LiftPos.START;
                                drive.followTrajectorySequenceAsync(middleundertruss);
                                arm.drop.setPosition(.669);

                                state = 6;
                            }
                            break;
                        case 6:
                            if (!drive.isBusy()) {

                                intake.intakewhile5();
                                drive.followTrajectory(middle_intakeforward);
                                waitTimer1.reset();
                                state = 7;
                            }
                            break;
                        case 7:
                            if (waitTimer1.seconds()>= waitTime1) {

                                intake.stopintake();
                                intake.outtake(.25);
                                intake.intake5(.6);
                                intake.outtake2nd(.3);
                                intake.outtake2ndon();
                                arm.drop.setPosition(.94);
                                drive.followTrajectoryAsync(middle_intakebackward);
                                intake.stopintake();
                                arm.release();
                                arm.lflap.setPosition(LFLAPUP);
                                arm.rflap.setPosition(RFLAPUP);

                                state = 8;
                            }
                            break;
                        case 8:
                            if (!drive.isBusy()) {

                                arm.raxon.setPosition(.783);
                                arm.laxon.setPosition(.217);
                                arm.bendwrist.setPosition(.159);
                                drive.followTrajectorySequenceAsync(middle_backstage2drop);
                                waitTimer2.reset();
                                state = 9;
                            }
                            break;
                        case 9:
                            if (waitTimer2.seconds() >= waitTime2) {

                                arm.grab();

                                waitTimer3.reset();
                                state = 10;
                            }
                            break;
                        case 10:
                            if (waitTimer3.seconds() >= waitTime3) {

                                arm.aftergrab();

                                waitTimer4.reset();
                                state = 11;
                            }
                            break;
                        case 11:
                            if (waitTimer4.seconds() >= waitTime4) {

                                arm.bendwrist.setPosition(.148);


                                waitTimer6.reset();
                                state = 12;
                            }
                            break;
                        case 12:
                            if (waitTimer6.seconds() >= waitTime6) {

                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.28);
                                arm.laxon.setPosition(.72);

                                waitTimer7.reset();
                                state = 13;
                            }
                            break;
                        case 13:
                            if (waitTimer7.seconds() >= waitTime7) {
                                arm.rotwrist.setPosition(.5375);
                                arm.bendwrist.setPosition(.77);
                                arm.drop.setPosition(.95);
                                waitTimer7.reset();
                                state = 14;
                            }
                            break;
                        case 14:
                            if (!drive.isBusy()) {

                                arm.release();


                                state = 15;
                            }
                            break;// goes for 4
                        case 15:
                            if (!drive.isBusy()) {
                                arm.intakePos();
                                arm.lflap.setPosition(LFLAPDOWN);
                                arm.rflap.setPosition(RFLAPDOWN);
                                drive.followTrajectorySequenceAsync(middle_plus4);
                                state = 16;
                            }
                            break;
                        case 16:
                            if (!drive.isBusy()) {

                                intake.intakewhile3();
                                drive.followTrajectory(middle_intakeforward4);
                                waitTimer10.reset();


                                state = 17;
                            }
                            break;
                        case 17:
                            if (waitTimer10.seconds() >= waitTime10) {
                                intake.outtake(.25);
                                intake.intake3(.7);
                                intake.outtake2nd(.3);
                                intake.outtake2ndon();
                                arm.drop.setPosition(.951);
                                drive.followTrajectorySequenceAsync(middle_intakebackward4);
                                intake.stopintake();
                                arm.lflap.setPosition(LFLAPUP);
                                arm.rflap.setPosition(RFLAPUP);
                                arm.release();


                                waitTimer8.reset();
                                state = 18;
                            }
                            break;
                        case 18:
                            if (!drive.isBusy() && waitTimer8.seconds() >= waitTime8) {

                                drive.followTrajectorySequenceAsync(middle_backstage4drop);
                                arm.raxon.setPosition(.783);
                                arm.laxon.setPosition(.217);
                                arm.bendwrist.setPosition(.159);


                                waitTimer2.reset();
                                state = 19;
                            }
                            break;
                        case 19:
                            if (waitTimer2.seconds() >= waitTime2) {

                                arm.grab();

                                waitTimer3.reset();
                                state = 20;
                            }
                            break;
                        case 20:
                            if (waitTimer3.seconds() >= waitTime3) {

                                arm.aftergrab();

                                waitTimer4.reset();
                                state = 21;
                            }
                            break;
                        case 21:
                            if (waitTimer4.seconds() >= waitTime4) {

                                arm.bendwrist.setPosition(.148);


                                waitTimer6.reset();
                                state = 22;
                            }
                            break;
                        case 22:
                            if (waitTimer6.seconds() >= waitTime6) {

                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.28);
                                arm.laxon.setPosition(.72);

                                waitTimer7.reset();
                                state = 23;
                            }
                            break;
                        case 23:
                            if (waitTimer7.seconds() >= waitTime7) {
                                arm.rotwrist.setPosition(.685);
                                arm.bendwrist.setPosition(.77);
                                arm.drop.setPosition(.95);
                                waitTimer7.reset();
                                state = 24;
                            }
                            break;
                        case 24:
                            if (!drive.isBusy()) {

                                arm.release();


                                state = 25;
                            }
                        case 25:
                            if (!drive.isBusy()) {

                                arm.intakePos();

                                waitTimer9.reset();
                                state = 26;
                            }
                            break;
                        case 26:
                            if (waitTimer9.seconds() >= waitTime9) {

                                done = true;
                                break;
                            }

                    }
                    drive.update();
                    lift.update(pos);
                }

                break;
            case NOT_FOUND: //left

                int stateleft = 0;
                boolean doneleft = false;
                while (opModeIsActive() && !isStopRequested() && !doneleft) {
                    switch(stateleft) {
                        case 0:
                            pos = Lift.LiftPos.LOW_AUTO;
                            drive.followTrajectoryAsync(left);
                            //  arm.drop.setPosition(.63);
                            arm.bendwrist.setPosition(.148);
                            stateleft = 1;
                            break;
                        case 1:
                            if (!drive.isBusy()) {
                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.3);
                                arm.laxon.setPosition(.7);
                                drive.followTrajectoryAsync(backup_left);
                                stateleft = 2;
                            }
                            break;
                        case 2:
                            if (!drive.isBusy()) {
                                arm.rotwrist.setPosition(.685);
                                arm.bendwrist.setPosition(.705);
                                arm.drop.setPosition(.95);
                                drive.followTrajectoryAsync(leftdropyellow);
                                waitTimer11.reset();
                                stateleft = 4;
                            }
                            break;
//                        case 3:
//                            if (!drive.isBusy()) {
//                               //
//                                arm.release();
//                                state = 4;
//                            }
//                            break;
                        case 4:
                            if (!drive.isBusy() && waitTimer11.seconds()>= waitTime11) {
                                // TODO: Maybe add a small delay
                                arm.release();
                                drive.followTrajectoryAsync(away_left);

                                stateleft = 5;
                            }
                            break;
                        case 5:
                            if (!drive.isBusy()) {
//change here truss
                                arm.intakePosafterscore();
                                arm.intakePos();
                                pos = Lift.LiftPos.START;
                                drive.followTrajectorySequenceAsync(leftplus2);
                                arm.drop.setPosition(.669);

                                stateleft = 6;
                            } // leftttt
                            break;
                        case 6:
                            if (!drive.isBusy()) {

                                intake.intakewhile5();
                                drive.followTrajectory(left_intakeforward);
                                waitTimer1.reset();
                                stateleft = 7;
                            }
                            break;
                        case 7:
                            if (waitTimer1.seconds()>= waitTime1) {

                                intake.stopintake();
                                intake.outtake(.15);
                                intake.intake5(.6);
                                arm.drop.setPosition(.945);

                                intake.outtake2ndon();
                                drive.followTrajectoryAsync(left_intakebackward);
                                intake.stopintake();
                                arm.release();
                                arm.lflap.setPosition(LFLAPUP);
                                arm.rflap.setPosition(RFLAPUP);

                                stateleft = 8;
                            }
                            break;
                        case 8:
                            if (!drive.isBusy()) {

                                arm.raxon.setPosition(.783);
                                arm.laxon.setPosition(.217);
                                arm.bendwrist.setPosition(.159);
                                drive.followTrajectorySequenceAsync(left_backstage2drop);
                                waitTimer2.reset();
                                stateleft = 9;
                            }
                            break;
                        case 9:
                            if (waitTimer2.seconds() >= waitTime2) {

                                arm.grab();

                                waitTimer3.reset();
                                stateleft = 10;
                            }
                            break;
                        case 10:
                            if (waitTimer3.seconds() >= waitTime3) {

                                arm.aftergrab();

                                waitTimer4.reset();
                                stateleft = 11;
                            }
                            break;
                        case 11:
                            if (waitTimer4.seconds() >= waitTime4) {

                                arm.bendwrist.setPosition(.148);


                                waitTimer6.reset();
                                stateleft = 12;
                            }
                            break;
                        case 12:
                            if (waitTimer6.seconds() >= waitTime6) {

                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.28);
                                arm.laxon.setPosition(.72);

                                waitTimer7.reset();
                                stateleft = 13;
                            }
                            break;
                        case 13:
                            if (waitTimer7.seconds() >= waitTime7) {
                                arm.rotwrist.setPosition(.685);
                                arm.bendwrist.setPosition(.77);
                                arm.drop.setPosition(.95);
                                waitTimer7.reset();
                                stateleft = 14;
                            }
                            break;
                        case 14:
                            if (!drive.isBusy()) {

                                arm.release();


                                stateleft = 15;
                            }
                            break;// goes for 4
                        case 15:
                            if (!drive.isBusy()) {
                                arm.intakePos();
                                arm.lflap.setPosition(LFLAPDOWN);
                                arm.rflap.setPosition(RFLAPDOWN);
                                drive.followTrajectorySequenceAsync(left_plus4);
                                stateleft = 16;
                            }
                            break;
                        case 16:
                            if (!drive.isBusy()) {

                                intake.intakewhile3();
                                drive.followTrajectory(left_intakeforward4);
                                waitTimer10.reset();


                                stateleft = 17;
                            }
                            break;
                        case 17:
                            if (waitTimer10.seconds() >= waitTime10) {
                                intake.outtake(.25);
                                intake.intake3(.7);
                                intake.outtake(.5);
                                drive.followTrajectorySequenceAsync(left_intakebackward4);
                                arm.lflap.setPosition(LFLAPUP);
                                arm.rflap.setPosition(RFLAPUP);
                                arm.release();


                                waitTimer8.reset();
                                stateleft = 18;
                            }
                            break;
                        case 18:
                            if (!drive.isBusy() && waitTimer8.seconds() >= waitTime8) {

                                drive.followTrajectorySequenceAsync(left_backstage4drop);
                                arm.raxon.setPosition(.783);
                                arm.laxon.setPosition(.217);
                                arm.bendwrist.setPosition(.159);


                                waitTimer2.reset();
                                stateleft = 19;
                            }
                            break;
                        case 19:
                            if (waitTimer2.seconds() >= waitTime2) {

                                arm.grab();

                                waitTimer3.reset();
                                stateleft = 20;
                            }
                            break;
                        case 20:
                            if (waitTimer3.seconds() >= waitTime3) {

                                arm.aftergrab();

                                waitTimer4.reset();
                                stateleft = 21;
                            }
                            break;
                        case 21:
                            if (waitTimer4.seconds() >= waitTime4) {

                                arm.bendwrist.setPosition(.148);


                                waitTimer6.reset();
                                stateleft = 22;
                            }
                            break;
                        case 22:
                            if (waitTimer6.seconds() >= waitTime6) {

                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.28);
                                arm.laxon.setPosition(.72);

                                waitTimer7.reset();
                                stateleft = 23;
                            }
                            break;
                        case 23:
                            if (waitTimer7.seconds() >= waitTime7) {
                                arm.rotwrist.setPosition(.685);
                                arm.bendwrist.setPosition(.77);
                                arm.drop.setPosition(.95);
                                waitTimer7.reset();
                                stateleft = 24;
                            }
                            break;
                        case 24:
                            if (!drive.isBusy()) {

                                arm.release();


                                stateleft = 25;
                            }
                        case 25:
                            if (!drive.isBusy()) {

                                arm.intakePos();

                                waitTimer9.reset();
                                stateleft = 26;
                            }
                            break;
                        case 26:
                            if (waitTimer9.seconds() >= waitTime9) {

                                doneleft = true;
                                break;
                            }

                    }
                    drive.update();
                    lift.update(pos);
                }

                //scoreLow(deposit_left, away_left, left_park);

                break;
            case RIGHT: //right
                int rstate = 0; ///middle
                boolean rdone = false;
                while (opModeIsActive() && !isStopRequested() && !rdone) {
                    switch(rstate) { //middle
                        case 0:
                            pos = Lift.LiftPos.LOW_AUTO;
                            drive.followTrajectoryAsync(right);
                            //  arm.drop.setPosition(.63);
                            arm.bendwrist.setPosition(.148);
                            rstate = 1;
                            break;
                        case 1:
                            if (!drive.isBusy()) {
                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.3);
                                arm.laxon.setPosition(.7);
                                drive.followTrajectoryAsync(rightback);
                                rstate = 2;
                            }
                            break;
                        case 2:
                            if (!drive.isBusy()) {
                                arm.rotwrist.setPosition(.5375);
                                arm.bendwrist.setPosition(.705);
                                arm.drop.setPosition(.95);
                                drive.followTrajectoryAsync(rightdropyellow);
                                rstate = 4;
                            }
                            break;
//                        case 3:
//                            if (!drive.isBusy()) {
//                               //
//                                arm.release();
//                                state = 4;
//                            }
//                            break;
                        case 4:
                            if (!drive.isBusy()) {
                                // TODO: Maybe add a small delay
                                arm.release();
                                drive.followTrajectoryAsync(away_right);

                                rstate = 5;
                            }
                            break;
                        case 5:
                            if (!drive.isBusy()) {
//change here to truss
                                arm.intakePosafterscore();
                                arm.intakePos();
                                pos = Lift.LiftPos.START;
                                drive.followTrajectorySequenceAsync(rightundertruss);
                                arm.drop.setPosition(.669);

                                rstate = 6;
                            }
                            break;
                        case 6:
                            if (!drive.isBusy()) {

                                intake.intakewhile5();
                                drive.followTrajectory(right_intakeforward);
                                waitTimer1.reset();
                                rstate = 7;
                            }
                            break;
                        case 7:
                            if (waitTimer1.seconds()>= waitTime1) {

                                intake.stopintake();
                                intake.outtake(.25);
                                intake.intake3(.6);
                                drive.followTrajectoryAsync(right_intakebackward);
                                intake.outtake2nd(.45);
                                arm.release();
                                arm.lflap.setPosition(LFLAPUP);
                                arm.rflap.setPosition(RFLAPUP);

                                rstate = 8;
                            }
                            break;
                        case 8:
                            if (!drive.isBusy()) {

                                arm.raxon.setPosition(.783);
                                arm.laxon.setPosition(.217);
                                arm.bendwrist.setPosition(.159);
                                drive.followTrajectorySequenceAsync(right_backstage2drop);
                                waitTimer2.reset();
                                rstate = 9;
                            }
                            break;
                        case 9:
                            if (waitTimer2.seconds() >= waitTime2) {

                                arm.grab();

                                waitTimer3.reset();
                                rstate = 10;
                            }
                            break;
                        case 10:
                            if (waitTimer3.seconds() >= waitTime3) {

                                arm.aftergrab();

                                waitTimer4.reset();
                                rstate = 11;
                            }
                            break;
                        case 11:
                            if (waitTimer4.seconds() >= waitTime4) {

                                arm.bendwrist.setPosition(.148);


                                waitTimer6.reset();
                                rstate = 12;
                            }
                            break;
                        case 12:
                            if (waitTimer6.seconds() >= waitTime6) {

                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.28);
                                arm.laxon.setPosition(.72);

                                waitTimer7.reset();
                                rstate = 13;
                            }
                            break;
                        case 13:
                            if (waitTimer7.seconds() >= waitTime7) {
                                arm.rotwrist.setPosition(.5375);
                                arm.bendwrist.setPosition(.77);
                                arm.drop.setPosition(.95);
                                waitTimer7.reset();
                                rstate = 14;
                            }
                            break;
                        case 14:
                            if (!drive.isBusy()) {

                                arm.release();


                                rstate = 15;
                            }
                            break;// goes for 4
                        case 15:
                            if (!drive.isBusy()) {
                                arm.intakePos();
                                arm.lflap.setPosition(LFLAPDOWN);
                                arm.rflap.setPosition(RFLAPDOWN);
                                drive.followTrajectorySequenceAsync(right_plus4);
                                rstate = 16;
                            }
                            break;
                        case 16:
                            if (!drive.isBusy()) {

                                intake.intakewhile3();
                                drive.followTrajectory(right_intakeforward4);
                                waitTimer10.reset();


                                rstate = 17;
                            }
                            break;
                        case 17:
                            if (waitTimer10.seconds() >= waitTime10) {
                                intake.outtake(.25);
                                intake.intake3(.885);
                                intake.outtake2nd(.45);
                                drive.followTrajectorySequenceAsync(right_intakebackward4);
                                arm.lflap.setPosition(LFLAPUP);
                                arm.rflap.setPosition(RFLAPUP);
                                arm.release();


                                waitTimer8.reset();
                                rstate = 18;
                            }
                            break;
                        case 18:
                            if (!drive.isBusy() && waitTimer8.seconds() >= waitTime8) {

                                drive.followTrajectorySequenceAsync(right_backstage4drop);
                                arm.raxon.setPosition(.783);
                                arm.laxon.setPosition(.217);
                                arm.bendwrist.setPosition(.159);


                                waitTimer2.reset();
                                rstate = 19;
                            }
                            break;
                        case 19:
                            if (waitTimer2.seconds() >= waitTime2) {

                                arm.grab();

                                waitTimer3.reset();
                                rstate = 20;
                            }
                            break;
                        case 20:
                            if (waitTimer3.seconds() >= waitTime3) {

                                arm.aftergrab();

                                waitTimer4.reset();
                                rstate = 21;
                            }
                            break;
                        case 21:
                            if (waitTimer4.seconds() >= waitTime4) {

                                arm.bendwrist.setPosition(.148);


                                waitTimer6.reset();
                                rstate = 22;
                            }
                            break;
                        case 22:
                            if (waitTimer6.seconds() >= waitTime6) {

                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.28);
                                arm.laxon.setPosition(.72);

                                waitTimer7.reset();
                                rstate = 23;
                            }
                            break;
                        case 23:
                            if (waitTimer7.seconds() >= waitTime7) {
                                arm.rotwrist.setPosition(.685);
                                arm.bendwrist.setPosition(.77);
                                arm.drop.setPosition(.95);
                                waitTimer7.reset();
                                rstate = 24;
                            }
                            break;
                        case 24:
                            if (!drive.isBusy()) {

                                arm.release();


                                rstate = 25;
                            }
                        case 25:
                            if (!drive.isBusy()) {

                                arm.intakePos();

                                waitTimer9.reset();
                                rstate = 26;
                            }
                            break;
                        case 26:
                            if (waitTimer9.seconds() >= waitTime9) {

                                rdone = true;
                                break;
                            }

                    }
                    drive.update();
                    lift.update(pos);
                }

                break;
        }

    }
}

