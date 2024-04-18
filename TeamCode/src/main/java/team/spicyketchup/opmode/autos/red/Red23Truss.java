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
import team.spicyketchup.subsystem.tests.redAudiencePipeline;

//import com.acmerobotics.roadrunner.geometry.Pose2d;


//import com.acmerobotics.roadrunner.trajectoryBuilder;


@Autonomous(name="Red_AUD_2+3_TRUSS", group="Auto")
public class Red23Truss extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenCvCamera webcam;
    Lift lift;
    Arm arm;

    Intake intake;
    //Arm arm;

    double LFLAPUP = .465;
    double LFLAPDOWN = .57;
    double RFLAPUP = .512;
    double RFLAPDOWN = .4309;
    double FRONTRELEASE = .472;
    double REARRELEASE = .235;
    double waitTime1 = 1.875;
    double waitTime2 = 0.57;
    double waitTime3 = 0.65;
    double waitTime4 = .01;
    double waitTime6 = .18;
    double waitTime7 = .4;
    double waitTime8 = .5;
    double waitTime9 = .5;
    double waitTime10 = 1.25;
    double waitTime11 = .5;
    double waitTime12 = .01;
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

    ElapsedTime waitTimer12 = new ElapsedTime();

    ElapsedTime runtime = new ElapsedTime();

    public enum START_POSITION {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        redAudiencePipeline detector = new redAudiencePipeline(telemetry);
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
        arm.intakePostele();
        arm.drop.setPosition(.9);

        Pose2d startPose = new Pose2d(-40, -63.42, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        // Pose2d rightintakepos = new Pose2d(-40, -63.42, Math.toRadians(-165));


        Trajectory left = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-45.99, -40), Math.toRadians(90))
                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-36.5, -35), Math.toRadians(90))
                .build();


        Trajectory right = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-31.5, -39.5), Math.toRadians(50.00))
                .build();



        Trajectory backup_left = drive.trajectoryBuilder(left.end())
                .back(7.5)
                .build();
        Trajectory leftatintake = drive.trajectoryBuilder(backup_left.end())
                .lineToLinearHeading(new Pose2d(-56.2, -41, Math.toRadians(-25.00)))
                .build();

        Trajectory left_intakeforward = drive.trajectoryBuilder(leftatintake.end())
                .back(5.5, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory leftintakeback1 = drive.trajectoryBuilder(left_intakeforward.end())
                .forward(7)
                .build();

        Trajectory lefttruss= drive.trajectoryBuilder(leftintakeback1.end())
                .splineTo(new Vector2d(-29.48, -57), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(21.44, -57.9), Math.toRadians(0.00))
              //  .splineTo(new Vector2d(42.52, -31.44), Math.toRadians(0.00))
                .build();
        TrajectorySequence leftbackdrp= drive.trajectorySequenceBuilder(lefttruss.end())
                .splineToConstantHeading(new Vector2d(21.44, -56.5), Math.toRadians(0.00))
                .splineTo(new Vector2d(42.52, -25.25), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(21.44, -57.9), Math.toRadians(0.00))
                //  .splineTo(new Vector2d(42.52, -31.44), Math.toRadians(0.00))
                .build();

//        Trajectory left_truss = drive.trajectoryBuilder(left_intakeforward.end())
//                .splineTo(new Vector2d(-24.76, -15.86), Math.toRadians(0.00))
//                .build();
//        Trajectory left_straight = drive.trajectoryBuilder(left_truss.end())
//                .forward(47)
//                .build();
//
//        Trajectory left_drop = drive.trajectoryBuilder(left_straight.end())
//                .splineTo(new Vector2d(41.24, -34), Math.toRadians(0.00))
//                .build();
        Trajectory deposit_left = drive.trajectoryBuilder(leftbackdrp.end())
                .forward(6.7)
                .build();
        Trajectory away_left = drive.trajectoryBuilder(deposit_left.end())
                .back(4.5)
                .build();
        Trajectory left_park = drive.trajectoryBuilder(away_left.end())
                .strafeLeft(15)
                .build();




        // hereeeeeeeeeeee
        Trajectory backup_middle = drive.trajectoryBuilder(middle.end())
                .back(7.5)
                .build();
        Trajectory middleatintake = drive.trajectoryBuilder(backup_middle.end())
                .lineToLinearHeading(new Pose2d(-56.2, -41, Math.toRadians(-25.00)))
                .build();

        Trajectory middle_intakeforward = drive.trajectoryBuilder(middleatintake.end())
                .back(4.8, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory middleintakeback1 = drive.trajectoryBuilder(middle_intakeforward.end())
                .forward(7)
                .build();

        Trajectory middletruss= drive.trajectoryBuilder(middleintakeback1.end())
                .splineTo(new Vector2d(-29.48, -56), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(21.44, -57.9), Math.toRadians(0.00))
                //  .splineTo(new Vector2d(42.52, -31.44), Math.toRadians(0.00))
                .build();
        TrajectorySequence middlebackdrp= drive.trajectorySequenceBuilder(middletruss.end())
                .splineToConstantHeading(new Vector2d(21.44, -55.5), Math.toRadians(0.00))
                .splineTo(new Vector2d(42.52, -32.75), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(21.44, -57.9), Math.toRadians(0.00))
                //  .splineTo(new Vector2d(42.52, -31.44), Math.toRadians(0.00))
                .build();

//
        Trajectory deposit_middle = drive.trajectoryBuilder(middlebackdrp.end())
                .forward(7.55)
                .build();
        Trajectory away_middle = drive.trajectoryBuilder(deposit_middle.end())
                .back(4.5)
                .build();

        Trajectory backup_right = drive.trajectoryBuilder(right.end())
                .back(7.5)
                .build();

        Trajectory rightatintake = drive.trajectoryBuilder(backup_right.end())
                .lineToLinearHeading(new Pose2d(-55, -36.8, Math.toRadians(0.00)))
                .build();

        Trajectory right_intakeforward = drive.trajectoryBuilder(rightatintake.end())
                .back(4.825, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory rightintakeback1 = drive.trajectoryBuilder(right_intakeforward.end())
                .forward(7)
                .build();

        Trajectory righttruss= drive.trajectoryBuilder(rightintakeback1.end())
                .splineTo(new Vector2d(-29.48, -55.4), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(21.44, -57.9), Math.toRadians(0.00))
                //  .splineTo(new Vector2d(42.52, -31.44), Math.toRadians(0.00))
                .build();
        TrajectorySequence rightbackdrp= drive.trajectorySequenceBuilder(righttruss.end())
                .splineToConstantHeading(new Vector2d(21.44, -56.85), Math.toRadians(0.00))
                .splineTo(new Vector2d(42.52, -37.5), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(21.44, -57.9), Math.toRadians(0.00))
                //  .splineTo(new Vector2d(42.52, -31.44), Math.toRadians(0.00))
                .build();


//
        Trajectory deposit_right = drive.trajectoryBuilder(rightbackdrp.end())
                .forward(8.75)
                .build();
        Trajectory away_right = drive.trajectoryBuilder(deposit_right.end())
                .back(4.5)
                .build();
        TrajectorySequence rightundertruss = drive.trajectorySequenceBuilder(away_right.end())
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(12, -56.2), Math.PI)
                .splineTo(new Vector2d(-30, -56.2), Math.toRadians(180))
                .splineTo(new Vector2d(-54.5, -31.95), Math.toRadians(180))
                .build();
        Trajectory right_intakeforward3 = drive.trajectoryBuilder(rightundertruss.end())
                .back(2.57, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory rightintakeback3 = drive.trajectoryBuilder(right_intakeforward3.end())
                .forward(7)
                .build();

        TrajectorySequence right_backstage2drop = drive.trajectorySequenceBuilder(rightintakeback3.end())
//                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(46.5, -14.83), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(-29.48, -56), Math.toRadians(0.00))
//                .splineTo(new Vector2d(30, -56), Math.toRadians(0.00))
//                .splineTo(new Vector2d(42.52, -36.5), Math.toRadians(19))
                .splineTo(new Vector2d(-29.5, -53), Math.toRadians(0.00),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(17.33, -54), Math.toRadians(0.00),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(54.45, -45.54), Math.toRadians(25))
                .build();
//left_>////////////////////////

        TrajectorySequence leftundertruss = drive.trajectorySequenceBuilder(away_right.end())
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(12, -56.2), Math.PI)
                .splineTo(new Vector2d(-30, -56.2), Math.toRadians(180))
                .splineTo(new Vector2d(-56.2, -44), Math.toRadians(135))
                .build();
        Trajectory left_intakeforward3 = drive.trajectoryBuilder(leftundertruss.end())
                .back(7.35, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory leftintakeback3 = drive.trajectoryBuilder(left_intakeforward3.end())
                .forward(7)
                .build();

        TrajectorySequence left_backstage2drop = drive.trajectorySequenceBuilder(leftintakeback3.end())
//                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(46.5, -14.83), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(-29.48, -56), Math.toRadians(0.00))
//                .splineTo(new Vector2d(30, -56), Math.toRadians(0.00))
//                .splineTo(new Vector2d(42.52, -36.5), Math.toRadians(19))
                .splineTo(new Vector2d(-29.5, -53), Math.toRadians(0.00),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(17.33, -54), Math.toRadians(0.00),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(54, -45.6), Math.toRadians(25))
                .build();


        TrajectorySequence middleundertruss = drive.trajectorySequenceBuilder(away_right.end())
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(12, -56.2), Math.PI)
                .splineTo(new Vector2d(-30, -56.2), Math.toRadians(180))
                .splineTo(new Vector2d(-54.5, -31.95), Math.toRadians(180))

                .build();
        Trajectory middle_intakeforward3 = drive.trajectoryBuilder(middleundertruss.end())
                .back(2.45, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory middleintakeback3 = drive.trajectoryBuilder(left_intakeforward3.end())
                .forward(7)
                .build();

        TrajectorySequence middle_backstage2drop = drive.trajectorySequenceBuilder(middleintakeback3.end())
//                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(46.5, -14.83), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(-29.48, -56), Math.toRadians(0.00))
//                .splineTo(new Vector2d(30, -56), Math.toRadians(0.00))
//                .splineTo(new Vector2d(42.52, -36.5), Math.toRadians(19))
                .splineTo(new Vector2d(-29.5, -53), Math.toRadians(0.00),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(17.33, -54), Math.toRadians(0.00),SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(53.95, -45.6), Math.toRadians(25))
                .build();


        waitForStart();
        if (isStopRequested()) return;
        redAudiencePipeline.Location location = detector.getLocation();
        Lift.LiftPos pos = Lift.LiftPos.LOW_AUTO;

        switch (location) {
            case LEFT: //left
//
                int state = 0; ///middle
                boolean done = false;
                while (opModeIsActive() && !isStopRequested() && !done) {
                    switch (state) {
                        //middle

                        case 0:
                            pos = Lift.LiftPos.START;
                            drive.followTrajectory(left);
                            drive.followTrajectory(backup_left);
                            drive.followTrajectoryAsync(leftatintake);
                            intake.intakewhile5();
                            //  arm.drop.setPosition(.63)
                            waitTimer1.reset();
                            state = 1;
                            break;
                        case 1:
                            if (!drive.isBusy()) {
                                drive.followTrajectory(left_intakeforward);
                                waitTimer1.reset();
                                state = 10;
                            }
                            break;
                        case 10:
                            if (!drive.isBusy() && waitTimer1.seconds() >= waitTime1) {

                                intake.outtake(.3);
//                                intake.intakewhile5();
//                                intake.outtake2ndaud();
                                drive.followTrajectoryAsync(leftintakeback1);
                                arm.release();
                                arm.flapsUp();

                                state = 2;
                            }
                            break;
                        case 2:
                            if (!drive.isBusy()) {
                                intake.stopIntake();
                                arm.drop.setPosition(.585);
                                drive.followTrajectoryAsync(lefttruss);
                                arm.raxon.setPosition(.784);
                                arm.laxon.setPosition(.216);
                                arm.bendwrist.setPosition(.152);
                                waitTimer2.reset();


                                state = 3;
                            }
                            break;
                        case 3:
                            if (waitTimer2.seconds() >= waitTime2) {
                            arm.grab();

                                waitTimer3.reset();
                                state = 4;
                            }
                            break;

                        case 4:
                            if (!drive.isBusy() && waitTimer3.seconds() >= waitTime3) {

                                arm.aftergrab();

                                waitTimer4.reset();
                                state = 5;
                            }
                            break;
                        case 5:
                            if (waitTimer4.seconds() >= waitTime4) {

                                arm.bendwrist.setPosition(.148);
                                drive.followTrajectorySequenceAsync(leftbackdrp);

                                waitTimer10.reset();
                                state = 6;
                            }
                            break;
                        case 6:
                            if (waitTimer10.seconds() >= waitTime10) {

                                pos = Lift.LiftPos.LOW_AUTOAUD;


                            waitTimer4.reset();
                                state = 7;
                            }
                            break;
                        case 7:
                            if (waitTimer4.seconds() >= waitTime4) {
                                arm.drop.setPosition(.65);
                                arm.raxon.setPosition(.3);
                                arm.laxon.setPosition(.7);

                                waitTimer6.reset();
                                state = 8;
                            }
                            break;
                        case 8:
                            if (waitTimer6.seconds() >= waitTime6) {
                                arm.rotwrist.setPosition(.001);
                                arm.bendwrist.setPosition(.705);
                                arm.drop.setPosition(.89);
                                //waitTimer7.reset();
                                state = 9;
                            }
                            break;
                        case 9:
                            if (!drive.isBusy()) {
                               drive.followTrajectory(deposit_left);
                                arm.release();
                                drive.followTrajectory(away_left);
                                arm.intakePosafterscore();
                                arm.intakePos();
                                pos = Lift.LiftPos.START;

                                waitTimer7.reset();
                                state = 15;
                            }
                            break;
                        case 15:
                            if (!drive.isBusy()) {
                                arm.intakePos();
                                arm.lflap.setPosition(LFLAPDOWN);
                                arm.rflap.setPosition(RFLAPDOWN);
                                drive.followTrajectorySequenceAsync(leftundertruss);
                                state = 16;
                            }
                            break;
                        case 16:
                            if (!drive.isBusy()) {

                                intake.intakewhile3();
                                drive.followTrajectory(left_intakeforward3);
                                waitTimer10.reset();


                                state = 17;
                            }
                            break;
                        case 17:
                            if (waitTimer10.seconds() >= waitTime10) {
                                intake.outtake(.25);
                                intake.intake3(.885);
                                intake.outtake2nd(.45);
                                drive.followTrajectoryAsync(leftintakeback3);
                                arm.lflap.setPosition(LFLAPUP);
                                arm.rflap.setPosition(RFLAPUP);
                                arm.release();


                                waitTimer8.reset();
                                state = 18;
                            }
                            break;
                        case 18:
                            if (!drive.isBusy() && waitTimer8.seconds() >= waitTime8) {

                                drive.followTrajectorySequenceAsync(left_backstage2drop);
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

                                arm.bendwrist.setPosition(.15);


                                waitTimer12.reset();
                                state = 22;
                            }
                            break;
                        case 22:
                            if (waitTimer12.seconds() >= waitTime12) {
                                pos = Lift.LiftPos.LOW;
                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.28);
                                arm.laxon.setPosition(.72);

                                waitTimer7.reset();
                                state = 23;
                            }
                            break;
                        case 23:
                            if (waitTimer7.seconds() >= waitTime7) {
                                arm.rotwrist.setPosition(.89);
                                arm.bendwrist.setPosition(.7);
                                arm.drop.setPosition(.82);
                                waitTimer7.reset();
                                state = 24;
                            }
                            break;
                        case 24:
                            if (!drive.isBusy() && waitTimer7.seconds() >= waitTime7) {

                                arm.release();

                                waitTimer7.reset();
                                state = 25;
                            }
                            break;
                        case 25:
                            if (!drive.isBusy() && waitTimer7.seconds() >= waitTime7) {
                                arm.intakePos();
                                pos = Lift.LiftPos.START;


                                waitTimer9.reset();
                                state = 11;
                            }
                            break;

                        case 11:
                            if (waitTimer9.seconds() >= waitTime9) {
                                done = true;
                                break;
                            }
                    }
                    drive.update();
                    lift.update(pos);
                }
                break;
            case RIGHT: //right
                int state1 = 0; ///middle
                boolean done1 = false;
                while (opModeIsActive() && !isStopRequested() && !done1) {
                    switch (state1) {
                        //middle

                        case 0:
                            pos = Lift.LiftPos.START;
                            drive.followTrajectory(middle);
                            drive.followTrajectory(backup_middle);
                            drive.followTrajectoryAsync(middleatintake);
                            intake.intakewhile5();
                            //  arm.drop.setPosition(.63)
                            waitTimer1.reset();
                            state1 = 1;
                            break;
                        case 1:
                            if (!drive.isBusy() ) {
                                drive.followTrajectory(middle_intakeforward);

                                waitTimer1.reset();
                                state1 = 10;
                            }
                            break;
                        case 10:
                            if (!drive.isBusy() && waitTimer1.seconds() >= waitTime1) {

                                intake.outtake(.3);
//                                intake.intake3(.7);
//                                intake.outtake2ndaud();
                                drive.followTrajectoryAsync(middleintakeback1);
                                arm.release();
                                arm.flapsUp();

                                state1 = 2;
                            }
                            break;
                        case 2:
                            if (!drive.isBusy()) {
                                intake.stopIntake();
                                arm.drop.setPosition(.59);
                                drive.followTrajectoryAsync(middletruss);
                                arm.raxon.setPosition(.783);
                                arm.laxon.setPosition(.217);
                                arm.bendwrist.setPosition(.159);
                                waitTimer2.reset();


                                state1 = 3;
                            }
                            break;
                        case 3:
                            if (waitTimer2.seconds() >= waitTime2) {
                                arm.grab();

                                waitTimer3.reset();
                                state1 = 4;
                            }
                            break;

                        case 4:
                            if (!drive.isBusy() && waitTimer3.seconds() >= waitTime3) {

                                arm.aftergrab();

                                waitTimer4.reset();
                                state1 = 5;
                            }
                            break;
                        case 5:
                            if (waitTimer4.seconds() >= waitTime4) {

                                arm.bendwrist.setPosition(.148);
                                drive.followTrajectorySequenceAsync(middlebackdrp);

                                waitTimer10.reset();
                                state1 = 6;
                            }
                            break;
                        case 6:
                            if (waitTimer10.seconds() >= waitTime10) {

                                pos = Lift.LiftPos.LOW_AUTOAUD;


                                waitTimer4.reset();
                                state1 = 7;
                            }
                            break;
                        case 7:
                            if (waitTimer4.seconds() >= waitTime4) {
                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.3);
                                arm.laxon.setPosition(.7);


                                waitTimer6.reset();
                                state1 = 8;
                            }
                            break;
                        case 8:
                            if (waitTimer6.seconds() >= waitTime6) {
                                arm.rotwrist.setPosition(.89);
                                arm.bendwrist.setPosition(.705);
                                arm.drop.setPosition(.89);


                                //waitTimer7.reset();
                                state1 = 9;
                            }
                            break;
                        case 9:
                            if (!drive.isBusy()) {
                                drive.followTrajectory(deposit_middle);
                                arm.release();
                                drive.followTrajectory(away_middle);
                                arm.intakePosafterscore();
                                arm.intakePos();
                                pos = Lift.LiftPos.START;

                                waitTimer7.reset();
                                state1 = 15;
                            }
                            break;
                        case 15:
                            if (!drive.isBusy()) {
                                arm.intakePos();
                                arm.lflap.setPosition(LFLAPDOWN);
                                arm.rflap.setPosition(RFLAPDOWN);
                                drive.followTrajectorySequenceAsync(middleundertruss);
                                state1 = 16;
                            }
                            break;
                        case 16:
                            if (!drive.isBusy()) {

                                intake.intakewhile3();
                                drive.followTrajectory(middle_intakeforward3);
                                waitTimer10.reset();


                                state1 = 17;
                            }
                            break;
                        case 17:
                            if (waitTimer10.seconds() >= waitTime10) {
                                intake.outtake(.25);
                                intake.intake3(.885);
                                intake.outtake2nd(.45);
                                drive.followTrajectoryAsync(middleintakeback3);
                                arm.lflap.setPosition(LFLAPUP);
                                arm.rflap.setPosition(RFLAPUP);
                                arm.release();


                                waitTimer8.reset();
                                state1 = 18;
                            }
                            break;
                        case 18:
                            if (!drive.isBusy() && waitTimer8.seconds() >= waitTime8) {

                                drive.followTrajectorySequenceAsync(middle_backstage2drop);
                                arm.raxon.setPosition(.783);
                                arm.laxon.setPosition(.217);
                                arm.bendwrist.setPosition(.159);


                                waitTimer2.reset();
                                state1 = 19;
                            }
                            break;
                        case 19:
                            if (waitTimer2.seconds() >= waitTime2) {

                                arm.grab();

                                waitTimer3.reset();
                                state1 = 20;
                            }
                            break;
                        case 20:
                            if (waitTimer3.seconds() >= waitTime3) {

                                arm.aftergrab();

                                waitTimer4.reset();
                                state1 = 21;
                            }
                            break;
                        case 21:
                            if (waitTimer4.seconds() >= waitTime4) {

                                arm.bendwrist.setPosition(.15);


                                waitTimer12.reset();
                                state1 = 22;
                            }
                            break;
                        case 22:
                            if (waitTimer12.seconds() >= waitTime12) {
                                pos = Lift.LiftPos.LOW;
                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.28);
                                arm.laxon.setPosition(.72);

                                waitTimer7.reset();
                                state1 = 23;
                            }
                            break;
                        case 23:
                            if (waitTimer7.seconds() >= waitTime7) {
                                arm.rotwrist.setPosition(.89);
                                arm.bendwrist.setPosition(.7);
                                arm.drop.setPosition(.82);
                                waitTimer7.reset();
                                state1 = 24;
                            }
                            break;
                        case 24:
                            if (!drive.isBusy() && waitTimer7.seconds() >= waitTime7) {

                                arm.release();

                                waitTimer7.reset();
                                state1 = 25;
                            }
                            break;
                        case 25:
                            if (!drive.isBusy() && waitTimer7.seconds() >= waitTime7) {
                                arm.intakePos();



                                waitTimer8.reset();
                                state1 = 26;
                            }
                            break;
                        case 26:
                            if (waitTimer8.seconds() >= waitTime8) {
                                pos = Lift.LiftPos.START;



                                waitTimer9.reset();
                                state1 = 11;
                            }
                            break;


                        case 11:
                            if (waitTimer9.seconds() >= waitTime9) {


                                done1 = true;
                                break;
                            }
                    }
                    drive.update();
                    lift.update(pos);

                }



                break;
            case  NOT_FOUND: //middle
                int state2 = 0; ///middle
                boolean done2 = false;
                while (opModeIsActive() && !isStopRequested() && !done2) {
                    switch (state2) {
                        //middle

                        case 0:
                            pos = Lift.LiftPos.START;
                            drive.followTrajectory(right);
                            drive.followTrajectory(backup_right);
                            drive.followTrajectoryAsync(rightatintake);
                            intake.intakewhile5();
                            //  arm.drop.setPosition(.63)
                            waitTimer1.reset();
                            state2 = 1;
                            break;
                        case 1:
                            if (!drive.isBusy() ) {
                                drive.followTrajectory(right_intakeforward);

                                waitTimer1.reset();
                                state2 = 10;
                            }
                            break;
                        case 10:
                            if (!drive.isBusy() && waitTimer1.seconds() >= waitTime1) {

                                intake.outtake(.3);
//                                intake.intake3(.7);
//                                intake.outtake2ndaud();
                                drive.followTrajectoryAsync(rightintakeback1);
                                arm.release();
                                arm.flapsUp();

                                state2 = 2;
                            }
                            break;
                        case 2:
                            if (!drive.isBusy()) {
                                intake.stopIntake();
                                arm.drop.setPosition(.59);
                                drive.followTrajectoryAsync(righttruss);
                                arm.raxon.setPosition(.783);
                                arm.laxon.setPosition(.217);
                                arm.bendwrist.setPosition(.1655);
                                waitTimer2.reset();


                                state2 = 3;
                            }
                            break;
                        case 3:
                            if (waitTimer2.seconds() >= waitTime2) {
                                arm.grab();

                                waitTimer3.reset();
                                state2 = 4;
                            }
                            break;

                        case 4:
                            if ( waitTimer3.seconds() >= waitTime3) {

                                arm.aftergrab();

                                waitTimer4.reset();
                                state2 = 5;
                            }
                            break;
                        case 5:
                            if (!drive.isBusy() && waitTimer4.seconds() >= waitTime4) {

                                arm.bendwrist.setPosition(.148);
                                drive.followTrajectorySequenceAsync(rightbackdrp);

                                waitTimer10.reset();
                                state2 = 6;
                            }
                            break;
                        case 6:
                            if (waitTimer10.seconds() >= waitTime10) {

                                pos = Lift.LiftPos.LOW_AUTOAUD;


                                waitTimer4.reset();
                                state2 = 7;
                            }
                            break;
                        case 7:
                            if (waitTimer4.seconds() >= waitTime4) {
                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.3);
                                arm.laxon.setPosition(.7);


                                waitTimer6.reset();
                                state2 = 8;
                            }
                            break;
                        case 8:
                            if (waitTimer6.seconds() >= waitTime6) {
                                arm.rotwrist.setPosition(.89);
                                arm.bendwrist.setPosition(.705);
                                arm.drop.setPosition(.89);


                                //waitTimer7.reset();
                                state2 = 9;
                            }
                            break;
                        case 9:
                            if (!drive.isBusy()) {
                                drive.followTrajectory(deposit_right);
                                arm.release();
                                drive.followTrajectory(away_right);
                                arm.intakePosafterscore();
                                arm.intakePos();
                                pos = Lift.LiftPos.START;

                                waitTimer7.reset();
                                state2 = 15;
                            }
                            break;

                        case 15:
                            if (!drive.isBusy()) {
                                arm.intakePos();
                                arm.lflap.setPosition(LFLAPDOWN);
                                arm.rflap.setPosition(RFLAPDOWN);
                                drive.followTrajectorySequenceAsync(rightundertruss);
                                state2 = 16;
                            }
                            break;
                        case 16:
                            if (!drive.isBusy()) {

                                intake.intakewhile3();
                                drive.followTrajectory(right_intakeforward3);
                                waitTimer10.reset();


                                state2 = 17;
                            }
                            break;
                        case 17:
                            if (waitTimer10.seconds() >= waitTime10) {
                                intake.outtake(.25);
                                intake.intake3(.885);
                                intake.outtake2nd(.45);
                                drive.followTrajectoryAsync(rightintakeback3);
                                arm.lflap.setPosition(LFLAPUP);
                                arm.rflap.setPosition(RFLAPUP);
                                arm.release();


                                waitTimer8.reset();
                                state2 = 18;
                            }
                            break;
                        case 18:
                            if (!drive.isBusy() && waitTimer8.seconds() >= waitTime8) {

                                drive.followTrajectorySequenceAsync(right_backstage2drop);
                                arm.raxon.setPosition(.783);
                                arm.laxon.setPosition(.217);
                                arm.bendwrist.setPosition(.159);


                                waitTimer2.reset();
                                state2 = 19;
                            }
                            break;
                        case 19:
                            if (waitTimer2.seconds() >= waitTime2) {

                                arm.grab();

                                waitTimer3.reset();
                                state2 = 20;
                            }
                            break;
                        case 20:
                            if (waitTimer3.seconds() >= waitTime3) {

                                arm.aftergrab();

                                waitTimer4.reset();
                                state2 = 21;
                            }
                            break;
                        case 21:
                            if (waitTimer4.seconds() >= waitTime4) {

                                arm.bendwrist.setPosition(.15);


                                waitTimer12.reset();
                                state2 = 22;
                            }
                            break;
                        case 22:
                            if (waitTimer12.seconds() >= waitTime12) {
                                pos = Lift.LiftPos.LOW;
                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.28);
                                arm.laxon.setPosition(.72);

                                waitTimer7.reset();
                                state2 = 23;
                            }
                            break;
                        case 23:
                            if (waitTimer7.seconds() >= waitTime7) {
                                arm.rotwrist.setPosition(.89);
                                arm.bendwrist.setPosition(.7);
                                arm.drop.setPosition(.82);
                                waitTimer7.reset();
                                state2 = 24;
                            }
                            break;
                        case 24:
                            if (!drive.isBusy() && waitTimer7.seconds() >= waitTime7) {

                                arm.release();

                                waitTimer7.reset();
                                state2 = 25;
                            }
                            break;
                        case 25:
                            if (!drive.isBusy() && waitTimer7.seconds() >= waitTime7) {
                                arm.intakePos();
                                pos = Lift.LiftPos.START;


                                waitTimer9.reset();
                                state2 = 13;
                            }
                            break;

                        case 13:
                            if (waitTimer9.seconds() >= waitTime9) {


                                done2 = true;
                                break;
                            }
                    }
                    drive.update();
                    lift.update(pos);

                }



                break;


        }




        webcam.stopStreaming();
    }
    public void grab(){

//        intake.intake(2.9);//grab

        intake.outtake(0.5);//grab
        arm.downpixel();//grab
        sleep(1350);//grab
        arm.grab();//grab
        sleep(500);//grab
        arm.drop.setPosition(466);
        arm.aftergrab();//grab

    }
}



