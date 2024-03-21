package Autos;

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

import Hardware.Arm;
import Hardware.Intake;
import Hardware.Lift;
import Hardware.redAudiencePipeline;

//import com.acmerobotics.roadrunner.geometry.Pose2d;


//import com.acmerobotics.roadrunner.trajectoryBuilder;


@Autonomous(name="Red_AUD_2+1_TRUSS", group="Auto")
public class redaud50maybe extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenCvCamera webcam;
    Lift lift;
    Arm arm;

    Intake intake;
    //Arm arm;
    double FRONTRELEASE = .472;
    double REARRELEASE = .235;
    double waitTime1 = 1.5;
    double waitTime2 = 0.6;
    double waitTime3 = 0.4;
    double waitTime4 = .1;
    double waitTime6 = .18;
    double waitTime7 = 2;
    double waitTime8 = .5;
    double waitTime9 = .75;
    double waitTime10 = 1.25;
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
    public enum START_POSITION{
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
                .splineTo(new Vector2d(-36.5, -35.99), Math.toRadians(90))
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
                .splineTo(new Vector2d(-29.48, -57.9), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(21.44, -57.9), Math.toRadians(0.00))
              //  .splineTo(new Vector2d(42.52, -31.44), Math.toRadians(0.00))
                .build();
        TrajectorySequence leftbackdrp= drive.trajectorySequenceBuilder(lefttruss.end())
                .splineToConstantHeading(new Vector2d(21.44, -57.9), Math.toRadians(0.00))
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
                .forward(6.4)
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
                .back(4.93, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory middleintakeback1 = drive.trajectoryBuilder(middle_intakeforward.end())
                .forward(7)
                .build();

        Trajectory middletruss= drive.trajectoryBuilder(middleintakeback1.end())
                .splineTo(new Vector2d(-29.48, -57.9), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(21.44, -57.9), Math.toRadians(0.00))
                //  .splineTo(new Vector2d(42.52, -31.44), Math.toRadians(0.00))
                .build();
        TrajectorySequence middlebackdrp= drive.trajectorySequenceBuilder(middletruss.end())
                .splineToConstantHeading(new Vector2d(21.44, -56.799), Math.toRadians(0.00))
                .splineTo(new Vector2d(42.52, -31.1), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(21.44, -57.9), Math.toRadians(0.00))
                //  .splineTo(new Vector2d(42.52, -31.44), Math.toRadians(0.00))
                .build();

//
        Trajectory deposit_middle = drive.trajectoryBuilder(middlebackdrp.end())
                .forward(6.601)
                .build();
        Trajectory away_middle = drive.trajectoryBuilder(deposit_middle.end())
                .back(4.5)
                .build();

        Trajectory backup_right = drive.trajectoryBuilder(right.end())
                .back(7.5)
                .build();
        Trajectory rightatintake = drive.trajectoryBuilder(backup_right.end())
                .lineToLinearHeading(new Pose2d(-56.2, -41, Math.toRadians(-25.00)))
                .build();

        Trajectory right_intakeforward = drive.trajectoryBuilder(rightatintake.end())
                .back(5.77, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory rightintakeback1 = drive.trajectoryBuilder(right_intakeforward.end())
                .forward(7)
                .build();

        Trajectory righttruss= drive.trajectoryBuilder(rightintakeback1.end())
                .splineTo(new Vector2d(-29.48, -57.9), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(21.44, -57.9), Math.toRadians(0.00))
                //  .splineTo(new Vector2d(42.52, -31.44), Math.toRadians(0.00))
                .build();
        TrajectorySequence rightbackdrp= drive.trajectorySequenceBuilder(righttruss.end())
                .splineToConstantHeading(new Vector2d(21.44, -56.2), Math.toRadians(0.00))
                .splineTo(new Vector2d(42.52, -36.5), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(21.44, -57.9), Math.toRadians(0.00))
                //  .splineTo(new Vector2d(42.52, -31.44), Math.toRadians(0.00))
                .build();

//
        Trajectory deposit_right = drive.trajectoryBuilder(rightbackdrp.end())
                .forward(6.955)
                .build();
        Trajectory away_right = drive.trajectoryBuilder(deposit_right.end())
                .back(4.5)
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
                            if (!drive.isBusy() ) {
                                drive.followTrajectory(left_intakeforward);

                                waitTimer1.reset();
                                state = 10;
                            }
                            break;
                        case 10:
                            if (!drive.isBusy() && waitTimer1.seconds() >= waitTime1) {

                                intake.outtake(.3);
                                intake.intake3(.7);
                                intake.outtake2ndaud();
                                drive.followTrajectoryAsync(leftintakeback1);
                                arm.release();
                                arm.flapsup();

                                state = 2;
                            }
                            break;
                        case 2:
                            if (!drive.isBusy()) {
                                intake.stopintake();
                                arm.drop.setPosition(.9);
                                drive.followTrajectoryAsync(lefttruss);
                                arm.raxon.setPosition(.783);
                                arm.laxon.setPosition(.217);
                                arm.bendwrist.setPosition(.159);
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
                                arm.rotwrist.setPosition(.89);
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
                                state = 11;
                            }
                            break;

                        case 11:
                            if (waitTimer7.seconds() >= waitTime7) {


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
                                intake.intake3(.7);
                                intake.outtake2ndaud();
                                drive.followTrajectoryAsync(middleintakeback1);
                                arm.release();
                                arm.flapsup();

                                state1 = 2;
                            }
                            break;
                        case 2:
                            if (!drive.isBusy()) {
                                intake.stopintake();
                                arm.drop.setPosition(.9);
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
                                state1 = 11;
                            }
                            break;

                        case 11:
                            if (waitTimer7.seconds() >= waitTime7) {


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
                                intake.intake3(.7);
                                intake.outtake2ndaud();
                                drive.followTrajectoryAsync(rightintakeback1);
                                arm.release();
                                arm.flapsup();

                                state2 = 2;
                            }
                            break;
                        case 2:
                            if (!drive.isBusy()) {
                                intake.stopintake();
                                arm.drop.setPosition(.9);
                                drive.followTrajectoryAsync(righttruss);
                                arm.raxon.setPosition(.783);
                                arm.laxon.setPosition(.217);
                                arm.bendwrist.setPosition(.159);
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
                            if (!drive.isBusy() && waitTimer3.seconds() >= waitTime3) {

                                arm.aftergrab();

                                waitTimer4.reset();
                                state2 = 5;
                            }
                            break;
                        case 5:
                            if (waitTimer4.seconds() >= waitTime4) {

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
                                state2 = 11;
                            }
                            break;

                        case 11:
                            if (waitTimer7.seconds() >= waitTime7) {


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



