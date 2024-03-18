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


@Autonomous(name="Red_AUD_2+3_TRUSS", group="Auto")
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
    double waitTime10 = 2.6;
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
                .splineTo(new Vector2d(-45.5, -40), Math.toRadians(90))
                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-37.5, -36.5), Math.toRadians(90))
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
                .back(6.09, SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                .splineTo(new Vector2d(42.52, -30.35), Math.toRadians(0.00))
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
                .forward(5.9)
                .build();
        Trajectory away_left = drive.trajectoryBuilder(deposit_left.end())
                .back(4.5)
                .build();
        Trajectory left_park = drive.trajectoryBuilder(away_left.end())
                .strafeLeft(15)
                .build();

        Trajectory backup_right = drive.trajectoryBuilder(right.end())
                .back(13)
                .build();


        Trajectory right_driveintake = drive.trajectoryBuilder(backup_right.end())
                .lineToLinearHeading(new Pose2d(-55, -36.1, Math.toRadians(0.00)))
                .build();
        Trajectory right_straight = drive.trajectoryBuilder(right_driveintake.end())
                .back(4.055)
                .build();
        Trajectory right_intakee = drive.trajectoryBuilder(right_driveintake.end())
                .back(3)
                .build();
        Trajectory right_truss = drive.trajectoryBuilder(right_straight.end())
                .lineToLinearHeading(new Pose2d(-42.5, -50, Math.toRadians(0.00)))
                .build();
        Trajectory right_strafe_ = drive.trajectoryBuilder(right_truss.end())
                .strafeRight(8.35)
                .build();

        Trajectory right_under_truss = drive.trajectoryBuilder(right_strafe_.end())
                .forward(50)
                .build();
        Trajectory rightat_back = drive.trajectoryBuilder(right_under_truss.end())
                .lineToLinearHeading(new Pose2d(46, -41.2, Math.toRadians(0.00)))
                .build();
        Trajectory rightapproach = drive.trajectoryBuilder(rightat_back.end())
                .forward(4.65)
                .build();
        Trajectory away_right = drive.trajectoryBuilder(rightapproach.end())
                .back(5.8)
                .build();
        Trajectory threeright = drive.trajectoryBuilder(away_right.end())
                .strafeRight(16.5)
                .build();
        Trajectory backstraightright = drive.trajectoryBuilder(threeright.end())
                .back(45)
                .build();
        Trajectory right_park = drive.trajectoryBuilder(backstraightright.end())
                .strafeRight(16)
                .build();
        Trajectory backup_middle = drive.trajectoryBuilder(middle.end())
                .back(2)
                .build();
        Trajectory strafe_middle = drive.trajectoryBuilder(backup_middle.end())
                .strafeLeft(16)
                .build();
        Trajectory stage_middle = drive.trajectoryBuilder(strafe_middle.end())
                .splineTo(new Vector2d(-42.88, -10.68), Math.toRadians(0.00))
                .build();
        Trajectory straight_middle = drive.trajectoryBuilder(stage_middle.end())
                .forward(57)
                .build();
        Trajectory drop_middle = drive.trajectoryBuilder(straight_middle.end())
                .splineTo(new Vector2d(39.17, -34.4), Math.toRadians(0.00))
                .build();
        Trajectory deposit_middle = drive.trajectoryBuilder(drop_middle.end())
                .forward(5.8)
                .build();
        Trajectory away_middle = drive.trajectoryBuilder(deposit_middle.end())
                .back(5.5)
                .build();
        Trajectory middle_after = drive.trajectoryBuilder(away_middle.end())
                .strafeLeft(25)
                .build();
        Trajectory middle_park = drive.trajectoryBuilder(middle_after.end())
                .forward(7.7)
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
                        case 6:
                            if (!drive.isBusy()) {

                                pos = Lift.LiftPos.LOW_AUTOAUD;


                            waitTimer4.reset();
                                state = 7;
                            }
                            break;
                        case 7:
                            if (waitTimer4.seconds() >= waitTime4) {
                                arm.drop.setPosition(.63);
                                arm.raxon.setPosition(.3);
                                arm.laxon.setPosition(.7);


                                waitTimer6.reset();
                                state = 8;
                            }
                            break;
                        case 8:
                            if (waitTimer6.seconds() >= waitTime6) {
                                arm.rotwrist.setPosition(.985);
                                arm.bendwrist.setPosition(.705);
                                arm.drop.setPosition(.95);


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
            case NOT_FOUND: //right
                drive.followTrajectory(right);
                drive.followTrajectory(backup_right);
                //drive.turn(Math.toRadians(-67.5));
                drive.followTrajectory(right_driveintake);
                intake.intakewhile();
                // setIntake();
                drive.followTrajectory(right_intakee);
                drive.followTrajectory(right_straight);
                sleep(400);
                intake.stopintake();
                grab();

                // setIntake();
                drive.followTrajectory(right_truss);
                drive.followTrajectory(right_strafe_);
                drive.followTrajectory(right_under_truss);
                drive.followTrajectory(rightat_back);
                intake.score();
                arm.drop.setPosition(466);
                //arm.out();
                lift.moveToTarget(Lift.LiftPos.LOW_AUTO);
                drive.followTrajectory(rightapproach);
                arm.release();
                sleep(200);
                drive.followTrajectory(away_right);
//                arm.intakePosafterscore();
//                lift.moveToTarget(Lift.LiftPos.START);
                drive.followTrajectory(threeright);
                drive.followTrajectory(backstraightright);

//                drive.followTrajectory(right_stage);
//                drive.followTrajectory(right_straight);
//                drive.followTrajectory(right_drop);
//                scoreLow(deposit_right, away_right);
//                drive.followTrajectory(after_right);
                // drive.followTrajectory(right_park);
                break;
            case RIGHT: //middle
                drive.followTrajectory(middle);
                drive.followTrajectory(backup_middle);
                drive.followTrajectory(strafe_middle);
//                drive.followTrajectory(stage_middle);
//                drive.followTrajectory(straight_middle);
//                drive.followTrajectory(drop_middle);
//                scoreLow(deposit_middle, away_middle);
//                drive.followTrajectory(middle_after);
                // drive.followTrajectory(middle_park);
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



