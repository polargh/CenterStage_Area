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
@Autonomous(name="Red_BACK_two+four_back", group="Auto")
public class Drop_RED extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenCvCamera webcam;
    Lift lift;
    Arm arm;
    Intake intake;
    double LFLAPUP = .465;
    double LFLAPDOWN = .57;
    double RFLAPUP = .5;
    double RFLAPDOWN = .4309;
    double waitTime1 = .5;
    double waitTime2 = 0.15;
    double waitTime3 = 1.05;
    double waitTime4 = .65;
    double waitTime6 = .45;
    double waitTime7 = .45;
    double waitTime8 = .25;
    private MultipleTelemetry tl;

    ElapsedTime waitTimer1 = new ElapsedTime();
    ElapsedTime waitTimer2 = new ElapsedTime();
    ElapsedTime waitTimer3 = new ElapsedTime();
    ElapsedTime waitTimer4 = new ElapsedTime();

    ElapsedTime waitTimer6 = new ElapsedTime();

    ElapsedTime waitTimer7 = new ElapsedTime();
    ElapsedTime waitTimer8 = new ElapsedTime();

    ElapsedTime runtime = new ElapsedTime();
//

    public enum START_POSITION {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    public enum elbowDownState { //INTAKE
        START,
        MID,
        ALMOST,
        INTAKE,
        WRIST,
        AFTERINTAKE,
        IDLE

    }

    public enum grab { //INTAKE
        START,

        DOWN,
        PICKPIXELS,
        UP,
        IDLE


    }


    public enum elbowUpState { //OUTTAKE no lift
        START,
        OUTTAKE,
        WRIST,
        IDLE


    }

    public enum Outtakelift { //OUTTAKE lift
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
                .splineTo(new Vector2d(22.69, -40), Math.toRadians(90.00))
                .build();
        Trajectory middle = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(13.8, -33.7), Math.toRadians(90.00), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory middleback = drive.trajectoryBuilder(middle.end())
                .back(12)
                .build();
        Trajectory middledropyellow = drive.trajectoryBuilder(middleback.end())
                .splineTo(new Vector2d(42.4, -33.35), Math.toRadians(0.00))
                .build();

//        TrajectorySequence middleafter = drive.trajectorySequenceBuilder(startPose)
//                .splineTo(new Vector2d(14, -41.5), Math.toRadians(90.00))
//                .splineToConstantHeading(new Vector2d(20.37, -52), Math.toRadians(90.00))
//                .splineTo(new Vector2d(42.4, -33.4), Math.toRadians(0.00))
//                .build();

        Trajectory left = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(8.93, -39.13), Math.toRadians(140.00))
                .build();
        Trajectory deposit_middle = drive.trajectoryBuilder(middledropyellow.end())
                .forward(7)
                .build();
        Trajectory away_middle = drive.trajectoryBuilder(deposit_middle.end())
                .back(5)
                .build();
        TrajectorySequence middleplus2 = drive.trajectorySequenceBuilder(away_middle.end())
               // .splineToConstantHeading(new Vector2d(35.91, -13.4), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(27, -13.4), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(-40, -13.4), Math.toRadians(180))
                .build();

        Trajectory middle_intakeforward = drive.trajectoryBuilder(middleplus2.end())
                .splineToConstantHeading(new Vector2d(-54.5, -11), Math.toRadians(0.00))
                .build();
        Trajectory middle_intakebackward = drive.trajectoryBuilder(middle_intakeforward.end())
                .splineToConstantHeading(new Vector2d(-52, -10), Math.toRadians(0.00))
                .build();
        TrajectorySequence middle_backstage2drop = drive.trajectorySequenceBuilder(middle_intakebackward.end())
                .splineToConstantHeading(new Vector2d(47.8, -14.83), Math.toRadians(0.00))
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
        Trajectory away_right = drive.trajectoryBuilder(deposit_right.end())
                .back(5)
                .build();
        Trajectory right_park = drive.trajectoryBuilder(away_right.end())
                // .strafeRight(15)
                .strafeLeft(33.5)
                .build();

        Trajectory backup_left = drive.trajectoryBuilder(middleplus2.end())
                .back(5)
                .build();
        Trajectory left_drop = drive.trajectoryBuilder(backup_left.end())
                .lineToLinearHeading(new Pose2d(44.24, -31.2, Math.toRadians(0.00)))
                .build();

        Trajectory deposit_left = drive.trajectoryBuilder(left_drop.end())
                .forward(6.8)
                .build();
        Trajectory away_left = drive.trajectoryBuilder(deposit_left.end())
                .back(5)
                .build();
        Trajectory left_park = drive.trajectoryBuilder(away_left.end())
                // .strafeRight(30)
                .strafeLeft(16.9)
                .build();

        waitForStart();
        if (isStopRequested()) return;
        detector_2_ranges.Location location = detector.getLocation();
        switch (location) {
            case LEFT: //middle

                // put state machine here


                drive.followTrajectory(middle);
                deposityellow = Outtakelift.START;
                drive.followTrajectory(middleback);
                drive.followTrajectory(middledropyellow);
//                lift.moveToTarget(Lift.LiftPos.LOW_AUTO);
//                arm.outyellowp1();
//                arm.outyellowp2();
               // sleep(200);
                drive.followTrajectory(deposit_middle);
                arm.release();
                drive.followTrajectory(away_middle);
                intakepos = elbowDownState.START;


//                lift.moveToTarget(Lift.LiftPos.START);
//                arm.afterdropintake();
//                drive.followTrajectorySequence(middleplus2);
//                intake.intakewhile5();
//                drive.followTrajectory(middle_intakeforward);
//                sleep(1400);
//                drive.followTrajectory(middle_intakebackward);
//                intake.stopintake();
//                intake.outtake(.75);
//                arm.flapsup();
//                sleep(200);
//                arm.downpixel();
//                arm.grab();
//                arm.aftergrab();
//                drive.followTrajectorySequence(middle_backstage2drop);

////                arm.outyellowp1();
//                arm.outyellowp2();
//                arm.drop();
//                sleep(300);
//                arm.intakePosafterscore();
//                sleep(100);
//                arm.intakePos();

                // drive.followTrajectorySequence(middleintake_2);

//                drive.followTrajectory(backup_middle);
//                drive.followTrajectory(drop_middle);
                // scoreLow(deposit_middle, away_middle, middle_park);

                break;
            case NOT_FOUND: //left
                drive.followTrajectory(left);
                drive.followTrajectory(backup_left);
                drive.followTrajectory(left_drop);
                //scoreLow(deposit_left, away_left, left_park);

                break;
            case RIGHT: //right
                drive.followTrajectory(right);
                drive.followTrajectory(backup_right);
                drive.followTrajectory(right_drop);
                // scoreLow(deposit_right, away_right, right_park);

                break;
        }

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            if (isStopRequested()) return;

            switch (deposityellow) { // yellow
                case START:
                    if (drive.isBusy()) {
                        lift.moveToTarget(Lift.LiftPos.LOW_AUTO);
                        arm.drop.setPosition(.63);
                        arm.bendwrist.setPosition(.148);
                        waitTimer1.reset();
                        outtake = Drop_RED.elbowUpState.OUTTAKE; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case OUTTAKE:
                    if (waitTimer1.seconds() >= waitTime1) {
                        arm.drop.setPosition(.63);
                        arm.raxon.setPosition(.3);
                        arm.laxon.setPosition(.7);
//                        robot.laxon.setPosition(.92);
                        waitTimer1.reset();
                        outtake = Drop_RED.elbowUpState.WRIST;
                    }
                    break;
                case WRIST:
                    if (waitTimer4.seconds() >= waitTime4) {

                        arm.rotwrist.setPosition(.685);
                        arm.bendwrist.setPosition(.705);


                        arm.drop.setPosition(.95);

                        waitTimer4.reset();
                        outtake = Drop_RED.elbowUpState.IDLE;
                    }
                    break;
                case IDLE:

                    break;

            }

            switch (outtake) { // scoring pos no lift, backstage
                case START:
                    if (!drive.isBusy()) {
                        arm.drop.setPosition(.63);
                        arm.bendwrist.setPosition(.145);
                        waitTimer1.reset();
                        outtake = Drop_RED.elbowUpState.OUTTAKE; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case OUTTAKE:
                    if (waitTimer1.seconds() >= waitTime1) {
                        arm.drop.setPosition(.63);
                        arm.raxon.setPosition(.3);
                        arm.laxon.setPosition(.7);
//                        robot.raxon.setPosition(.23);
//                        robot.laxon.setPosition(.77);
//
                        waitTimer1.reset();
                        outtake = Drop_RED.elbowUpState.WRIST;
                    }
                    break;
                case WRIST:
                    if (waitTimer4.seconds() >= waitTime4) {

                        arm.rotwrist.setPosition(.595);
                        arm.bendwrist.setPosition(.682);
                        arm.lflap.setPosition(LFLAPDOWN);
                        arm.rflap.setPosition(RFLAPDOWN);


                        arm.drop.setPosition(.89);

                        waitTimer4.reset();
                        outtake = Drop_RED.elbowUpState.IDLE;
                    }
                    break;
                case IDLE:

                    break;

            }

            switch (intakepos) { // start pos
                case START:
                    if (!drive.isBusy()) {
                        lift.moveToTarget(Lift.LiftPos.START);
                        arm.lflap.setPosition(LFLAPDOWN);
                        arm.rflap.setPosition(RFLAPDOWN);
                        arm.rotwrist.setPosition(.685);
                        arm.bendwrist.setPosition(.151);
                        arm.release();
                        arm.drop.setPosition(.63);
                        waitTimer2.reset();
                        intakepos = Drop_RED.elbowDownState.MID; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case MID:
                    if (waitTimer2.seconds() >= waitTime2) {
                        arm.raxon.setPosition(.55);
                        arm.laxon.setPosition(.45);
//                        robot.raxon.setPosition(.3);
//                        robot.laxon.setPosition(.7);


                        waitTimer2.reset();
                        intakepos = Drop_RED.elbowDownState.INTAKE;
                    }
                    break;
                case ALMOST:
                    if (waitTimer2.seconds() >= waitTime2) {
                        arm.raxon.setPosition(.6);
                        arm.laxon.setPosition(.4);
                        arm.bendwrist.setPosition(.153);
//                        robot.raxon.setPosition(.3);
//                        robot.laxon.setPosition(.7);


                        waitTimer6.reset();
                        intakepos = Drop_RED.elbowDownState.INTAKE;
                    }
                    break;
                case INTAKE:
                    if (waitTimer6.seconds() >= waitTime6) {
                        arm.rotwrist.setPosition(.497);
                        // robot.drop.setPosition(.89);
                        arm.raxon.setPosition(.66);
                        arm.laxon.setPosition(.34);
                        arm.bendwrist.setPosition(.157);

                        intakepos = Drop_RED.elbowDownState.AFTERINTAKE;
                        waitTimer8.reset();
                    }
                    break;
                case AFTERINTAKE:
                    if (waitTimer8.seconds() >= waitTime8) {

                        arm.drop.setPosition(.89);

                        intakepos = Drop_RED.elbowDownState.IDLE;

                    }
                    break;
                case IDLE:

                    break;

            }

            switch (claw) { //grab from transfer
                case START:
                    if (!drive.isBusy()) {
                        arm.release();
                        arm.lflap.setPosition(LFLAPUP);
                        arm.rflap.setPosition(RFLAPUP);

                        waitTimer6.reset();
                        claw = Drop_RED.grab.DOWN; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case DOWN:
                    if (waitTimer6.seconds() >= waitTime6) {
                        arm.raxon.setPosition(.783);
                        arm.laxon.setPosition(.217);
                        arm.bendwrist.setPosition(.159);
                        //  robot.drop.setPosition(.63);
                        waitTimer6.reset();
                        claw = Drop_RED.grab.PICKPIXELS;
                    }
                    break;
                case PICKPIXELS:
                    if (waitTimer6.seconds() >= waitTime6) {
                        arm.grab();

                        waitTimer7.reset();
                        claw = Drop_RED.grab.UP;
                    }
                    break;
                case UP:
                    if (waitTimer7.seconds() >= waitTime7) {
                        arm.raxon.setPosition(.66);
                        arm.laxon.setPosition(.34);
                        arm.bendwrist.setPosition(.15);
                        claw = Drop_RED.grab.IDLE;
                    }
                    break;
                    case IDLE:

                    break;

            }


            webcam.stopStreaming();
        }
//    public void scoreLow(Trajectory backdrop, Trajectory away, Trajectory park){
//
//
//
//        arm.goToScoringPos();
//        lift.moveToTarget(Lift.LiftPos.LOW_AUTO);
//
//        drive.followTrajectory(backdrop);
//        arm.deposit(1);
//        drive.followTrajectory(away);
//        arm.autonparkpos();
//        drive.followTrajectory(park);
//
//        lift.moveToTarget(Lift.LiftPos.START);
//        arm.intakePos();
//
//
//    }
    }
}



