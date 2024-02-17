package CompCode;


//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.util.ElapsedTime;
//import com.arcrobotics.ftclib.controller.PIDController;

        import Hardware.v2bot_map;

@TeleOp(name="center_tele")
public class coolTele extends LinearOpMode {


    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode

    //TELEmap robot   = new TELEmap();
    v2bot_map robot = new v2bot_map();

    ElapsedTime toggleTimer = new ElapsedTime();
    double toggleTime = .25;

    double dropstack = 5;
    double stackpos = 0;

    double FRONTGRAB = .2729;
    double REARGRAB = .413;
    double FRONTRELEASE = .472;
    double REARRELEASE = .235;

    double rotate_pos = 3;
    // used with the dump servo, this will get covered in a bit
//    ElapsedTime toggleTimer = new ElapsedTime();
    //double toggleTime = .25;


    // public static int HIGH = 500; //2900 = HIGH
//        public ElapsedTime runtime = new ElapsedTime();
//    public PIDController controller;
//    public static double p = 0.005, i = 0, d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
//    public static double f = 0.0007;  // prevents arm from falling from gravity
//
//
//    public enum LiftPos {
//        START,
//        LOW,
//        MID,
//        MIDHIGH,
//        LOW_AUTO,
//        HIGH,
//        MANUAL
//    }
//
//    public DcMotorEx larm;
//    public DcMotorEx rarm;
//    public static int START_POS = 0;
//    public static int LOW_POS = 1030;
//    public static int MID_POS = 1460;
//    public static int MID_HIGH_POS = 1750;
//    //max 2320
//    public static int HIGH_POS = 2250;
//    public static int LOW_AUTO = 900;
//    public static int liftTarget = 0; // target position
//
////    int MANUAL = larm.getCurrentPosition() +20;
//
//    private MultipleTelemetry tl;
//
//
////    int MANUAL = larm.getCurrentPosition() +20;
//
//    //private MultipleTelemetry tl;

    double intakepos = 0;
    double waitTime1 = .5;
    double waitTime2 = 0.15;
    double waitTime3 = 1.05;
    double waitTime4 = .65;
    double waitTime6 = .45;
    double waitTime7 = .45;

    ElapsedTime waitTimer1 = new ElapsedTime();
    ElapsedTime waitTimer2 = new ElapsedTime();
    ElapsedTime waitTimer3 = new ElapsedTime();
    ElapsedTime waitTimer4 = new ElapsedTime();

    ElapsedTime waitTimer6 = new ElapsedTime();

    ElapsedTime waitTimer7 = new ElapsedTime();

    ElapsedTime runtime = new ElapsedTime();
//

    double SpeedAdjust = 1;

//    double servospeed = 0.5;
//
//    double lDropPos = 0;
//    double rDropPos = 0;

    enum elbowDownState { //INTAKE
        START,
        MID,
        INTAKE,
        WRIST

    }
    enum grab { //INTAKE
        START,
        DOWN,
        PICKPIXELS,
        UP


    }


    enum elbowUpState { //OUTTAKE
        START,
        OUTTAKE,
        WRIST


    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        // Lift lift = new Lift(hardwareMap, telemetry);


        robot.drop.setPosition(.5);
        robot.frontclaw.setPosition(FRONTRELEASE);
        robot.rearclaw.setPosition(REARRELEASE);
        robot.rotwrist.setPosition(.49);
        robot.raxon.setPosition(.785);
        robot.laxon.setPosition(.215);
        robot.bendwrist.setPosition(.16);

        waitForStart();

        if (isStopRequested()) return;

        elbowUpState outtake = elbowUpState.START;
        elbowDownState intake = elbowDownState.START;
        grab claw = grab.START;
//
//        //LiftPos liftTarget = LiftPos.START;
//        liftTarget = 0;
        while (opModeIsActive() && !isStopRequested()) {

            switch (outtake) { // scoring pos
                case START:
                    if (gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up) {
                       intakepos = 1;
                        robot.bendwrist.setPosition(.105);
                        waitTimer1.reset();
                        outtake = elbowUpState.OUTTAKE; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case OUTTAKE:
                    if(waitTimer1.seconds() >= waitTime1) {
                        intakepos = 1;
                        robot.raxon.setPosition(.5);
                        robot.laxon.setPosition(.5);
//                        robot.raxon.setPosition(.08);
//                        robot.laxon.setPosition(.92);
                        waitTimer1.reset();
                        outtake = elbowUpState.WRIST;
                    }
                    break;
                case WRIST:
                    if(waitTimer4.seconds() >= waitTime4) {

                        rotate_pos = 4;
                        robot.bendwrist.setPosition(.677);



                        intakepos = 0;

                        waitTimer4.reset();
                        outtake = elbowUpState.START;
                    }
                    break;

            }

            switch (intake) { // start pos
                case START:
                    if (gamepad2.cross) {
                        rotate_pos = 3;
                        robot.bendwrist.setPosition(.16);
                        robot.frontclaw.setPosition(FRONTRELEASE);
                        robot.rearclaw.setPosition(REARRELEASE);
                        intakepos = 1;
                        waitTimer2.reset();
                        intake = elbowDownState.MID; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case MID:
                    if(waitTimer2.seconds() >= waitTime2) {
                        robot.raxon.setPosition(.7);
                        robot.laxon.setPosition(.3);
//                        robot.raxon.setPosition(.3);
//                        robot.laxon.setPosition(.7);


                        waitTimer6.reset();
                        intake = elbowDownState.INTAKE;
                    }
                    break;
                    case INTAKE:
                    if(waitTimer6.seconds() >= waitTime6) {
                        robot.raxon.setPosition(.785);
                        robot.laxon.setPosition(.215);
                        intakepos = 0;

                        intake = elbowDownState.START;
                    }
                    break;


            }

            switch (claw) { //grab from transfer
                case START:
                    if (gamepad2.triangle) {
                        robot.frontclaw.setPosition(FRONTRELEASE);
                        robot.rearclaw.setPosition(REARRELEASE);


                        waitTimer6.reset();
                        claw = grab.DOWN; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case DOWN:
                    if(waitTimer6.seconds() >= waitTime6) {
                        robot.raxon.setPosition(.96);
                        robot.laxon.setPosition(.04);
                        robot.bendwrist.setPosition(.172);

                        waitTimer6.reset();
                        claw = grab.PICKPIXELS;
                    }
                    break;
                case PICKPIXELS:
                    if(waitTimer6.seconds() >= waitTime6) {
                        robot.frontclaw.setPosition(FRONTGRAB);
                        robot.rearclaw.setPosition(REARGRAB);

                        waitTimer7.reset();
                        claw = grab.UP;
                    }
                    break;
                case UP:
                    if(waitTimer7.seconds() >= waitTime7) {
                        robot.raxon.setPosition(.785);
                        robot.laxon.setPosition(.215);
                        robot.bendwrist.setPosition(.16);
                        claw = grab.START;
                    }
                    break;

            }



            if (gamepad1.right_trigger > 0) { //out
                robot.intake.setPower(-3);

            } else if (gamepad1.left_trigger > 0) { //in
                //servospeed = 0.5;
                intakepos = 1;
                robot.intake.setPower(3);


            }else if (gamepad1.left_bumper) {

            }else {

                robot.intake.setPower(0);
                //robot.wheel.setPower(0);
               // intakepos = 0;
            }

            if (gamepad1.right_bumper) { //out
                intakepos = 1;
                robot.intake.setPower(3);
            } else if (gamepad1.left_trigger > 0) { //in
                //servospeed = 0.5;
                intakepos = 1;
                robot.intake.setPower(3);




            } else if (gamepad1.right_trigger > 0) { //out
                robot.intake.setPower(-3);
            }else  { //in
                //servospeed = 0.5;
               // intakepos = 0;
                robot.intake.setPower(0);


            }

//            else {
//                robot.intake.setPower(0);
//                //robot.wheel.setPower(0);
//               // intakepos = 0;
//            }


            if (gamepad1.triangle) { //out
                robot.drop.setPosition(.5);

            }
            if (gamepad1.square) { //out
                robot.drop.setPosition(.385);

            }


            if (gamepad1.circle) { //out
                robot.drop.setPosition(.485);
            }
            if (gamepad1.cross) { //out
                robot.drop.setPosition(.47);
            }
            if (gamepad2.circle) { //grab rear
                robot.rearclaw.setPosition(REARGRAB);
            }
            if (gamepad2.square) { //grab front
                robot.frontclaw.setPosition(FRONTGRAB);
            }
            if (gamepad2.left_stick_button) { //release front
                robot.frontclaw.setPosition(FRONTRELEASE);
            }
            if (gamepad2.right_stick_button) { //release rear
                robot.rearclaw.setPosition(REARRELEASE);
            }

            if (gamepad2.right_bumper){
                //toggle
                if (rotate_pos<5) {
                    toggleTimer.reset();
                    rotate_pos++;
                }
                while (opModeIsActive()
                        && (toggleTimer.seconds() < toggleTime)
                ) {

                }

            } else if (gamepad2.left_bumper){
                //toggle L

                if (rotate_pos >1) {
                    toggleTimer.reset();
                    rotate_pos--;
                }
                while (opModeIsActive()
                        && (toggleTimer.seconds() < toggleTime)
                ) {

                }

            }

            if (intakepos == 1 ) {
                robot.drop.setPosition(.4655);
            }
            if (intakepos == 0) {
                robot.drop.setPosition(.5);
            }
                //start stack
               if (rotate_pos == 1 ) {
                    robot.rotwrist.setPosition(.547);
                }
                 if (rotate_pos == 2) {
                    robot.rotwrist.setPosition(.53);
                }
                if (rotate_pos == 3){
                    robot.rotwrist.setPosition(.49);
                }
                 if (rotate_pos == 4){
                    robot.rotwrist.setPosition(.4655);
                }
                if (rotate_pos == 5){
                    robot.rotwrist.setPosition(.435);
                }



            if (gamepad1.left_bumper) {
                SpeedAdjust = 4;
            } else if (gamepad1.right_bumper) {
                SpeedAdjust = 2.5;
            }

//            if (gamepad2.square) {
//                robot.raxon.setPosition(0.11); //outake
//                robot.laxon.setPosition(0.89);
//
//            }
//            if (gamepad2.circle) {
//                robot.raxon.setPosition(0.89); // intake
//                robot.laxon.setPosition(0.11);
//            }

            if (gamepad1.dpad_down) {
                robot.bendwrist.setPosition(0.11);


            }
            if (gamepad1.dpad_up) {
                robot.bendwrist.setPosition(0.89);

            }


            robot.leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
            robot.leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);

              if (gamepad2.right_trigger > 0) {
                    robot.climb.setPower(2);

                }else if (gamepad2.left_trigger > 0) {
                    robot.climb.setPower(-2);
                }else {
                    robot.climb.setPower(0);
                }


               telemetry.addData("rotate#:",rotate_pos);
              telemetry.update();
        }
    }
}
