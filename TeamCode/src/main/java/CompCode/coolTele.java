package CompCode;


//import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import Hardware.v2bot_map;


import com.arcrobotics.ftclib.controller.PIDController;

@TeleOp(name="center_tele")
public class coolTele extends LinearOpMode {


    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode

    //TELEmap robot   = new TELEmap();
    v2bot_map robot = new v2bot_map();

    ElapsedTime toggleTimer = new ElapsedTime();
    ElapsedTime toggleTimer2 = new ElapsedTime();
    public PIDController controller;
    public static double p = 0.0055, i = 0, d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = 0.00049;
    double toggleTime = .25;
    double toggleTime2 = .25;
    public DcMotorEx larm;
    public DcMotorEx rarm;

    double dropstack = 5;
    double stackpos = 0;

    double FRONTGRAB = .335;
    double REARGRAB = .398;
    double FRONTRELEASE = .477;
    double REARRELEASE = .231;

//    double FRONTRELEASE = .477; close to grab pos
//    double REARRELEASE = .231;
    double LFLAPUP = .465;
    double LFLAPDOWN = .57;
    double RFLAPUP = .512;
    double RFLAPDOWN = .4309;


    double rotate_pos = 3;

    double drop_pos = 5;

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

    double intakepos = 4;
    double waitTime1 = .35;
    double waitTime2 = 0.15;
    double waitTime3 = 1.05;
    double waitTime4 = .3;
    double waitTime9 = .65;
    double waitTime6 = .495;
    double waitTime7 = .4;
    double waitTime8 = .25;
    double waitTime10 = .4;
    double waitTime11 = .4;
    double waitTime12 = .45; //dronelaunch
    double waitTime13 = .45; //climb
    private MultipleTelemetry tl;

    ElapsedTime waitTimer1 = new ElapsedTime();
    ElapsedTime waitTimer2 = new ElapsedTime();
    ElapsedTime waitTimer3 = new ElapsedTime();
    ElapsedTime waitTimer4 = new ElapsedTime();

    ElapsedTime waitTimer6 = new ElapsedTime();

    ElapsedTime waitTimer7 = new ElapsedTime();
    ElapsedTime waitTimer8 = new ElapsedTime();

    ElapsedTime waitTimer10 = new ElapsedTime();
    ElapsedTime waitTimer11 = new ElapsedTime();

    ElapsedTime waitTimer12 = new ElapsedTime();
    ElapsedTime waitTimer13 = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();
//

    double SpeedAdjust = 1;
    public static int liftTarget = 0;
    public static int START_POS = 0;
    public static int LOW_POS = 550;
    public static int MID_POS = 700;
    public static int MID_HIGH_POS = 1000;
    //max 2320
    public static int HIGH_POS = 1600;
    public static int LOW_AUTO = 900;
    int drone_target = 0;

//    double servospeed = 0.5;
//
//    double lDropPos = 0;
//    double rDropPos = 0;

    enum elbowDownState { //INTAKE
        START,
        MID,
        ALMOST,
        INTAKE,
        WRIST,
        AFTERINTAKE

    }
    enum grab { //INTAKE
        START,

        DOWN,
        PICKPIXELS,
        UP


    }


    enum elbowUpState { //OUTTAKE no lift
        START,
        OUTTAKE,
        WRIST


    }

    enum Outtakelift { //OUTTAKE lift
        START,
        OUTTAKE,
        WRIST


    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        Lift lift = new Lift(hardwareMap, telemetry);
        // Lift lift = new Lift(hardwareMap, telemetry);


        robot.drop.setPosition(.89);
        robot.frontclaw.setPosition(FRONTRELEASE);
        robot.rearclaw.setPosition(REARRELEASE);

        robot.lflap.setPosition(LFLAPDOWN);
        robot.rflap.setPosition(RFLAPDOWN);
//        robot.lflap.setPosition(.465); up
//        robot.rflap.setPosition(.5); up
        robot.rotwrist.setPosition(.41);
        // robot.drop.setPosition(.89);
        robot.raxon.setPosition(.66);
        robot.laxon.setPosition(.34);
        robot.bendwrist.setPosition(.1552);
        robot.drone.setPosition(.5);

        waitForStart();

        if (isStopRequested()) return;

        elbowUpState outtake = elbowUpState.START;
        elbowDownState intake = elbowDownState.START;
        grab claw = grab.START;
        Outtakelift outlift = Outtakelift.START;
//
//        //LiftPos liftTarget = LiftPos.START;
//        liftTarget = 0;
        while (opModeIsActive() && !isStopRequested()) {

            switch (outlift) { // scoring pos with lift
                case START:
                    if ( gamepad2.dpad_down || gamepad2.dpad_up) {
                        robot.drop.setPosition(.63);
                        robot.bendwrist.setPosition(.148);
                        waitTimer1.reset();
                        outtake = elbowUpState.OUTTAKE; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case OUTTAKE:
                    if(waitTimer1.seconds() >= waitTime1) {
                        robot.drop.setPosition(.63);
                        robot.raxon.setPosition(.3);
                        robot.laxon.setPosition(.7);
//                        robot.laxon.setPosition(.92);
                        waitTimer1.reset();
                        outtake = elbowUpState.WRIST;
                    }
                    break;
                case WRIST:
                    if(waitTimer4.seconds() >= waitTime4) {

                        rotate_pos = 1;
                        robot.bendwrist.setPosition(.733);



                        robot.drop.setPosition(.95);

                        waitTimer4.reset();
                        outtake = elbowUpState.START;
                    }
                    break;

            }

            switch (outtake) { // scoring pos no lift
                case START:
                    if (gamepad2.square) {
                        robot.drop.setPosition(.63);
                        robot.bendwrist.setPosition(.145);
                        waitTimer1.reset();
                        outtake = elbowUpState.OUTTAKE; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case OUTTAKE:
                    if(waitTimer1.seconds() >= waitTime1) {
                        robot.drop.setPosition(.63);
                        robot.raxon.setPosition(.3);
                        robot.laxon.setPosition(.7);
//                        robot.raxon.setPosition(.23);
//                        robot.laxon.setPosition(.77);
//
                        waitTimer1.reset();
                        outtake = elbowUpState.WRIST;
                    }
                    break;
                case WRIST:
                    if(waitTimer4.seconds() >= waitTime4) {

                        rotate_pos = 1;
                        robot.bendwrist.setPosition(.682);
                        robot.lflap.setPosition(LFLAPDOWN);
                        robot.rflap.setPosition(RFLAPDOWN);


                        robot.drop.setPosition(.89);

                        waitTimer4.reset();
                        outtake = elbowUpState.START;
                    }
                    break;

            }

            switch (intake) { // start pos
                case START:
                    if (gamepad2.cross) {

                        robot.lflap.setPosition(LFLAPDOWN);
                        robot.rflap.setPosition(RFLAPDOWN);
                        rotate_pos = 3;
                        robot.bendwrist.setPosition(.151);
                        robot.frontclaw.setPosition(FRONTRELEASE);
                        robot.rearclaw.setPosition(REARRELEASE);
                        robot.drop.setPosition(.63);
                        waitTimer2.reset();
                        intake = elbowDownState.MID; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case MID:
                    if(waitTimer2.seconds() >= waitTime2) {
                        robot.raxon.setPosition(.55);
                        robot.laxon.setPosition(.45);
//                        robot.raxon.setPosition(.3);
//                        robot.laxon.setPosition(.7);


                        waitTimer2.reset();
                        intake = elbowDownState.INTAKE;
                    }
                    break;
                case ALMOST:
                    if(waitTimer2.seconds() >= waitTime2) {
                        robot.raxon.setPosition(.6);
                        robot.laxon.setPosition(.4);
                        robot.bendwrist.setPosition(.153);
//                        robot.raxon.setPosition(.3);
//                        robot.laxon.setPosition(.7);


                        waitTimer6.reset();
                        intake = elbowDownState.INTAKE;
                    }
                    break;
                    case INTAKE:
                    if(waitTimer6.seconds() >= waitTime6) {
                       // robot.drop.setPosition(.89);
                        robot.raxon.setPosition(.66);
                        robot.laxon.setPosition(.34);
                        robot.bendwrist.setPosition(.1595);

                        intake = elbowDownState.AFTERINTAKE;
                        waitTimer8.reset();
                    }
                    break;
                case AFTERINTAKE:
                    if(waitTimer8.seconds() >= waitTime8) {

                        robot.drop.setPosition(.89);

                        intake = elbowDownState.START;

                    }
                    break;


            }

            switch (claw) { //grab from transfer
                case START:
                    if (gamepad2.triangle) {
                        robot.frontclaw.setPosition(FRONTRELEASE);
                        robot.rearclaw.setPosition(REARRELEASE);
                        robot.lflap.setPosition(LFLAPUP);
                        robot.rflap.setPosition(RFLAPUP);

                        waitTimer6.reset();
                        claw = grab.DOWN; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case DOWN:
                    if(waitTimer6.seconds() >= waitTime6) {
                        robot.raxon.setPosition(.783);
                        robot.laxon.setPosition(.217);
                        robot.bendwrist.setPosition(.161);
                      //  robot.drop.setPosition(.63);
                        waitTimer10.reset();
                        claw = grab.PICKPIXELS;
                    }
                    break;
                case PICKPIXELS:
                    if(waitTimer10.seconds() >= waitTime10) {
                        robot.frontclaw.setPosition(FRONTGRAB);
                        robot.rearclaw.setPosition(REARGRAB);

                        waitTimer7.reset();
                        claw = grab.UP;
                    }
                    break;
                case UP:
                    if(waitTimer7.seconds() >= waitTime7) {
                        robot.raxon.setPosition(.66);
                        robot.laxon.setPosition(.34);
                        robot.bendwrist.setPosition(.15);
                        claw = grab.START;
                    }
                    break;

            }



            if (gamepad1.cross) { //grab rear
                robot.lflap.setPosition(LFLAPUP);
                robot.rflap.setPosition(RFLAPUP);
            }



            if (gamepad2.circle) { //grab rear
               rotate_pos = 5;
            }
//            if (gamepad2.square) { //grab front
//                robot.frontclaw.setPosition(FRONTGRAB);
//            }
            if (gamepad2.left_stick_button) { //release front
                robot.frontclaw.setPosition(FRONTRELEASE);
            }
            if (gamepad2.right_stick_button) { //release rear
                robot.rearclaw.setPosition(REARRELEASE);
            }

            if (gamepad2.right_bumper){
                //toggle
                if (rotate_pos>-1) {
                    toggleTimer.reset();
                    rotate_pos--;
                }
                while (opModeIsActive()
                        && (toggleTimer.seconds() < toggleTime)
                ) {

                }

            } else if (gamepad2.left_bumper){
                //toggle L

                if (rotate_pos <6) {
                    toggleTimer.reset();
                    rotate_pos++;
                }
                while (opModeIsActive()
                        && (toggleTimer.seconds() < toggleTime)
                ) {

                }

            }

            if (gamepad1.dpad_down){
                //toggle
                if (intakepos<4) {
                    toggleTimer2.reset();
                    intakepos++;
                }
                while (opModeIsActive()
                        && (toggleTimer2.seconds() < toggleTime2)
                ) {

                }
            }  if (gamepad1.dpad_up){
                //toggle L

                if (intakepos>1) {
                    toggleTimer2.reset();
                    intakepos--;
                }
                while (opModeIsActive()
                        && (toggleTimer2.seconds() < toggleTime2)
                ) {

                }

            }
            if (intakepos == 4 ) {
                drop_pos = .5795;
            }
            if (intakepos == 3 ) {
               drop_pos = .63;
            }
            if (intakepos == 2 ) {
               drop_pos = .685;
            }
            if (intakepos == 1) {
                drop_pos = .89;
            }

            if (gamepad1.right_trigger > 0) { //out
                robot.intake.setPower(-3);
                robot.drop.setPosition(.89);



            } else if (gamepad1.left_trigger > 0) { //in
                //servospeed = 0.5;
                robot.drop.setPosition(drop_pos);
//               robot.lflap.setPosition(LFLAPDOWN);
//                robot.rflap.setPosition(RFLAPDOWN);
                robot.intake.setPower(4);

            }else  {

                robot.intake.setPower(0);

            }




            if (rotate_pos == -1 ) {
                robot.rotwrist.setPosition(.985);
            }
                //start stack
            if (rotate_pos == 0 ) {
                robot.rotwrist.setPosition(.89);
            }
               if (rotate_pos == 1 ) {
                    robot.rotwrist.setPosition(.7);
                }
                 if (rotate_pos == 2) {
                    robot.rotwrist.setPosition(.5375); //
                }
                if (rotate_pos ==3){
                    robot.rotwrist.setPosition(.41); //
                }
                 if (rotate_pos == 4){
                    robot.rotwrist.setPosition(.32);
                }
                if (rotate_pos == 5){
                    robot.rotwrist.setPosition(.15);
                }
            if (rotate_pos == 6){
                robot.rotwrist.setPosition(.001);
            }

//            if (rotate_pos == 1 ) {
//                robot.rotwrist.setPosition(.4);
//            }
//            if (rotate_pos == 2) {
//                robot.rotwrist.setPosition(.53);
//            }
//            if (rotate_pos == 3){
//                robot.rotwrist.setPosition(.49);
//            }
//            if (rotate_pos == 4){
//                robot.rotwrist.setPosition(.4655);
//            }
//            if (rotate_pos == 5){
//                robot.rotwrist.setPosition(.435);
//            }




            else if (gamepad2.dpad_down) {
                liftTarget = LOW_POS;

            }
                else if (gamepad2.square) {
                    liftTarget = START_POS;

                }
            else if (gamepad2.dpad_up) {
                liftTarget = HIGH_POS;
            }
            if (gamepad2.cross) {
                liftTarget = START_POS;
            }

            else if (gamepad2.dpad_right) {
                liftTarget = larm.getCurrentPosition() + 40;
            }
            else if (gamepad2.dpad_left) {
                liftTarget = larm.getCurrentPosition() - 40;
            }


            if (gamepad1.left_bumper) {
                SpeedAdjust = 4;
            } else if (gamepad1.right_bumper) {
                SpeedAdjust = 1;
            }

            if (gamepad1.cross) {
               // robot.climb.r(100);

            }
//


            robot.leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
            robot.leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);

//              if (gamepad2.right_trigger > 0) {
//                    robot.climb.setPower(-2);
//
//                }else if (gamepad2.left_trigger > 0) {
//                    robot.climb.setPower(2);
//                }else {
//                    robot.climb.setPower(0);
//                }
//            if(gamepad1.square) {
//
//                    robot.drone.setPosition(.8);
//
//            }
            if (gamepad1.circle) { //drone
                waitTimer12.reset();
                while(waitTimer12.seconds() < waitTime12) {
                    robot.climb.setPower(1);
                }
                robot.climb.setPower(0);
            }
//
            if(gamepad1.triangle) {
                robot.drone.setPosition(.8);
            }
            if (gamepad2.right_trigger > 0 ) {

                robot.climb.setPower(-1);
            }else if (gamepad2.left_trigger >0 ) {

                    robot.climb.setPower(1);

               // drone_target = robot.climb.getCurrentPosition() + 150;
            } else {
                robot.climb.setPower(0);
            }
            if (gamepad1.square) {
              //  drone_target = 2500;
                waitTimer13.reset();
                while(waitTimer13.seconds() < waitTime13) {
                    robot.climb.setPower(1);
                }
                robot.climb.setPower(0);
            }
            if (robot.intakedis.getDistance(DistanceUnit.CM) > .25 && robot.intakedis.getDistance(DistanceUnit.CM) < 1 && waitTimer11.seconds() >= waitTime11) {

                telemetry.addData("Pixel Status", "2 PIXELS");
                robot.lgreenLED.setState(true);
                robot.lredLED.setState(false);
                robot.rgreenLED.setState(true);
                robot.rredLED.setState(false);

            } else{
                telemetry.addData("Pixel Status", "NO PIXELS");
                robot.lgreenLED.setState(false);
                robot.lredLED.setState(true);
                robot.rgreenLED.setState(false);
                robot.rredLED.setState(true);
            }

//            robot.climb.setPower(3);
//            robot.climb.setTargetPosition(drone_target);

               telemetry.addData("rotate#:",rotate_pos);
            telemetry.addData("intake#:",intakepos);

              telemetry.update();
              lift.update();
        }
    }
    class Lift {
        public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware

            controller = new PIDController(p, i, d);
            tl = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            larm = hardwareMap.get(DcMotorEx.class, "Llift");
            rarm = hardwareMap.get(DcMotorEx.class, "Rlift");


            larm.setDirection(DcMotorEx.Direction.FORWARD);
            rarm.setDirection(DcMotorEx.Direction.REVERSE);

            larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void update() {
//                setTarget(target);
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift


            controller.setPID(p, i, d);

            int larmPos = larm.getCurrentPosition();
            int rarmPos = rarm.getCurrentPosition();

            double Lpid = controller.calculate(larmPos, liftTarget);
            double Rpid = controller.calculate(rarmPos, liftTarget);

            // double Lff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; //* (12/voltageSensor.getVoltage()
            // double Rff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; // * (12/voltageSensor.getVoltage()

            double Lpower = Lpid + f;
            double Rpower = Rpid + f;

            larm.setPower(Lpower);
            rarm.setPower(Rpower);


            tl.update();
        }

        public boolean atTarget() {
            return controller.atSetPoint();
        }

//            public void moveToTarget(LiftPos target) {
//
//                setTarget(target);
//
//                while (!atTarget()) {
//                    update(target);
//
//                }
//            }

//            public void setTarget(LiftPos target) {
//                int encoderTarget = 0;
//                // Beep boop this is the lift update function
//                // Assume this runs some PID controller for the lift
//                switch (target) {
//                    //int MANUAL = larm.getCurrentPosition() +20;
//
//                    case START:
//                        encoderTarget = START_POS;
//                        break;
//                    case LOW:
//                        encoderTarget = LOW_POS;
//                        break;
//
//                    case MID:
//                        encoderTarget = MID_POS;
//                        break;
//                    case MIDHIGH:
//                        encoderTarget = MID_HIGH_POS;
//                        break;
//                    case HIGH:
//                        encoderTarget = HIGH_POS;
//                        break;
//
//                    case LOW_AUTO:
//                        encoderTarget = LOW_AUTO;
//                        break;
//                    case MANUAL:
//                        encoderTarget = larm.getCurrentPosition() + 20;
//                        break;
//                }

//                controller.setSetPoint(encoderTarget);
//            }

    }
}
