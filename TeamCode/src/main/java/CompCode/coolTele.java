package CompCode;


//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.util.ElapsedTime;
//import com.arcrobotics.ftclib.controller.PIDController;

        import Hardware.v2bot_map;

@TeleOp(name="goofball tele")
public class coolTele extends LinearOpMode {


    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode

    //TELEmap robot   = new TELEmap();
    v2bot_map robot = new v2bot_map();

    double dropstack = 5;
    double stackpos = 0;
    // used with the dump servo, this will get covered in a bit
    ElapsedTime toggleTimer = new ElapsedTime();
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
//    double waitTime1 = .5;
//    double waitTime2 = 0;
//    double waitTime3 = 1.05;
//
//    ElapsedTime waitTimer1 = new ElapsedTime();
//    ElapsedTime waitTimer2 = new ElapsedTime();
//    ElapsedTime waitTimer3 = new ElapsedTime();
//    ElapsedTime runtime = new ElapsedTime();
//

    double SpeedAdjust = 1;

//    double servospeed = 0.5;
//
//    double lDropPos = 0;
//    double rDropPos = 0;

//    enum elbowDownState { //INTAKE
//        START,
//        INTAKE,
//        WRIST
//
//    }
//
//    enum elbowUpState { //OUTTAKE
//        START,
//        OUTTAKE,
//
//
//    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        // Lift lift = new Lift(hardwareMap, telemetry);

//        robot.wrist.setPosition(0.999);
//
//        robot.raxon.setPosition(.87);
//        robot.laxon.setPosition(.12);
//        robot.drone.setPosition(.5);
//            robot.ldrop.setPosition(0.214);
//            robot.rdrop.setPosition(0.175);
        robot.drop.setPosition(.5);
        waitForStart();

        if (isStopRequested()) return;

//        elbowUpState outtake = elbowUpState.START;
//        elbowDownState intake = elbowDownState.START;
//
//        //LiftPos liftTarget = LiftPos.START;
//        liftTarget = 0;
        while (opModeIsActive() && !isStopRequested()) {


            if (gamepad1.right_trigger > 0) { //out
                robot.intake.setPower(-3);

            } else if (gamepad1.left_trigger > 0) { //in
                //servospeed = 0.5;
                robot.drop.setPosition(.465);
                robot.intake.setPower(3);


            }else if (gamepad1.left_bumper) {
            }else {
                robot.intake.setPower(0);
                //robot.wheel.setPower(0);
                robot.drop.setPosition(.5);
            }

            if (gamepad1.right_bumper) { //out
                robot.drop.setPosition(.465);
                robot.intake.setPower(3);
            } else if (gamepad1.left_trigger > 0) { //in
                //servospeed = 0.5;
                robot.drop.setPosition(.465);
                robot.intake.setPower(3);


            } else {
                robot.intake.setPower(0);
                //robot.wheel.setPower(0);
                robot.drop.setPosition(.5);
            }


            if (gamepad1.triangle) { //out
                robot.drop.setPosition(.5);

            }
            if (gamepad1.circle) { //out
                robot.drop.setPosition(.485);

            }
            if (gamepad1.cross) { //out
                robot.drop.setPosition(.47);

            }
            if (gamepad1.square) { //out
                robot.drop.setPosition(.46);

            }
            telemetry.addData("stack position", stackpos);

            robot.leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
            robot.leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);

        }
    }
}
