package Hardware;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import Hardware.Lift;
import Hardware.v2bot_map;

public class Arm {
    public Servo rotwrist ;
    public Servo raxon;
    public Servo bendwrist;
    public Servo laxon;
    public Servo drop;

    public Servo rearclaw;
    public Servo frontclaw;
    public Servo lflap;
    public Servo rflap;
    double FRONTGRAB = .335;
    double REARGRAB = .398;
    double FRONTRELEASE = .477;
    double REARRELEASE = .231;


    double LFLAPUP = .465;
    double LFLAPDOWN = .57425;
    double RFLAPUP = .512;
    double RFLAPDOWN = .4309;

    Lift lift;
    Telemetry telemetry;


    public Arm(v2bot_map robot, Telemetry telemetry){
        this.rotwrist = robot.rotwrist;
        this.raxon = robot.raxon;
        this.bendwrist = robot.bendwrist;
        this.laxon = robot.laxon;
        this.drop = robot.drop;
        this.frontclaw = robot.frontclaw;
        this.rearclaw = robot.rearclaw;
        this.rflap = robot.rflap;
        this.lflap = robot.lflap;

        this.telemetry = telemetry;

    }
    public Arm(HardwareMap hwMap, Telemetry telemetry){
        rotwrist = hwMap.get(Servo.class, "spinwrist");
        raxon = hwMap.get(Servo.class, "raxon");
        laxon = hwMap.get(Servo.class, "laxon");
        bendwrist = hwMap.get(Servo.class, "bendwrist");
        drop = hwMap.get(Servo.class, "drop");
        frontclaw = hwMap.get(Servo.class, "frontclaw");
        rearclaw = hwMap.get(Servo.class, "rearclaw");
        lflap = hwMap.get(Servo.class, "lflap");
        rflap = hwMap.get(Servo.class, "rflap");
        this.telemetry = telemetry;
    }
    public void pulloutpixel(){
        drop.setPosition(.465);
        bendwrist.setPosition(.145);
    }
    public void goToScoringPoslift(){
        drop.setPosition(.465);
        raxon.setPosition(.3);
        laxon.setPosition(.7);
    }

    public void deposit(){

    }


    public void intakePos(){
       rotwrist.setPosition(.41);
        drop.setPosition(.63);
        raxon.setPosition(.64);
        laxon.setPosition(.36);
        bendwrist.setPosition(.15);
        rearclaw.setPosition(REARRELEASE);
        frontclaw.setPosition(FRONTRELEASE);


    }
    public void intakePostele(){
        rotwrist.setPosition(.41);
        drop.setPosition(.63);


        rearclaw.setPosition(REARRELEASE);
        frontclaw.setPosition(FRONTRELEASE);
        raxon.setPosition(.66);
        laxon.setPosition(.34);
        bendwrist.setPosition(.1595);


    }
    public void init(){
        rotwrist.setPosition(.41);
        drop.setPosition(.89);
        raxon.setPosition(.64);
        laxon.setPosition(.46);
        bendwrist.setPosition(.15);
        rearclaw.setPosition(REARGRAB);
        frontclaw.setPosition(FRONTGRAB);
        lflap.setPosition(LFLAPDOWN);
        rflap.setPosition(RFLAPDOWN);

    }
    public void afterdropintake(){

        drop.setPosition(.89);


    }
    public void intakePosafterscore(){
        raxon.setPosition(.61);
        laxon.setPosition(.39);
        bendwrist.setPosition(.153);
        rotwrist.setPosition(.41);
        rearclaw.setPosition(REARRELEASE);
        frontclaw.setPosition(FRONTRELEASE);
        drop.setPosition(.4825);
    }
    public void flapsup(){ //almost grab
        lflap.setPosition(LFLAPUP);
        rflap.setPosition(RFLAPUP);
        rearclaw.setPosition(REARRELEASE);
        frontclaw.setPosition(FRONTRELEASE);

    }
    public void downpixel(){ //almost grab
        rotwrist.setPosition(.41);
       raxon.setPosition(.783);
       laxon.setPosition(.217);
       bendwrist.setPosition(.159);
        drop.setPosition(.466);

    }
    public void grab(){ //almost grab
        rearclaw.setPosition(REARGRAB);
        frontclaw.setPosition(FRONTGRAB);
    }
    public void aftergrab(){ //almost grab
        raxon.setPosition(.64);
        laxon.setPosition(.36);
        bendwrist.setPosition(.151);
        drop.setPosition(.466);
    }
    public void outyellowp2(){ //almost grab
       // lift.moveToTarget(Lift.LiftPos.LOW_AUTO);
        //bendwrist.setPosition(.705);
//        drop.setPosition(.95);
//        raxon.setPosition(.3);
//        laxon.setPosition(.7);
        rotwrist.setPosition(.15);
    }
    public void outyellowp1(){ //almost grab
        // lift.moveToTarget(Lift.LiftPos.LOW_AUTO);
        bendwrist.setPosition(.705);
        drop.setPosition(.95);
        raxon.setPosition(.3);
        laxon.setPosition(.7);
       // rotwrist.setPosition(.4);
    }
    public void release(){ //almost grab
       frontclaw.setPosition(FRONTRELEASE);
       rearclaw.setPosition(REARRELEASE);
    }

//    ElapsedTime runtime = new ElapsedTime();
//        runtime.reset();
//        while(runtime.seconds() <= sec){
//        rotwrist.setPosition(.99);
//    }
//        wheel.setPosition(.5);
//        runtime.reset();
}
