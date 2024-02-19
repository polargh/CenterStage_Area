package Hardware;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Hardware.v2bot_map;

public class Arm {
    public Servo rotwrist ;
    public Servo raxon;
    public Servo bendwrist;
    public Servo laxon;
    public Servo drop;

    public Servo rearclaw;
    public Servo frontclaw;
    double FRONTGRAB = .2729;
    double REARGRAB = .413;
    double FRONTRELEASE = .472;
    double REARRELEASE = .235;




    Telemetry telemetry;


    public Arm(v2bot_map robot, Telemetry telemetry){
        this.rotwrist = robot.rotwrist;
        this.raxon = robot.raxon;
        this.bendwrist = robot.bendwrist;
        this.laxon = robot.laxon;
        this.drop = robot.drop;
        this.frontclaw = robot.frontclaw;
        this.rearclaw = robot.rearclaw;
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
        this.telemetry = telemetry;
    }
    public void pulloutpixel(){
        drop.setPosition(.465);
        bendwrist.setPosition(.145);
    }
    public void goToScoringPos(){
        drop.setPosition(.465);
        raxon.setPosition(.3);
        laxon.setPosition(.7);
    }

    public void deposit(){

    }


    public void intakePos(){
        raxon.setPosition(.658);
        laxon.setPosition(.342);
        bendwrist.setPosition(.158);
        rotwrist.setPosition(.497);
        rearclaw.setPosition(REARRELEASE);
        frontclaw.setPosition(FRONTRELEASE);
        drop.setPosition(.5);
    }
    public void intakePosafterscore(){
        raxon.setPosition(.658);
        laxon.setPosition(.342);
        bendwrist.setPosition(.158);
        rotwrist.setPosition(.497);
        rearclaw.setPosition(REARRELEASE);
        frontclaw.setPosition(FRONTRELEASE);
        drop.setPosition(.4825);
    }
    public void downpixel(){ //almost grab
        rotwrist.setPosition(.497);
        raxon.setPosition(.821);
        laxon.setPosition(.179);
        bendwrist.setPosition(.1528);
        drop.setPosition(.466);

    }
    public void grab(){ //almost grab
        rearclaw.setPosition(REARGRAB);
        frontclaw.setPosition(FRONTGRAB);
    }
    public void aftergrab(){ //almost grab
        raxon.setPosition(.64);
        laxon.setPosition(.36);
        bendwrist.setPosition(.149);
        drop.setPosition(.466);
    }
    public void out(){ //almost grab
        drop.setPosition(.466);
        raxon.setPosition(.295);
        laxon.setPosition(.705);
        bendwrist.setPosition(.6835);
        rotwrist.setPosition(.4);
    }
    public void drop(){ //almost grab
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
