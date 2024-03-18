package Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Intake {

    Servo drop;
    DcMotorEx intake;
    public Intake(v2bot_map robot, Telemetry telemetry){
        this.drop = robot.drop;
        this.intake = robot.intake;

    }
    public Intake(HardwareMap hwMap, Telemetry telemetry){

        drop = hwMap.get(Servo.class, "drop");
        intake = hwMap.get(DcMotorEx.class, "intake");

    }


    public void intake5(double sec) {
        drop.setPosition(.59);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.seconds() <= sec){
        intake.setPower(1.9);
    }
        drop.setPosition(.89);
       intake.setPower(0);
        runtime.reset();
    }
    public void intake3(double sec) {
        drop.setPosition(.585);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.seconds() <= sec){
            intake.setPower(1.9);
        }
        drop.setPosition(.89);
        intake.setPower(0);
        runtime.reset();
    }
    public void intakewhile() {
        drop.setPosition(.482);
            intake.setPower(1.9);
    }
    public void intakewhile5() {
        drop.setPosition(.6605);
        intake.setPower(.8);
    }
    public void intakewhile3() {
        drop.setPosition(.57945);
        intake.setPower(1);
    }
    public void stopintake() {
        drop.setPosition(.95);
        intake.setPower(0);
    }


    public void outtake(double sec) {
        drop.setPosition(.6);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.seconds() <= sec){
            intake.setPower(-.5);
        }
        intake.setPower(0);
        runtime.reset();
    }
    public void outtake2nd(double sec) {
        drop.setPosition(.95);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.seconds() <= sec){
            intake.setPower(-.5);
        }
        intake.setPower(0);
        runtime.reset();
    }
    public void outtake2ndon() {
        drop.setPosition(.95);


            intake.setPower(-.1);

    }
    public void outtake2ndaud() {
        drop.setPosition(.95);


        intake.setPower(-.45);

    }

    public void score() {
       drop.setPosition(.6);
    }


    }


