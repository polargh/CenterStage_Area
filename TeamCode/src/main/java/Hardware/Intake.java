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


    public void intake(double sec) {
        drop.setPosition(.48285);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.seconds() <= sec){
        intake.setPower(1.9);
    }
        drop.setPosition(.5);
       intake.setPower(0);
        runtime.reset();
    }
    public void intakewhile() {
        drop.setPosition(.482);
            intake.setPower(1.9);
    }
    public void stopintake() {
        drop.setPosition(.48285);
        intake.setPower(0);
    }


    public void outtake(double sec) {
        drop.setPosition(.5);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.seconds() <= sec){
            intake.setPower(-.75);
        }
        intake.setPower(0);
        runtime.reset();
    }

    public void score() {
       drop.setPosition(.466);
    }


    }


