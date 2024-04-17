package Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    double lastPower;

    Servo drop;
    DcMotorEx intake;

    public Intake(SKRobot robot, Telemetry telemetry) {
        this.drop = robot.intakePitch;
        this.intake = robot.intakeMotor;

    }

    public Intake(HardwareMap hwMap, Telemetry telemetry) {

        drop = hwMap.get(Servo.class, "drop");
        intake = hwMap.get(DcMotorEx.class, "intake");

    }


    public void intake5(double sec) {
        drop.setPosition(.5755);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (runtime.seconds() <= sec) {
            setIntakePower(1);
        }

        drop.setPosition(.82);
        setIntakePower(0);
        runtime.reset();
    }

    public void intake3(double sec) {
        drop.setPosition(.485);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (runtime.seconds() <= sec) {
            setIntakePower(1);
        }

        drop.setPosition(.82);
        setIntakePower(0);
        runtime.reset();
    }

    public void intakewhile() {
        drop.setPosition(.535);
        setIntakePower(1);
    }

    public void intakewhile5() {
        drop.setPosition(.595);
        setIntakePower(.8);
    }

    public void intakewhile3() {
        drop.setPosition(.485);
        setIntakePower(1);
    }

    public void stopintake() {
        drop.setPosition(.82);
        setIntakePower(0);
    }

    public void outtake(double sec) {
        drop.setPosition(.6);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (runtime.seconds() <= sec) {
            setIntakePower(-.55);
        }

        setIntakePower(0);
        runtime.reset();
    }

    public void outtake2nd(double sec) {
        drop.setPosition(.82);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (runtime.seconds() <= sec) {
            setIntakePower(-.55);
        }
        
        setIntakePower(0);
        runtime.reset();
    }

    public void outtake2ndon() {
        drop.setPosition(.82);
        setIntakePower(-.1);
    }

    public void outtake2ndaud() {
        drop.setPosition(.82);
        setIntakePower(-.55);
    }

    public void score() {
       drop.setPosition(.6);
    }

    /**
    * Sets the power and caches it for later, ensures that power is never set more than once
    * @param power The power you want to set
    * @author polar
    */
    private void setIntakePower(double power) {
        if (power != lastPower) {
            intake.setPower(power);
        }

        lastPower = power;
    }
}


