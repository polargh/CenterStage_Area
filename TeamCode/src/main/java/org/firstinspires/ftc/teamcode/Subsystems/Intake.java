package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake implements Subsystem {

    Servo claw;
    public Intake(HardwareMap map) {
        claw = map.get(Servo.class, "claw");
    }

    @Override
    public void update() {

    }

    @Override
    public void toInit() {

    }
}
