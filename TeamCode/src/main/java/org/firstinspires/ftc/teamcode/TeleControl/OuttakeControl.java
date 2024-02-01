package org.firstinspires.ftc.teamcode.TeleControl;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

public class OuttakeControl implements Control{
    Outtake outtake;
    Gamepad gamepad1;
    Gamepad gamepad2;

    public OuttakeControl(Outtake ot, Gamepad gp1, Gamepad gp2) {
        gamepad1 = gp1;
        gamepad2 = gp2;
        outtake = ot;
    }

    @Override
    public void update() {
        if (gamepad1.a) {


        }
    }
}
