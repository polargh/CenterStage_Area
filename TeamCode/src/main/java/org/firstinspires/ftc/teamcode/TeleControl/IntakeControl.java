package org.firstinspires.ftc.teamcode.TeleControl;

import com.qualcomm.robotcore.hardware.Gamepad;
import Hardware.Intake;

public class IntakeControl implements Control{
    Intake intake;
    Gamepad gamepad1;
    Gamepad gamepad2;

    public IntakeControl(Intake intake, Gamepad gp1, Gamepad gp2) {
        gamepad1 = gp1;
        gamepad2 = gp2;
        this.intake = intake;
    }

    @Override
    public void update() {
        if (gamepad1.a) {

        }
    }
}
