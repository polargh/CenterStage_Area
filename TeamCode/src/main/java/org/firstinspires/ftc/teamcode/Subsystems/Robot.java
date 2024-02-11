package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot implements Subsystem {

    public Outtake outtake;
    public Intake intake;
    Drivebase drivebase;
    Subsystem[] systems;

    public Robot(HardwareMap hardwareMap) {
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        drivebase = new Drivebase(hardwareMap);
        systems = new Subsystem[]{outtake, intake, drivebase};
    }

    @Override
    public void update() {
        // updating every subsystem
        for (Subsystem system : systems) {
            system.update();
        }
    }

    @Override
    public void toInit() {
        // setting every subsystem to init
        for (Subsystem system : systems) {
            system.toInit();
        }
    }
}
