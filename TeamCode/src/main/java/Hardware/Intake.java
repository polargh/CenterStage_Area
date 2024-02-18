package Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Intake implements Subsystem {

    Servo drop;
    public Intake(HardwareMap map) {
        drop = map.get(Servo.class, "drop");
    }

    @Override
    public void update() {

    }

    @Override
    public void toInit() {

    }
}
