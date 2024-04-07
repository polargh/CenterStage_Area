package Autos.AutoFunctions;

import Hardware.Intake;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class IntakeCommands{
Intake intake;
    public void IntakeCycle1() {
    intake.stopintake();
    intake.outtake(.25);
    intake.intake5(.6);
    intake.outtake2nd(.3);
    intake.outtake2ndon();
    }

}