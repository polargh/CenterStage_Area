package team.spicyketchup.opmode.autos.functions;

import team.spicyketchup.subsystem.Intake;

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