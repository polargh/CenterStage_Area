package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.sfdev.assembly.state.StateMachine;
//import com.sfdev.assembly.state.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.TeleControl.IntakeControl;
import org.firstinspires.ftc.teamcode.TeleControl.OuttakeControl;
@Disabled
@TeleOp
public class Teleop extends LinearOpMode {

    enum States {
        NEUTRAL

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        OuttakeControl outtakeControl = new OuttakeControl(robot.outtake, gamepad1, gamepad2);
        IntakeControl intakeControl = new IntakeControl(robot.intake, gamepad1, gamepad2);

        // setting everything to init
        robot.toInit();

       // StateMachine machine = new StateMachineBuilder()
//               // .state(States.NEUTRAL)
//                .onEnter(() -> {
//                    // method calls
//                })
//                .onExit(() -> {
//                    // method calls
//
//                })
//                .transition(() -> gamepad2.x)
//
//
//                .build();
//
//        waitForStart();
//
//        machine.start();

        while (opModeIsActive()) {



            // updating state machine
           // machine.update();

            // updating robot classes
            robot.update();
            outtakeControl.update();
            intakeControl.update();
        }
    }
}
