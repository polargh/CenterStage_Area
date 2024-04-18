package team.spicyketchup.opmode.autos.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Hivemind Auto Blue 2+3", group = "tests")
public class HivemindAuto extends LinearOpMode {

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            TrajectorySequence sequence = drive.trajectorySequenceBuilder(new Pose2d(-35.5, 60, Math.toRadians(180)))
                    .build();

            drive.followTrajectorySequence(sequence);
        }
    }
}
