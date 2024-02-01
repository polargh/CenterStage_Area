package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Drivebase implements Subsystem {
    public SampleMecanumDrive drive;
    Pose2d position;
    public Pose2d pose;

    public Drivebase(HardwareMap map, Pose2d pos) {
       position = pos;
       drive = new SampleMecanumDrive(map);
       drive.setPoseEstimate(pos);
       pose = pos;
    }

    public Drivebase(HardwareMap map) {
        this(map, new Pose2d(0, 0, 0));
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void setPower(Pose2d powers) {
        drive.setWeightedDrivePower(new Pose2d(powers.getX(), powers.getY(), powers.getHeading()));
    }
    public SampleMecanumDrive getDrive() {
        return drive;
    }

    public void followTrajectory(Trajectory traj) {
        drive.followTrajectory(traj);
    }

    @Override
    public void update() {
        drive.update();
        pose = drive.getPoseEstimate();
    }

    public void toInit(){}
}
