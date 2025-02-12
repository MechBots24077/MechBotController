package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AUTOPARK extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Servo claw = hardwareMap.get(Servo.class, "Claw");
        claw.setPosition(0.55);
        Servo wrist = hardwareMap.get(Servo.class, "Wrist");
        wrist.setPosition(0.5);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(5, -17))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(5, -17))
                .strafeTo(new Vector2d(2, -34))
                .build();


        drive.followTrajectory(traj1);
        drive.waitForIdle();
        drive.followTrajectory(traj2);

        sleep(2000);
    }
}
