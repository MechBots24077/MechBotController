package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.mcdanielpps.mechframework.util.RobotSystem;
import com.mcdanielpps.mechframework.util.Time;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.motion.LiftController;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AUTOPARK extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        RobotSystem.getInstance().Init(telemetry, hardwareMap, gamepad1, gamepad2);
        HardwareReferences references = new HardwareReferences();

        LiftController lift = new LiftController();
        lift.SetMotors(references.LeftLift, references.RightLift);

        waitForStart();

        lift.InitMotors();

        if (isStopRequested()) return;

//        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(30, -24), 0)
//                .build();
//
//        drive.followTrajectory(traj);

        lift.GoTo(1000, 2.0);


        sleep(2000);
    }
}
