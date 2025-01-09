package org.firstinspires.ftc.teamcode.teleop;

import com.mcdanielpps.mechframework.util.RobotSystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp")
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotSystem manager = RobotSystem.getInstance();
        manager.Init(telemetry, hardwareMap, gamepad1, gamepad2);

        TeleOpTask mainTask = new TeleOpTask();
        mainTask.Init();

        waitForStart();

        manager.SpawnTask(mainTask);

        while(opModeIsActive()) {
            manager.Update();
        }
    }
}
