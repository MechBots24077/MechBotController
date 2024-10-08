package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="First OpMode")
public class FirstOpMode extends LinearOpMode {

    private DcMotor testMotor = null;

    @Override
    public void runOpMode() {
        testMotor = hardwareMap.get(DcMotor.class, "test");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            float speed = gamepad1.left_stick_y;
            testMotor.setPower(speed);
        }
    }

}
