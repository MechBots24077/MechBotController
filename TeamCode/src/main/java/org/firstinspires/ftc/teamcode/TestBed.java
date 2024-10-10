package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestBed")
public class TestBed extends LinearOpMode {

    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor RL = null;
    private DcMotor RR = null;

    private DcMotor LLift = null;
    private DcMotor RLift = null;

    private DcMotor Wrist = null;

    private Servo Claw = null;

    private void Setup() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        RL = hardwareMap.get(DcMotor.class, "RL");
        RR = hardwareMap.get(DcMotor.class, "RR");

        Wrist = hardwareMap.get(DcMotor.class, "Wrist");
        Wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Wrist.setPower(1.0);

        LLift = hardwareMap.get(DcMotor.class, "LLift");
        RLift = hardwareMap.get(DcMotor.class, "RLift");

        Claw = hardwareMap.get(Servo.class, "Claw");
    }

    private void UpdateWheels() {
        // Get input
        double forwardInput = -gamepad1.left_stick_y;
        double sidewaysInput = gamepad1.left_stick_x;
        double turnInput = gamepad1.right_stick_x;

        // Crazy magic math https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/BasicOmniOpMode_Linear.java
        double flPower  = forwardInput + sidewaysInput + turnInput;
        double frPower = forwardInput - sidewaysInput - turnInput;
        double rlPower   = forwardInput - sidewaysInput + turnInput;
        double rrPower  = forwardInput + sidewaysInput - turnInput;

        // Normalize the values
        double max = Math.max(Math.abs(flPower), Math.abs(frPower));
        max = Math.max(max, Math.abs(rlPower));
        max = Math.max(max, Math.abs(rrPower));

        if (max > 1.0) {
            flPower  /= max;
            frPower /= max;
            rlPower   /= max;
            rrPower  /= max;
        }

        // Apply the calculated power to the wheels
        FL.setPower(flPower);
        FR.setPower(frPower);
        RL.setPower(rlPower);
        RR.setPower(rrPower);
    }

    private void UpdateLift() {
        int target = (int)map(gamepad2.left_stick_y, -1.0, 1.0, 0.0, 5000.0);
        Wrist.setTargetPosition(target);
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    private boolean upDebounce = false;
    private boolean downDebounce = false;
    private void UpdateClaw() {
        // 0.14 open 0.39 closed

        double clawInput = gamepad2.right_trigger;

        Claw.setPosition(map(clawInput, 0.0, 1.0, 0.30, 0.39));

        /*
        boolean up = false;
        boolean down = false;

        if (upDebounce == false && gamepad1.dpad_up) {
            upDebounce = true;
            up = true;
        } else if (gamepad1.dpad_up == false) {
            upDebounce = false;
        }

        if (downDebounce == false && gamepad1.dpad_down) {
            downDebounce = true;
            down = true;
        } else if (gamepad1.dpad_down == false) {
            downDebounce = false;
        }

        double movement = (up ? 0.01 : 0.0) + (down ? -0.01 : 0.0);
        Claw.setPosition(Claw.getPosition() + movement);

        telemetry.addData("Claw", Claw.getPosition());

         */
    }

    @Override
    public void runOpMode() {
        Setup();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        telemetry.addData("Status", "Running");

        while (opModeIsActive()) {
            UpdateWheels();
            UpdateLift();
            UpdateClaw();

            telemetry.update();
        }
    }

}
