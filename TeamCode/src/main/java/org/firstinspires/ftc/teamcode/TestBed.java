package org.firstinspires.ftc.teamcode;

import com.mcdanielpps.mechframework.util.Time;
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

    private DcMotor Lift = null;

    private DcMotor Wrist = null;

    private Servo Claw = null;

    private void Setup() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        RL = hardwareMap.get(DcMotor.class, "RL");
        RR = hardwareMap.get(DcMotor.class, "RR");

        Wrist = hardwareMap.get(DcMotor.class, "Wrist");
        Wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Claw = hardwareMap.get(Servo.class, "Claw");
    }

    private void UpdateWheels() {
        final double defaultSpeed = 0.4;

        // Get input
        double forwardInput = Math.pow(gamepad1.left_stick_x, 3);
        double sidewaysInput = Math.pow(-gamepad1.left_stick_y, 3);
        double turnInput = Math.pow(gamepad1.right_stick_x, 3);
        double speedInput = Math.pow(gamepad1.right_trigger, 3);

        // Crazy magic math https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/BasicOmniOpMode_Linear.java
        double flPower  = forwardInput + sidewaysInput - turnInput;
        double frPower = forwardInput - sidewaysInput - turnInput;
        double rlPower   = forwardInput - sidewaysInput + turnInput;
        double rrPower  = forwardInput + sidewaysInput + turnInput;

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

        // Calculate speed coefficient
        double speedCoefficient = defaultSpeed + speedInput * (1.0 - defaultSpeed);

        // Apply the calculated power to the wheels
        FL.setPower(flPower * speedCoefficient);
        FR.setPower(frPower * speedCoefficient);
        RL.setPower(rlPower * speedCoefficient);
        RR.setPower(rrPower * speedCoefficient);
    }

    private void UpdateLift(double deltaTime) {
        final double wristSpeed = 50.0;
        final double liftSpeed = 200.0;

        double wristPos = Wrist.getCurrentPosition();
        double wristInput = Math.pow(-gamepad2.left_stick_y, 3);
        Wrist.setTargetPosition((int)(wristPos + wristInput * wristSpeed));

        double liftPos = Lift.getCurrentPosition();
        double liftInput = Math.pow(gamepad2.right_stick_y, 3);
        Lift.setTargetPosition((int)(liftPos + liftInput * liftSpeed));
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    //private boolean upDebounce = false;
    //private boolean downDebounce = false;
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

        Wrist.setTargetPosition(0);
        Wrist.setPower(1.0);
        Wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Lift.setTargetPosition(0);
        Lift.setPower(1.0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Time.Init();
        while (opModeIsActive()) {
            Time.Update();

            UpdateWheels();
            UpdateLift(Time.DeltaTime());
            UpdateClaw();

            telemetry.update();
        }
    }

}
