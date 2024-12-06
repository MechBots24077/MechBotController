package org.firstinspires.ftc.teamcode;

import com.mcdanielpps.mechframework.input.Input;
import com.mcdanielpps.mechframework.motion.MecanumWheelController;
import com.mcdanielpps.mechframework.util.Time;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestBed")
public class TestBed extends LinearOpMode {
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor RL = null;
    private DcMotor RR = null;

    private MecanumWheelController m_WheelController = new MecanumWheelController();

    private void GetHardwareReferences() {
        m_WheelController.FL = hardwareMap.get(DcMotor.class, "FL");
        m_WheelController.FR = hardwareMap.get(DcMotor.class, "FR");
        m_WheelController.RL = hardwareMap.get(DcMotor.class, "RL");
        m_WheelController.RR = hardwareMap.get(DcMotor.class, "RR");

        m_WheelController.InvertFL = true;
        m_WheelController.InvertRL = true;
    }

    private void UpdateWheels() {
        // Map the 0-1 input from the trigger to 0.4-1
        double speedCoefficient = 0.4 + Input.ApplyFilter(gamepad1.right_trigger) * 0.6;

        telemetry.addData("Boost", speedCoefficient);
        m_WheelController.UpdateWheels(
                Input.ApplyFilter(-gamepad1.left_stick_x),
                Input.ApplyFilter(gamepad1.left_stick_y),
                Input.ApplyFilter(gamepad1.right_stick_x),
                speedCoefficient
        );
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    @Override
    public void runOpMode() {
        GetHardwareReferences();

        m_WheelController.ResetMotors();

        waitForStart();

        m_WheelController.InitMotors(false);

        Time.Init();
        while (opModeIsActive()) {
            Time.Update();

            UpdateWheels();

            telemetry.update();
        }
    }

}
