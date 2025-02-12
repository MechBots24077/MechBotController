package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.mcdanielpps.mechframework.input.Input;
import com.mcdanielpps.mechframework.motion.MecanumWheelController;
import com.mcdanielpps.mechframework.motion.OdometryTranslator;
import com.mcdanielpps.mechframework.util.Time;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.motion.LiftController;

@TeleOp(name="TeleOp")
public class TeleOpMode extends LinearOpMode {
    private CRServo m_Extension = null;
    private Servo m_Claw = null;
    private Servo m_Wrist = null;

    private MecanumWheelController m_WheelController = new MecanumWheelController();
    private OdometryTranslator m_OdometryTranslator = new OdometryTranslator();
    private LiftController m_LiftController = new LiftController();

    private void GetHardwareReferences() {
        m_WheelController.FL = hardwareMap.get(DcMotor.class, "FL");
        m_WheelController.FR = hardwareMap.get(DcMotor.class, "FR");
        m_WheelController.RL = hardwareMap.get(DcMotor.class, "RL");
        m_WheelController.RR = hardwareMap.get(DcMotor.class, "RR");
        m_WheelController.InvertFL = true;
        m_WheelController.InvertRL = true;

        m_OdometryTranslator.Left = m_WheelController.RR;
        m_OdometryTranslator.Center = m_WheelController.RL;
        m_OdometryTranslator.Right = m_WheelController.FL;

        m_LiftController.SetMotors(
                hardwareMap.get(DcMotor.class, "LLift"),
                hardwareMap.get(DcMotor.class, "RLift"),
                hardwareMap.get(DigitalChannel.class, "L2")
        );

        m_Extension = hardwareMap.get(CRServo.class, "Extension");
        m_Claw = hardwareMap.get(Servo.class, "Claw");
        m_Wrist = hardwareMap.get(Servo.class, "Wrist");
    }

    private void ProcessMovementInput(TelemetryPacket packet) {
        // Map the 0-1 input from the trigger to 0.4-1
        double speedCoefficient = 0.4 + Input.ApplyFilter(gamepad1.right_trigger) * 0.6;

        m_WheelController.UpdateWheels(
            Input.ApplyFilter(-gamepad1.left_stick_x),
            Input.ApplyFilter(gamepad1.left_stick_y),
            Input.ApplyFilter(gamepad1.right_stick_x),
            speedCoefficient
        );

        m_OdometryTranslator.UpdateTelemetry(telemetry);
    }

    private void ProcessLiftInput(TelemetryPacket packet) {
        double liftPos = m_LiftController.GetCurrentGoal();
        double liftInput = Input.ApplyFilter(-gamepad2.right_stick_y);
        telemetry.addData("Lift pos", m_LiftController.GetCurrentPosition());


        m_LiftController.MoveToPosition((int)(liftPos + liftInput * 2000.0 * Time.DeltaTime()));
        m_LiftController.Update(packet);
    }

    private void ProcessExtensionInput(TelemetryPacket packet) {
        if (gamepad2.dpad_up) {
            m_Extension.setPower(0.6);
            m_Extension.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (gamepad2.dpad_down) {
            m_Extension.setPower(0.6);
            m_Extension.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            m_Extension.setPower(0.0);
        }
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    private void ProcessClawInput(TelemetryPacket packet) {
        double clawInput = gamepad2.right_trigger;
        double wristInput = gamepad2.left_trigger;

        double wristangle = map(1.0 - wristInput, 0.0, 1.0, 0.2, 0.7);
        telemetry.addData("Angle", wristangle);

        m_Claw.setPosition(map(1.0 - clawInput, 0.0, 1.0, 0.32, 0.55));
        m_Wrist.setPosition(wristangle);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        GetHardwareReferences();

        DigitalChannel L1 = hardwareMap.get(DigitalChannel.class, "L1"); // Claw
        DigitalChannel L2 = hardwareMap.get(DigitalChannel.class, "L2"); // Lift
        DigitalChannel L3 = hardwareMap.get(DigitalChannel.class, "L3"); // Extension
        DigitalChannel L4 = hardwareMap.get(DigitalChannel.class, "L4");

        m_WheelController.ResetMotors();

        waitForStart();

        m_WheelController.InitMotors(false);
        m_LiftController.InitMotors();

        L1.setMode(DigitalChannel.Mode.INPUT);
        L2.setMode(DigitalChannel.Mode.INPUT);
        L3.setMode(DigitalChannel.Mode.INPUT);
        L4.setMode(DigitalChannel.Mode.INPUT);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Time.Init();
        while(opModeIsActive()) {
            Time.Update();

            TelemetryPacket packet = new TelemetryPacket();

            ProcessMovementInput(packet);
            ProcessLiftInput(packet);
            ProcessExtensionInput(packet);
            ProcessClawInput(packet);

            telemetry.addData("L1", L1.getState());
            telemetry.addData("L2", L2.getState());
            telemetry.addData("L3", L3.getState());
            telemetry.addData("L4", L4.getState());

            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
