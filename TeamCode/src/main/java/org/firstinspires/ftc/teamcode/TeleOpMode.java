package org.firstinspires.ftc.teamcode;

import com.mcdanielpps.mechframework.input.Input;
import com.mcdanielpps.mechframework.motion.MecanumWheelController;
import com.mcdanielpps.mechframework.util.Time;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.motion.LiftController;

@TeleOp(name="TeleOp")
public class TeleOpMode extends LinearOpMode {
    private CRServo m_Extension = null;
    private Servo m_Claw = null;
    private Servo m_Wrist = null;
    private DigitalChannel m_LiftLimit = null;

    private MecanumWheelController m_WheelController = new MecanumWheelController();
    private LiftController m_LiftController = new LiftController();

    private void GetHardwareReferences() {
        m_WheelController.FL = hardwareMap.get(DcMotor.class, "FL");
        m_WheelController.FR = hardwareMap.get(DcMotor.class, "FR");
        m_WheelController.RL = hardwareMap.get(DcMotor.class, "RL");
        m_WheelController.RR = hardwareMap.get(DcMotor.class, "RR");
        m_WheelController.InvertFL = true;
        m_WheelController.InvertRL = true;
        m_WheelController.InvertFR = true;
        m_WheelController.InvertRR = true;

        m_LiftController.SetMotors(
                hardwareMap.get(DcMotor.class, "LLift"),
                hardwareMap.get(DcMotor.class, "RLift")
        );

        m_Extension = hardwareMap.get(CRServo.class, "Extension");
        m_Claw = hardwareMap.get(Servo.class, "Claw");
        m_Wrist = hardwareMap.get(Servo.class, "Wrist");
        m_LiftLimit = hardwareMap.get(DigitalChannel.class, "LiftLimit");
    }

    private void ProcessMovementInput() {
        // Map the 0-1 input from the trigger to 0.4-1
        double speedCoefficient = 0.4 + Input.ApplyFilter(gamepad1.right_trigger) * 0.6;

        m_WheelController.UpdateWheels(
            Input.ApplyFilter(-gamepad1.left_stick_x),
            Input.ApplyFilter(gamepad1.left_stick_y),
            Input.ApplyFilter(gamepad1.right_stick_x),
            speedCoefficient
        );
    }

    private void ProcessLiftInput() {
        double liftPos = m_LiftController.GetCurrentGoal();
        double liftInput = Input.ApplyFilter(-gamepad2.right_stick_y);
        telemetry.addData("Lift pos", m_LiftController.GetCurrentPosition());


        m_LiftController.MoveToPosition((int)(liftPos + liftInput * 1000.0 * Time.DeltaTime()));
        m_LiftController.Update();
    }

    private void ProcessExtensionInput() {
        if (gamepad2.dpad_up) {
            m_Extension.setPower(0.3);
            m_Extension.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (gamepad2.dpad_down) {
            m_Extension.setPower(0.3);
            m_Extension.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            m_Extension.setPower(0.0);
        }
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    private void ProcessClawInput() {
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

        m_WheelController.ResetMotors();

        waitForStart();

        m_WheelController.InitMotors(false);
        m_LiftController.InitMotors();

        m_LiftLimit.setMode(DigitalChannel.Mode.INPUT);

        Time.Init();
        while(opModeIsActive()) {
            Time.Update();

            ProcessMovementInput();
            ProcessLiftInput();
            ProcessExtensionInput();
            ProcessClawInput();

            telemetry.update();
        }
    }
}
