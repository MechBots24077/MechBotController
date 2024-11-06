package org.firstinspires.ftc.teamcode;

import com.mcdanielpps.mechframework.input.Input;
import com.mcdanielpps.mechframework.motion.MecanumWheelController;
import com.mcdanielpps.mechframework.motion.MotorController;
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
    private DigitalChannel m_LiftLimit = null;

    private AnalogInput m_TuneKnob = null;

    private MecanumWheelController m_WheelController = new MecanumWheelController();
    private LiftController m_LiftController = new LiftController();

    private void GetHardwareReferences() {
        m_WheelController.FL = hardwareMap.get(DcMotor.class, "FL");
        m_WheelController.FR = hardwareMap.get(DcMotor.class, "FR");
        m_WheelController.RL = hardwareMap.get(DcMotor.class, "RL");
        m_WheelController.RR = hardwareMap.get(DcMotor.class, "RR");
        m_WheelController.InvertFL = true;

        m_LiftController.SetMotors(
                hardwareMap.get(DcMotor.class, "LLift"),
                hardwareMap.get(DcMotor.class, "RLift")
        );
//        m_LiftController.LLift = hardwareMap.get(DcMotor.class, "LLift");
//        m_LiftController.RLift = hardwareMap.get(DcMotor.class, "RLift");

        m_Extension = hardwareMap.get(CRServo.class, "Extension");
        m_Claw = hardwareMap.get(Servo.class, "Claw");
        m_LiftLimit = hardwareMap.get(DigitalChannel.class, "LiftLimit");

        m_TuneKnob = hardwareMap.get(AnalogInput.class, "Pot");
    }

    private void ProcessMovementInput() {
        // Map the 0-1 input from the trigger to 0.4-1
        double speedCoefficient = 0.4 + Input.ApplyFilter(gamepad1.right_trigger) * 0.6;

        m_WheelController.UpdateWheelsGamepad(
            Input.ApplyFilter(-gamepad1.left_stick_x),
            Input.ApplyFilter(gamepad1.left_stick_y),
            Input.ApplyFilter(gamepad1.right_stick_x),
            speedCoefficient
        );
    }

    private void ProcessLiftInput() {
        double liftPos = m_LiftController.GetCurrentGoaL();
        double liftInput = Input.ApplyFilter(-gamepad2.left_stick_y);
        telemetry.addData("Lift pos", m_LiftController.GetCurrentPosition());


        //m_LiftController.MoveToPosition((int)(liftInput * 1000.0));
        //telemetry.addData("Lift Target", (int)(liftInput * 1000.0));
        m_LiftController.MoveToPosition((int)(liftPos + liftInput * 50.0));
        telemetry.addData("Lift target", (int)(liftPos + liftInput * 50.0));
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

        m_Claw.setPosition(map(clawInput, 0.0, 1.0, 0.32, 0.55));
    }

    private void ProcessTuneInput() {
        double tuneVal = map(m_TuneKnob.getVoltage(), 0.0, 3.34, 0.0, 10.0);

        telemetry.addData("Tune",  tuneVal);
        //P: 3.0 I: 1.0 D: 0.0
        telemetry.addData("P", m_LiftController.LLift.m_PID.Kp);
        telemetry.addData("I", m_LiftController.LLift.m_PID.Ki);
        telemetry.addData("D", m_LiftController.LLift.m_PID.Kd);

        if (gamepad2.y) {
            m_LiftController.LLift.m_PID.Kp = tuneVal;
            m_LiftController.RLift.m_PID.Kp = tuneVal;
        }

        if (gamepad2.b) {
            m_LiftController.LLift.m_PID.Ki = tuneVal;
            m_LiftController.RLift.m_PID.Ki = tuneVal;
        }

        if (gamepad2.a) {
            m_LiftController.LLift.m_PID.Kd = tuneVal;
            m_LiftController.RLift.m_PID.Kd = tuneVal;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        GetHardwareReferences();

        m_WheelController.ResetMotors();
        //m_LiftController.ResetMotors();

        waitForStart();

        m_WheelController.InitMotors(true);
        m_LiftController.InitMotors();

        m_LiftLimit.setMode(DigitalChannel.Mode.INPUT);

        Time.Init();
        while(opModeIsActive()) {
            Time.Update();

            ProcessTuneInput();

            ProcessMovementInput();
            ProcessLiftInput();
            ProcessExtensionInput();
            ProcessClawInput();

            telemetry.update();
        }
    }
}
