package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.mcdanielpps.mechframework.input.Input;
import com.mcdanielpps.mechframework.motion.MecanumWheelController;
import com.mcdanielpps.mechframework.util.MechUtil;
import com.mcdanielpps.mechframework.util.RobotSystem;
import com.mcdanielpps.mechframework.util.Time;
import com.mcdanielpps.mechframework.util.task.Task;
import com.mcdanielpps.mechframework.util.task.TaskStatus;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HardwareReferences;
import org.firstinspires.ftc.teamcode.motion.LiftController;

public class TeleOpTask implements Task {
    private TaskStatus m_Status = TaskStatus.Running;
    @Override
    public TaskStatus GetStatus() { return m_Status; }

    private RobotSystem m_System = null;
    private HardwareReferences m_Hardware = null;

    private MecanumWheelController m_WheelController = new MecanumWheelController();
    private LiftController m_LiftController = new LiftController();

    public void Init() {
        m_System = RobotSystem.getInstance();

        m_Hardware = new HardwareReferences();

        m_WheelController.FL = m_Hardware.FL;
        m_WheelController.FR = m_Hardware.FR;
        m_WheelController.RL = m_Hardware.RL;
        m_WheelController.RR = m_Hardware.RR;
        m_WheelController.InvertFL = true;
        m_WheelController.InvertRL = true;

        m_WheelController.ResetMotors();

        m_LiftController.SetMotors(
                m_Hardware.LeftLift,
                m_Hardware.RightLift
        );
    }

    @Override
    public void Start() {
        m_WheelController.InitMotors(false);
        m_LiftController.InitMotors();
    }

    private void ProcessMovementInput() {
        Gamepad gamepad1 = m_System.GetGamepad1();

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
        TelemetryPacket packet = RobotSystem.getInstance().GetTelemetryPacket();
        Gamepad gamepad2 = m_System.GetGamepad2();

        double liftPos = m_LiftController.GetCurrentGoal();
        double liftInput = -gamepad2.right_stick_y;
        m_System.GetTelemetry().addData("Lift Pos", m_LiftController.GetCurrentPosition());

        m_LiftController.MoveToPosition((int)(liftPos + liftInput * 2000.0 * Time.DeltaTime()));
        m_LiftController.Update(packet);
    }

    private void ProcessExtensionInput() {
        Gamepad gamepad2 = m_System.GetGamepad2();

        if (gamepad2.dpad_up) {
            m_Hardware.Extension.setPower(0.6);
            m_Hardware.Extension.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (gamepad2.dpad_down) {
            m_Hardware.Extension.setPower(0.6);
            m_Hardware.Extension.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            m_Hardware.Extension.setPower(0.0);
        }
    }

    private void ProcessClawInput() {
        Gamepad gamepad2 = m_System.GetGamepad2();

        double wristInput = gamepad2.left_trigger;
        double wristAngle = MechUtil.map(1.0 - wristInput, 0.0, 1.0, 0.2, 0.7);
        m_Hardware.Wrist.setPosition(wristAngle);

        double clawInput = gamepad2.right_trigger;
        double clawAngle = MechUtil.map(1.0 - clawInput, 0.0, 1.0, 0.32, 0.55);
        m_Hardware.Claw.setPosition(clawAngle);
    }

    @Override
    public void Update() {
        ProcessMovementInput();
        ProcessLiftInput();
        ProcessExtensionInput();
        ProcessClawInput();
    }
}
