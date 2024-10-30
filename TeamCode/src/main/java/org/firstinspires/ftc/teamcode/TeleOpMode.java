package org.firstinspires.ftc.teamcode;

import com.mcdanielpps.mechframework.input.Input;
import com.mcdanielpps.mechframework.motion.MecanumWheelController;
import com.mcdanielpps.mechframework.util.Time;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.motion.LiftController;

@TeleOp(name="TeleOp")
public class TeleOpMode extends LinearOpMode {
    private CRServo m_Extension = null;
    private Servo m_Claw = null;

    private MecanumWheelController m_WheelController = new MecanumWheelController();
    private LiftController m_LiftController = new LiftController();

    private void GetHardwareReferences() {
        m_WheelController.FL = hardwareMap.get(DcMotor.class, "FL");
        m_WheelController.FR = hardwareMap.get(DcMotor.class, "FR");
        m_WheelController.RL = hardwareMap.get(DcMotor.class, "RL");
        m_WheelController.RR = hardwareMap.get(DcMotor.class, "RR");

        m_LiftController.LLift = hardwareMap.get(DcMotor.class, "LLift");
        m_LiftController.RLift = hardwareMap.get(DcMotor.class, "RLift");

        m_Extension = hardwareMap.get(CRServo.class, "Extension");
        m_Claw = hardwareMap.get(Servo.class, "Claw");
    }

    private void ProcessMovementInput() {
        // Map the 0-1 input from the trigger to 0.4-1
        double speedCoefficient = 0.4 + Input.ApplyFilter(gamepad1.right_trigger) * 0.6;

        m_WheelController.UpdateWheelsGamepad(
            Input.ApplyFilter(gamepad1.left_stick_x),
            Input.ApplyFilter(gamepad1.left_stick_y),
            Input.ApplyFilter(gamepad1.right_stick_x),
            speedCoefficient
        );
    }

    private void ProcessLiftInput() {
        double liftPos = m_LiftController.GetCurrentPosition();
        double liftInput = Input.ApplyFilter(-gamepad2.left_stick_y);
        telemetry.addData("Lift input", liftInput);
        telemetry.addData("Lift pos", liftPos);
        m_LiftController.MoveToPosition((int)(liftPos + liftInput * 50));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        GetHardwareReferences();

        m_WheelController.ResetMotors();
        m_LiftController.ResetMotors();

        waitForStart();

        m_WheelController.InitMotors(true);
        m_LiftController.InitMotors();

        Time.Init();
        while(opModeIsActive()) {
            Time.Update();

            ProcessMovementInput();
            ProcessLiftInput();

            telemetry.update();
        }
    }
}
