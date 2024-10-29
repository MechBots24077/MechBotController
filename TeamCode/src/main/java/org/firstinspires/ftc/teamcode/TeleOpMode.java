package org.firstinspires.ftc.teamcode;

import com.mcdanielpps.mechframework.motion.MecanumWheelController;
import com.mcdanielpps.mechframework.util.Time;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp")
public class TeleOpMode extends LinearOpMode {

    private DcMotor m_FL = null;
    private DcMotor m_FR = null;
    private DcMotor m_RL = null;
    private DcMotor m_RR = null;

    private DcMotor m_LLift = null;
    private DcMotor m_RLift = null;

    private Servo m_Extension = null;
    private Servo m_Claw = null;

    private MecanumWheelController m_WheelController = new MecanumWheelController();

    private void GetHardwareReferences() {
        m_FL = hardwareMap.get(DcMotor.class, "FL");
        m_FR = hardwareMap.get(DcMotor.class, "FR");
        m_RL = hardwareMap.get(DcMotor.class, "RL");
        m_RR = hardwareMap.get(DcMotor.class, "RR");

        m_LLift = hardwareMap.get(DcMotor.class, "LLift");
        m_RLift = hardwareMap.get(DcMotor.class, "RLift");

        m_Extension = hardwareMap.get(Servo.class, "Extension");
        m_Claw = hardwareMap.get(Servo.class, "Claw");
    }

    private void ResetMotors() {
        m_LLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_RLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void InitMotors() {
        m_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m_LLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_RLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void InitWheels() {
        m_WheelController.FL = m_FL;
        m_WheelController.FR = m_FR;
        m_WheelController.RL = m_RL;
        m_WheelController.RR = m_RR;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        GetHardwareReferences();
        ResetMotors();

        InitWheels();

        waitForStart();

        InitMotors();

        Time.Init();
        while(opModeIsActive()) {
            Time.Update();

            telemetry.update();
        }
    }
}
