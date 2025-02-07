package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.mcdanielpps.mechframework.motion.MotorController;
import com.mcdanielpps.mechframework.util.MechUtil;
import com.mcdanielpps.mechframework.util.PIDController;
import com.mcdanielpps.mechframework.util.Time;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class LiftController {
    public MotorController LLift = null;
    public MotorController RLift = null;

    public void SetMotors(DcMotor llift, DcMotor rlift) {
        LLift = new MotorController(llift, RobotConfig.PID_KP, RobotConfig.PID_KI, RobotConfig.PID_KD);
        RLift = new MotorController(rlift, RobotConfig.PID_KP, RobotConfig.PID_KI, RobotConfig.PID_KD);
    }

    public void InitMotors() {
        LLift.Init();
        RLift.Init();
    }

    public void MoveToPosition(int position) {
        if (position < 0.0)
            return;
        if (position > 4200.0)
            return;

        RLift.Goal = position;
        LLift.Goal = -position;
    }

    public void Update() {
        LLift.UpdatePID(RobotConfig.PID_KP, RobotConfig.PID_KI, RobotConfig.PID_KD);
        RLift.UpdatePID(RobotConfig.PID_KP, RobotConfig.PID_KI, RobotConfig.PID_KD);
        double lPower = LLift.Update("LLift", -1.0f);
        double rPower = RLift.Update("RLift", 1.0f);

//        packet.put("Motor Diff", lPower + rPower);
    }

    public void GoTo(int position, double time)
    {
        double start = GetCurrentPosition();

        double startTime = Time.TimeGetter.currentTimeMillis();
        double endTime = startTime + time;

        while (Math.abs(position - GetCurrentPosition()) > 20.0)
        {
            MoveToPosition((int)(start + (1.0 - (endTime - Time.TimeGetter.currentTimeMillis()) / time)));
            Update();
        }
    }

    //private int m_CurrentPosition = 0;

//    private final PIDController m_PID = new PIDController(RobotConfig.PID_KP, RobotConfig.PID_KI, RobotConfig.PID_KD, 0.02, -100.0, 100.0, 0.005);
//    private long m_LastMeasurement = 0;
//
//    public void Update(TelemetryPacket packet) {
//        long currentTime = Time.TimeGetter.currentTimeMillis();
//        if ((currentTime - m_LastMeasurement) < 5) { return; }
//        m_LastMeasurement = currentTime;
//
//        m_PID.Kp = RobotConfig.PID_KP;
//        m_PID.Ki = RobotConfig.PID_KI;
//        m_PID.Kd = RobotConfig.PID_KD;
//
//        m_CurrentPosition = RLift.Motor.getCurrentPosition();
//
//        double output = m_PID.Update((double)RLift.Goal, (double)m_CurrentPosition);
//        RLift.Motor.setPower(output / 100.0);
//        LLift.Motor.setPower(-output / 100.0);
//
//        packet.put("RLift Position", m_CurrentPosition);
//        double lPos = LLift.Motor.getCurrentPosition();
//        packet.put("LLift Position", lPos * -1.0f);
//        packet.put("Goal", RLift.Goal);
//        packet.put("Power", output / 100.0);
//
//        packet.put("Diff", m_CurrentPosition + lPos);
//    }

    public int GetCurrentPosition() {
        return RLift.GetCurrentPos();
    }

    public int GetCurrentGoal() {
        return RLift.Goal;
    }
}
