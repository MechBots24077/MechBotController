package org.firstinspires.ftc.teamcode.motion;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.mcdanielpps.mechframework.motion.MotorController;
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

    public void Update(TelemetryPacket packet) {
        LLift.UpdatePID(RobotConfig.PID_KP, RobotConfig.PID_KI, RobotConfig.PID_KD);
        RLift.UpdatePID(RobotConfig.PID_KP, RobotConfig.PID_KI, RobotConfig.PID_KD);

        LLift.Update(packet, "LLift");
        RLift.Update(packet, "RLift");
    }

    public int GetCurrentPosition() {
        return RLift.GetCurrentPos();
    }

    public int GetCurrentGoal() {
        return RLift.Goal;
    }
}
