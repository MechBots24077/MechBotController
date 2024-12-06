package org.firstinspires.ftc.teamcode.motion;

import com.mcdanielpps.mechframework.motion.MotorController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftController {
    public MotorController LLift = null;
    public MotorController RLift = null;

    public void SetMotors(DcMotor llift, DcMotor rlift) {
        LLift = new MotorController(llift, 3.0, 1.0, 0.0);
        RLift = new MotorController(rlift, 3.0, 1.0, 0.0);
    }

    public void InitMotors() {
        LLift.Init();
        RLift.Init();
    }

    public void MoveToPosition(int position) {
        RLift.Goal = position;
        LLift.Goal = -position;
    }

    public void Update() {
        LLift.Update();
        RLift.Update();
    }

    public int GetCurrentPosition() {
        return RLift.GetCurrentPos();
    }

    public int GetCurrentGoal() {
        return RLift.Goal;
    }
}
