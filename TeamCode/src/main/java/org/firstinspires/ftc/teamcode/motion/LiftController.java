package org.firstinspires.ftc.teamcode.motion;

import com.mcdanielpps.mechframework.motion.MotorController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftController {
    public MotorController LLift = null;
    public MotorController RLift = null;

    public void SetMotors(DcMotor llift, DcMotor rlift) {
        LLift = new MotorController(llift, 0.0, 0.0, 0.0);
        RLift = new MotorController(rlift, 0.0, 0.0, 0.0);
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

    public int GetCurrentGoaL() {
        return RLift.Goal;
    }
}

//public class LiftController {
//    public DcMotor LLift = null;
//    public DcMotor RLift = null;
//
//    public void ResetMotors() {
//        LLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LLift.setTargetPosition(0);
//        RLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RLift.setTargetPosition(0);
//    }
//
//    public void InitMotors() {
//        LLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LLift.setPower(1.0);
//        RLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RLift.setPower(1.0);
//    }
//
//    public void MoveToPosition(int position) {
//        RLift.setTargetPosition(position);
//        LLift.setTargetPosition(-position);
//    }
//
//    public int GetCurrentPosition() {
//        return RLift.getCurrentPosition();
//    }
//}
