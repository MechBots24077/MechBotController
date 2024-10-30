package org.firstinspires.ftc.teamcode.motion;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftController {
    public DcMotor LLift = null;
    public DcMotor RLift = null;

    public void ResetMotors() {
        LLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LLift.setTargetPosition(0);
        RLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RLift.setTargetPosition(0);
    }

    public void InitMotors() {
        LLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LLift.setPower(1.0);
        RLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RLift.setPower(1.0);
    }

    public void MoveToPosition(int position) {
        RLift.setTargetPosition(position);
        LLift.setTargetPosition(-position);
    }

    public int GetCurrentPosition() {
        return RLift.getCurrentPosition();
    }
}
