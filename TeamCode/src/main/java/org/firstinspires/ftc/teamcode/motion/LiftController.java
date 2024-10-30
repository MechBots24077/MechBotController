package org.firstinspires.ftc.teamcode.motion;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftController {
    public DcMotor LLift = null;
    public DcMotor RLift = null;

    public void ResetMotors() {
        LLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void InitMotors() {
        LLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
