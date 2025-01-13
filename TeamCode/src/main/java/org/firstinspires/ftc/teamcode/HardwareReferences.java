package org.firstinspires.ftc.teamcode;

import com.mcdanielpps.mechframework.util.RobotSystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareReferences {
    public DcMotor FL;
    public DcMotor FR;
    public DcMotor RL;
    public DcMotor RR;

    public DcMotor LeftLift;
    public DcMotor RightLift;

    public CRServo Extension;
    public Servo Claw;
    public Servo Wrist;

    //public DigitalChannel LiftLimit;
    //public DigitalChannel ExtensionLimit;

    public HardwareReferences() {
        HardwareMap hardwareMap = RobotSystem.getInstance().GetHardwareMap();

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        RL = hardwareMap.get(DcMotor.class, "RL");
        RR = hardwareMap.get(DcMotor.class, "RR");

        LeftLift = hardwareMap.get(DcMotor.class, "LLift");
        RightLift = hardwareMap.get(DcMotor.class, "RLift");

        Extension = hardwareMap.get(CRServo.class, "Extension");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Wrist = hardwareMap.get(Servo.class, "Wrist");

        //LiftLimit = hardwareMap.get(DigitalChannel.class, "LLimit");
        //ExtensionLimit = hardwareMap.get(DigitalChannel.class, "ELimit");
    }

    public DcMotor OdometryLeft() { return RR; }
    public DcMotor OdometryCenter() { return RL; }
    public DcMotor OdometryRight() { return FL; }
}
