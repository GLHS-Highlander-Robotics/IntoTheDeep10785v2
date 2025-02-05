package org.firstinspires.ftc.teamcode.subsystem.outtake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.constants.constants;

public class outtake {
    public DcMotorEx leftMotor, rightMotor;
    public Servo leftServo, rightServo, clawServo;
    private constants consts;

    public outtake(HardwareMap hardwareMap){
        consts = new constants();
        leftMotor = hardwareMap.get(DcMotorEx.class, consts.outtake_leftslide_hm);
        rightMotor = hardwareMap.get(DcMotorEx.class, consts.outtake_rightslide_hm);
        leftServo = hardwareMap.get(Servo.class, consts.outtake_left_hm);
        rightServo = hardwareMap.get(Servo.class, consts.outtake_right_hm);
        clawServo = hardwareMap.get(Servo.class, consts.outtake_claw_hm);
    }



}
