package org.firstinspires.ftc.teamcode.subsystem.outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        clawServo.setPosition(1);
        leftServo.setPosition(0);
        rightServo.setPosition(1);
    }

    public void setLiftPosition(double ticks){
        rightMotor.setTargetPosition((int)ticks);
        leftMotor.setTargetPosition((int)ticks);
    }

    public void setArmPosition(double pos){
        leftServo.setPosition(pos);
        rightServo.setPosition(1 - pos);
    }

    public void setMotorState(DcMotor.RunMode runMode){
        leftMotor.setMode(runMode);
        rightMotor.setMode(runMode);
    }

    public void setMotorPowers(double powers){
        if(((Math.max(leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition())) > consts.outtakeLimit && powers >= 0) || ((Math.max(leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition()))<=-3)&&powers<=0){
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }
        if(Math.signum((double) (leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition())) > consts.outMotorTolerances){
            if(leftMotor.getCurrentPosition() > rightMotor.getCurrentPosition()){
                if(powers > 0){
                    rightMotor.setPower(Math.signum(powers));
                }
                else {
                    leftMotor.setPower(-1 * Math.signum(powers));
                }
            }
            else{
                if(powers > 0){
                    rightMotor.setPower(-1 * Math.signum(powers));
                }
                else {
                    leftMotor.setPower(Math.signum(powers));
                }
            }
        }
        else{
            rightMotor.setPower(powers);
            leftMotor.setPower(powers);
        }
    }
    public void resetMotors(){
        if(Math.max(rightMotor.getCurrent(CurrentUnit.MILLIAMPS), leftMotor.getCurrent(CurrentUnit.MILLIAMPS)) >= consts.currentDrawThreshold){

        }
        else{
            setMotorPowers(-1);
        }
    }

}
