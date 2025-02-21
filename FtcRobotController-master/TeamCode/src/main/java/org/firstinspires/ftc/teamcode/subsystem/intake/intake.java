package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.constants.constants;

public class intake{
    public DcMotorEx left_slide, right_slide;
    public Servo leftServo, rightServo, rollServo;
    private constants consts;
    private double lastError=0;
    public enum SERVO_STATE{
        IN, UP, OUT, UP_2
    }
   public SERVO_STATE state;

    public intake(HardwareMap hardwareMap){
        consts = new constants();
        left_slide = hardwareMap.get(DcMotorEx.class, consts.intake_leftslide_hm);
        right_slide = hardwareMap.get(DcMotorEx.class, consts.intake_rightslide_hm);
        leftServo = hardwareMap.get(Servo.class, consts.intake_left_hm);
        rightServo = hardwareMap.get(Servo.class, consts.intake_right_hm);
        rollServo = hardwareMap.get(Servo.class, consts.intake_roller_hm);
        left_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        state = SERVO_STATE.IN;
        setServos();
    }

    public void setLiftPosition(double ticks){
        right_slide.setTargetPosition((int)ticks);
        left_slide.setTargetPosition((int)ticks);
    }

    public void setState(SERVO_STATE state) {
        this.state = state;
    }

    public void setArmPosition(double pos){
        leftServo.setPosition(pos);
        rightServo.setPosition(1-pos);
    }

    public void runToPosition(int target){
        double error = (double) (target - left_slide.getCurrentPosition());
        double derivative = error-lastError;
        left_slide.setPower((error*consts.iP) + (derivative*consts.iD));
        right_slide.setPower((error*consts.iP) + (derivative*consts.iD));
        lastError = error;
        right_slide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setRollServo(double input){
        rollServo.setPosition((input*0.5)+0.5);
    }

    public void setServos(){
        switch (state){
            case IN:
                leftServo.setPosition(0.1);
                rightServo.setPosition(0.9);
                break;
            case UP:
                leftServo.setPosition(0.5);
                rightServo.setPosition(0.5);
                break;
            case OUT:
                leftServo.setPosition(1);
                rightServo.setPosition(0);
                break;
            case UP_2:
                leftServo.setPosition(0.5);
                rightServo.setPosition(0.5);
                break;
        }
    }
    public void switchState(){
        switch(state){
            case IN:
                state=SERVO_STATE.UP;
                break;
            case UP:
                state=SERVO_STATE.OUT;
                break;
            case OUT:
                state=SERVO_STATE.UP_2;
                break;
            case UP_2:
                state=SERVO_STATE.IN;
                break;
        }
    }

    public void setMotorPowers(double powers){
        if(Math.signum((double) (left_slide.getCurrentPosition() - right_slide.getCurrentPosition())) > consts.outMotorTolerances){
            if(left_slide.getCurrentPosition() > right_slide.getCurrentPosition()){
                if(powers > 0){
                    right_slide.setPower(Math.signum(powers));
                }
                else {
                    left_slide.setPower(-1*Math.signum(powers));
                }
            }
            else{
                if(powers > 0){
                    right_slide.setPower(-1*Math.signum(powers));
                }
                else {
                    left_slide.setPower(Math.signum(powers));
                }
            }
        }
        else{
            right_slide.setPower(powers);
            left_slide.setPower(powers);
        }
    }

}
