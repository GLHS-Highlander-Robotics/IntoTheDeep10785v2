package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.constants.constants;

public class intake{
    public DcMotorEx left_slide, right_slide;
    public Servo leftServo, rightServo, rollServo;
    private constants consts;
    private double lastError=0;
    enum SERVO_STATE{
        IN, UP, OUT
    }
    SERVO_STATE state;

    public intake(HardwareMap hardwareMap){
        consts = new constants();
        left_slide = hardwareMap.get(DcMotorEx.class, consts.intake_leftslide_hm);
        right_slide = hardwareMap.get(DcMotorEx.class, consts.intake_rightslide_hm);
        leftServo = hardwareMap.get(Servo.class, consts.intake_left_hm);
        rightServo = hardwareMap.get(Servo.class, consts.intake_right_hm);
        rollServo = hardwareMap.get(Servo.class, consts.intake_roller_hm);
        state = SERVO_STATE.IN;
        setServos();
    }

    public void runToPosition(int target){
        double error = (double) (target - left_slide.getCurrentPosition());
        double derivative = error-lastError;
        left_slide.setPower((error*consts.iP) + (derivative*consts.iD));
        right_slide.setPower((error*consts.iP) + (derivative*consts.iD));
        lastError = error;
    }

    public void setRollServo(double input){
        rollServo.setPosition((input*0.5)+0.5);
    }

    public void setServos(){
        switch (state){
            case IN:
                leftServo.setPosition(0);
                rightServo.setPosition(1);
                break;
            case UP:
                leftServo.setPosition(0.5);
                rightServo.setPosition(0.5);
                break;
            case OUT:
                leftServo.setPosition(1);
                rightServo.setPosition(0);
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
