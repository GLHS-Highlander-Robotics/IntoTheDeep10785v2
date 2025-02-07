package org.firstinspires.ftc.teamcode.laboratory;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystem.outtake.outtake;
import org.firstinspires.ftc.teamcode.subsystem.constants.constants;

@Config
@TeleOp(name = "Reset Motor Test", group = "Laboratory")
public class resetMotors extends OpMode {

    public outtake out;

    public constants consts;

    public enum slideState{
        INIT,
        CONTROLLED,
        RESET
    }

    public slideState state = slideState.INIT;

    @Override
    public void init(){
        out = new outtake(hardwareMap);
        consts = new constants();
    }

    @Override
    public void loop(){
        slide();
    }

    public void slide(){
        switch (state){
            case INIT:
                if((gamepad1.left_stick_x*gamepad1.left_stick_x+gamepad1.left_stick_y*gamepad1.left_stick_y)>consts.gp1dead *consts.gp1dead || gamepad1.a){
                    state = slideState.CONTROLLED;
                }
                break;
            case CONTROLLED:
                if(gamepad1.a){
                    state = slideState.RESET;
                }
                out.setMotorPowers(gamepad1.left_stick_y);
                break;
            case RESET:
                out.resetMotors();
                if(gamepad1.b){
                    out.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    out.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    out.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    out.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    state = slideState.INIT;
                }
        }
    }

}
