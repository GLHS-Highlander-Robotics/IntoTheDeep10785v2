package org.firstinspires.ftc.teamcode.laboratory;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystem.outtake.outtake;

@Config
@TeleOp(name = "Outtake Lab")
public class outtake_lab extends OpMode {

    outtake out;
    boolean a, b;
    int state1 = 0;
    int state2 = 0;

    @Override
    public void init(){
        out = new outtake(hardwareMap);
        out.clawServo.setPosition(1);
        out.leftServo.setPosition(1);
    }
    @Override
    public void loop(){
        double power = -gamepad2.left_stick_y;
        if(Math.abs(power) <= 0.1 || (Math.max(out.leftMotor.getCurrentPosition(), out.rightMotor.getCurrentPosition())>2000) || (Math.min(out.leftMotor.getCurrentPosition(), out.rightMotor.getCurrentPosition())< -3) || (Math.max(out.leftMotor.getCurrent(CurrentUnit.MILLIAMPS), out.rightMotor.getCurrent(CurrentUnit.MILLIAMPS))>5000)){
            power = 0;
        }
        out.setMotorPowers(power);
        if(gamepad2.a && !a){
            switch (state1) {
                case 0:
                    out.rightServo.setPosition(0);
                    out.leftServo.setPosition(1);
                    a = true;
                    state1 = 1;
                    break;
                case 1:
                    out.rightServo.setPosition(0.35);
                    out.leftServo.setPosition(0.65);
                    a = true;
                    state1 = 2;
                    break;
                case 2:
                    out.rightServo.setPosition(1);
                    out.leftServo.setPosition(0);
                    a = true;
                    state1 = 0;
                    break;
            }
        }
        else if (!gamepad1.a){
            a = false;
        }
        if(gamepad2.b && !b){
            switch (state2) {
                case 0:
                    out.clawServo.setPosition(0);
                    b = true;
                    state2 = 1;
                    break;
                case 1:
                    out.clawServo.setPosition(1);
                    a = true;
                    state2 = 0;
                    break;
            }
        }
        else if (! gamepad1.b){
            b = false;
        }
        telemetry.addData("Left Motor Power", out.leftMotor.getPower());
        telemetry.addData("Left Motor Current Draw", out.leftMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Right Motor Power", out.rightMotor.getPower());
        telemetry.addData("Right Motor Current Draw", out.rightMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Right Servo Position", out.rightServo.getPosition());
        telemetry.update();
    }
}
