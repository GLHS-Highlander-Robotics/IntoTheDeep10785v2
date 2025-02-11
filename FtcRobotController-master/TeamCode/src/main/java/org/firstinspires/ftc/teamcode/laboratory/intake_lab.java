package org.firstinspires.ftc.teamcode.laboratory;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystem.intake.intake;

@Config
@TeleOp(name = "Intake Lab")
public class intake_lab extends OpMode {

    intake in;
    boolean rt;

    @Override
    public void init(){
        in = new intake(hardwareMap);
        in.left_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        rt = false;
    }
    @Override
    public void loop(){
        double power = gamepad1.left_stick_y;
        if(Math.abs(power) <= 0.1){power = 0;}
        in.setMotorPowers(power);
        if(gamepad1.right_trigger>=0.3 && !rt){
            rt = true;
            in.switchState();
        }
        else if (gamepad1.right_trigger < 0.3) {rt = false;}
        in.setServos();
        in.setRollServo(gamepad1.right_stick_y);
    }
}
