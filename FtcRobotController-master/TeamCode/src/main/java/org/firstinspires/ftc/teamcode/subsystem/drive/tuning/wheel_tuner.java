package org.firstinspires.ftc.teamcode.subsystem.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drive.drive;

@Config
@TeleOp(name = "Wheel Direction Tuner", group = "tuning")
public class wheel_tuner extends OpMode {
    drive drive;

    @Override
    public void init(){
        drive = new drive(hardwareMap);
    }
    @Override
    public void loop(){
        if(gamepad1.y){
            drive.FrontL.setPower(0.2);
        }
        else if(gamepad1.b){
            drive.FrontR.setPower(0.2);
        }
        else if(gamepad1.x){
            drive.BackL.setPower(0.2);
        }
        else if(gamepad1.a){
            drive.BackR.setPower(0.2);
        }
        else{
            drive.stopBot();
        }
    }
}
