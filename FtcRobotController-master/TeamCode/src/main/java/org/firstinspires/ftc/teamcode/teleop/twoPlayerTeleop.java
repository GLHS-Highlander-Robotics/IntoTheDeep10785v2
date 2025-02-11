package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drive.drive;
import org.firstinspires.ftc.teamcode.subsystem.localization.localization;
import org.firstinspires.ftc.teamcode.subsystem.intake.intake;
import org.firstinspires.ftc.teamcode.subsystem.constants.constants;

@Config
@TeleOp(name = "Two Player Teleop >:)")
public class twoPlayerTeleop extends OpMode {

    //Subsystems
    drive drive;
    localization find;
    intake intake;

    //Constants
    constants consts;

    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new drive(hardwareMap);
        find = new localization(hardwareMap);
        consts = new constants();
    }

    @Override
    public void loop(){
        updateDrive();
        getTelemetry();
        telemetry.update();
    }

    public void updateDrive(){
        if(gamepad1.left_stick_x*gamepad1.left_stick_x+gamepad1.left_stick_y*gamepad1.left_stick_y >= consts.gp1dead*consts.gp1dead || Math.abs(gamepad1.right_stick_x) >= consts.gp1dead) {
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            drive.FieldCentricDrive(forward, strafe, turn, find.myOtos.getPosition().h);
            telemetry.addData("turn", turn);
        }
        else drive.stopBot();
    }

    public void getTelemetry(){
        telemetry.addData("x pos", find.myOtos.getPosition().x);
        telemetry.addData("y pos", find.myOtos.getPosition().y);
        telemetry.addData("h pos", find.myOtos.getPosition().h);
    }
}
