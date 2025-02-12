package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drive.drive;
import org.firstinspires.ftc.teamcode.subsystem.localization.localization_SparkFunOTOS;
import org.firstinspires.ftc.teamcode.subsystem.intake.intake;
import org.firstinspires.ftc.teamcode.subsystem.constants.constants;
import org.firstinspires.ftc.teamcode.subsystem.calculations.Basic;

@Config
@TeleOp(name = "Two Player Teleop >:)")
public class twoPlayerTeleop extends OpMode {

    //Subsystems
    drive drive;
    localization_SparkFunOTOS find;
    intake intake;

    //Constants
    constants consts;

    Basic basic;

    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new drive(hardwareMap);
        find = new localization_SparkFunOTOS(hardwareMap);
        consts = new constants();
    }

    @Override
    //
    public void loop(){
        updateDrive();
        getTelemetry();
        telemetry.update();
    }

    public void updateDrive(){
        double forward;
        double strafe;
        double turn;
        if(basic.withinDeadzone(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x, consts.gp1dead)) {
            forward = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right){
                if (gamepad1.dpad_up) {
                    forward = -consts.dpadSpeed;
                }
                if (gamepad1.dpad_down) {
                    forward = consts.dpadSpeed;
                }
                if (gamepad1.dpad_left) {
                    strafe = -consts.dpadSpeed;
                }
                if (gamepad1.dpad_right) {
                    strafe = consts.dpadSpeed;
                }
            }
            if(consts.useFieldCentric) {
                drive.FieldCentricDrive(forward, strafe, turn, find.myOtos.getPosition().h);
            } else {
                drive.RobotCentricDrive(forward, strafe, turn);
            }
            telemetry.addData("turn", turn);
        }
        else if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            forward = 0;
            strafe = 0;
            turn = 0;
            if (gamepad1.dpad_up) {
                forward = -0.3;
            }
            if (gamepad1.dpad_down) {
                forward = 0.3;
            }
            if (gamepad1.dpad_left) {
                strafe = -0.3;
            }
            if (gamepad1.dpad_right) {
                strafe = 0.3;
            }
            if(consts.useFieldCentric) {
                drive.FieldCentricDrive(forward, strafe, turn, find.myOtos.getPosition().h);
            } else {
                drive.RobotCentricDrive(forward, strafe, turn);
            }
        }
        else drive.stopBot();
    }

    public void getTelemetry(){
        telemetry.addData("x pos", find.myOtos.getPosition().x);
        telemetry.addData("y pos", find.myOtos.getPosition().y);
        telemetry.addData("h pos", find.myOtos.getPosition().h);
    }
}
