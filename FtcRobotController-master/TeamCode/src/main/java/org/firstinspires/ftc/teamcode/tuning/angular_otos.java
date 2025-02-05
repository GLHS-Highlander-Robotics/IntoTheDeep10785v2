package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.subsystem.localization.localization;

@Config
@TeleOp(name = "OTOS Angular Tuning (10 revolutions)")
public class angular_otos extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        localization find;
        find = new localization(hardwareMap);

        while(!isStarted()){
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            find.configureOtos(telemetry);
        }
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("rotation", find.getPosition().h);
            telemetry.update();
        }
    }
}

