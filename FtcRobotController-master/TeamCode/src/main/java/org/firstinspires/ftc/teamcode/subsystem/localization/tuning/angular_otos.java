package org.firstinspires.ftc.teamcode.subsystem.localization.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.localization.localization_SparkFunOTOS;

@Config
@TeleOp(name = "OTOS Angular Tuning (10 revolutions)", group = "tuning")

public class angular_otos extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        localization_SparkFunOTOS find;
        find = new localization_SparkFunOTOS(hardwareMap);

        while(!isStarted()){
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            find.configureOtos(telemetry);
        }
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("rotation", find.myOtos.getPosition().h);
            telemetry.update();
        }
    }
}
