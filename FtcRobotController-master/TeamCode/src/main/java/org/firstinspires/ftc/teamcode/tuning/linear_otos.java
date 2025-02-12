package org.firstinspires.ftc.teamcode.tuning;

import org.firstinspires.ftc.teamcode.subsystem.localization.localization_SparkFunOTOS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "OTOS Linear Tuning (120\")")
public class linear_otos extends LinearOpMode {
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
            telemetry.addData("movement", find.myOtos.getPosition().y);
            telemetry.update();
        }
    }
}
