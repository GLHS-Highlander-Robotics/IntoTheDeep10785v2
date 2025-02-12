package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.localization.localization_SparkFunOTOS;

@Config
@Autonomous(name = "Test Auto", group = "Test")
public class testauto extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        //Parts
        localization_SparkFunOTOS find;

        find = new localization_SparkFunOTOS(hardwareMap);

        while(!isStarted()){
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            find.configureOtos(telemetry);
        }
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("x", find.findPosition().x);
            telemetry.addData("y", find.findPosition().y);
            telemetry.addData("h", find.findPosition().h);
            telemetry.update();
        }
    }


}
