package org.firstinspires.ftc.teamcode.subsystem.localization.tuning;

import org.firstinspires.ftc.teamcode.subsystem.localization.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "OTOS testing")
public class linear_otos extends LinearOpMode {
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
            telemetry.addData("x movement", find.myOtos.getPosition().y);
            telemetry.addData("y movement", find.myOtos.getPosition().y);
            telemetry.addData("h movement", find.myOtos.getPosition().y);
            telemetry.update();
        }
    }
}
