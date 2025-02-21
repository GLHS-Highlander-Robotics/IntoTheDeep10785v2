package org.firstinspires.ftc.teamcode.subsystem.localization.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.localization.localization;

@Config
@TeleOp(name = "IMU Angular Test")
public class angular_imu extends OpMode {
    localization find;
    @Override
    public void init(){
        find = new localization(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        find.configureOtos(telemetry);
    }

    @Override
    public void loop(){
        telemetry.addData("rotation", find.imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.update();
    }
}

