package org.firstinspires.ftc.teamcode.laboratory;


// Create Sample PID code for motor, set constants to 0

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.localization.localization;
import org.firstinspires.ftc.teamcode.subsystem.drive.drive;
import org.firstinspires.ftc.teamcode.subsystem.outtake.outtake;

@Config
@Autonomous(name = "PID AUTO TEST", group = "testing")
public class PID_Test extends OpMode {

    localization find;
    drive drive;
    outtake out;
    ElapsedTime runtime;

    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        find = new localization(hardwareMap);
        drive = new drive(hardwareMap);
        out = new outtake(hardwareMap);
        find.configureOtos(telemetry);
        runtime = new ElapsedTime();
        telemetry.addData("targy", 0);
        telemetry.addData("y pos", 0);
        telemetry.update();
    }

    @Override
    public void loop(){
        runtime.reset();
        while(runtime.seconds() >= 0 && runtime.seconds() <=2.5) {
            drive.gotoPos(0, 10, find.myOtos.getPosition(), find.imu);
            telemetry.addData("targy", 10);
            telemetry.addData("y pos", find.myOtos.getPosition().y);
            telemetry.update();
        }
        while(runtime.seconds() >= 2.5 && runtime.seconds() <=5) {
            drive.gotoPos(0, 0, find.myOtos.getPosition(), find.imu);
            telemetry.addData("targy", 0);
            telemetry.addData("y pos", find.myOtos.getPosition().y);
            telemetry.update();
        }
    }

}
