package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.drive.drive;
import org.firstinspires.ftc.teamcode.subsystem.localization.localization;
import org.firstinspires.ftc.teamcode.subsystem.outtake.outtake;

@Config
@Autonomous(name = "Specimen Auto")
public class testauto extends OpMode {

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
        out.clawServo.setPosition(1);
        find.resetHeading();
        out.setArmPosition(0.2);
    }
    @Override
    public void start(){
        runtime.reset();
    }

    @Override
    public void loop(){
        mainAuto();
    }


    public void mainAuto(){
        while(runtime.seconds() > 0 && runtime.seconds() <=3.5) {
            drive.gotoPos(0, -32, find.myOtos.getPosition(), find.imu);
            out.setLiftPosition(100);
            out.setArmPosition(1);
            out.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            out.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            out.setMotorPowers(0.5);
            out.clawServo.setPosition(1);
        }
        while(runtime.seconds() > 3.5 && runtime.seconds() <=5) {
            out.setArmPosition(0.6);
        }
        while(runtime.seconds() > 5 && runtime.seconds() <=5.5) {
            out.clawServo.setPosition(0);
        }
        while(runtime.seconds() > 5.5 && runtime.seconds() <=7.5) {
            out.setArmPosition(0.85);
            drive.gotoPos(0, -5, find.myOtos.getPosition(), find.imu);
        }while(runtime.seconds() > 7.5 && runtime.seconds() <=10.5) {
            out.setArmPosition(0.85);
            drive.gotoPos(-40, -5, find.myOtos.getPosition(), find.imu);
            out.setLiftPosition(0);
        }
    }

    public void squareTest(){
        while(runtime.seconds() >= 0 && runtime.seconds() <=1) {
            drive.gotoPos(0, 10, find.myOtos.getPosition(), find.imu);
        }
        while(runtime.seconds() > 1 && runtime.seconds() <=2) {
            drive.gotoPos(10, 10, find.myOtos.getPosition(), find.imu);

        }
        while(runtime.seconds() > 2 && runtime.seconds() <= 3){
            drive.gotoPos(10, 0, find.myOtos.getPosition(), find.imu);
        }
        while(runtime.seconds() > 3 && runtime.seconds() <= 4){
            drive.gotoPos(0, 0, find.myOtos.getPosition(), find.imu);
        }
    }
}
