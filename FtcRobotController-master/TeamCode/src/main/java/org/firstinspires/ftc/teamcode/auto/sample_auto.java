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
import org.firstinspires.ftc.teamcode.subsystem.intake.intake;
import org.firstinspires.ftc.teamcode.subsystem.localization.localization;
import org.firstinspires.ftc.teamcode.subsystem.outtake.outtake;

@Config
@Autonomous(name = "Sample Auto")
public class sample_auto extends OpMode {

    localization find;
    org.firstinspires.ftc.teamcode.subsystem.drive.drive drive;
    outtake out;
    intake in;
    ElapsedTime runtime;

    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        find = new localization(hardwareMap);
        drive = new drive(hardwareMap);
        out = new outtake(hardwareMap);
        in = new intake(hardwareMap);
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
        telemetry.update();
    }


    public void mainAuto(){
        while(runtime.seconds() > 0 && runtime.seconds() <=1.5) {
            drive.gotoPos(-19, 15, find.myOtos.getPosition(), find.imu, 0);
            out.setLiftPosition(2000);
            out.setArmPosition(0.9);
            out.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            out.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            out.setMotorPowers(0.7);
            in.setState(intake.SERVO_STATE.OUT);
            in.setServos();
            out.clawServo.setPosition(1);
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }
        while(runtime.seconds() > 1.5 && runtime.seconds() <=3 ) {
            drive.gotoPos(-32, 9, find.myOtos.getPosition(), find.imu, 0);
            out.clawServo.setPosition(1);
            out.setLiftPosition(2000);
        }
        while(runtime.seconds() > 3 && runtime.seconds() <=3.5) {
            out.clawServo.setPosition(0);
            drive.stopBot();

        }
        while(runtime.seconds() > 3.5 && runtime.seconds() <=5.5) {
            out.setMotorPowers(0.7);
            out.clawServo.setPosition(0);
            drive.gotoPos(-19.9, 28, find.myOtos.getPosition(), find.imu, 0);
            in.setState(intake.SERVO_STATE.OUT);
            in.setServos();
            in.setRollServo(1);
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }
        while(runtime.seconds() > 5.5 && runtime.seconds() <=7.5) {
            out.setLiftPosition(0);
            out.setArmPosition(0);
            in.setState(intake.SERVO_STATE.IN);
            in.setServos();
            in.setRollServo(1);
            drive.stopBot();
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }
        while(runtime.seconds() > 7.5 && runtime.seconds() <=8) {
            in.setRollServo(-1);
            drive.gotoPos(-22, 12, find.myOtos.getPosition(), find.imu, 0);
            drive.stopBot();
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }
        while(runtime.seconds() > 8 && runtime.seconds() <=9) {
            out.clawServo.setPosition(1);
            drive.stopBot();
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }
        while(runtime.seconds() > 9 && runtime.seconds() <=11) {
            drive.gotoPos(-27, 4, find.myOtos.getPosition(), find.imu, 0);
            out.clawServo.setPosition(1);
            out.setLiftPosition(2000);
            out.setArmPosition(0.9);
            out.setMotorPowers(0.7);
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }
        while(runtime.seconds() > 11 && runtime.seconds() <= 12){
            out.clawServo.setPosition(0);
            drive.stopBot();
            in.setState(intake.SERVO_STATE.OUT);
            in.setServos();
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }while(runtime.seconds() > 12 && runtime.seconds() <=14) {
            drive.gotoPos(-3 , 28, find.myOtos.getPosition(), find.imu, 0);
            out.setMotorPowers(0.7);
            out.clawServo.setPosition(0);
            in.setRollServo(1);
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }
        while(runtime.seconds() > 14 && runtime.seconds() <=16) {
            out.setLiftPosition(0);
            out.setArmPosition(0);
            in.setState(intake.SERVO_STATE.IN);
            in.setServos();
            in.setRollServo(1);
            drive.stopBot();
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }
        while(runtime.seconds() > 16 && runtime.seconds() <=18.5) {
            in.setRollServo(-1);
            drive.gotoPos(-20.1, 12, find.myOtos.getPosition(), find.imu, 0);
            drive.stopBot();
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }
        while(runtime.seconds() > 18.5 && runtime.seconds() <=19.5) {
            out.clawServo.setPosition(1);
            drive.stopBot();
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }
        while(runtime.seconds() > 19.5 && runtime.seconds() <=21.5) {
            drive.gotoPos(-28, 4, find.myOtos.getPosition(), find.imu, 0);
            out.setLiftPosition(2000);
            out.setArmPosition(0.9);
            out.setMotorPowers(0.7);
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }
        while(runtime.seconds() > 21.5 && runtime.seconds() <= 22.5){
            out.clawServo.setPosition(0);
            drive.stopBot();
            telemetry.addData("IMU Heading", find.imu.getRobotYawPitchRollAngles().getYaw());
        }
        while (runtime.seconds() > 22.5 && runtime.seconds() <= 24.5) {
            drive.gotoPos(-25, 20, find.myOtos.getPosition(), find.imu, 0);
        }
        while (runtime.seconds() > 24.5 && runtime.seconds() <= 26.5) {
            out.setLiftPosition(0);
            out.setArmPosition(0);
            in.setRollServo(0);
        }
    }
}
