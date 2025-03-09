package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystem.calculations.Vector2D;
import org.firstinspires.ftc.teamcode.subsystem.constants.constants;

public class drive {
    public DcMotorEx FrontL, FrontR, BackL, BackR;
    public double integral = 0;
    public double lastErrorHeading = 0;
    public double lastErrorDrive = 0;
    public double lastErrorStrafe = 0;
    public constants consts;



    public drive(HardwareMap hardwareMap){
        consts = new constants();
        FrontL = hardwareMap.get(DcMotorEx.class, consts.fl_hm);
        FrontR = hardwareMap.get(DcMotorEx.class, consts.fr_hm);
        BackL = hardwareMap.get(DcMotorEx.class, consts.bl_hm);
        BackR = hardwareMap.get(DcMotorEx.class, consts.br_hm);
        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public double pidHeading(double target, double kp, double ki, double kd, double current){
        double error = target - current;
        double derivative = error - lastErrorHeading;
        if(error > Math.PI){
            error -= Math.PI*2;
        } else if(error < -Math.PI){
            error += Math.PI*2;
        }
        double correction = (error * kp) + (integral * ki) + (derivative * kd);
        lastErrorHeading = error;
        if(correction <= 1) {
            integral += error;
        }
        return correction;
    }

    public double pdfTranslate(double kp, double kd, double kf, double error){
        double derivative = error - lastErrorDrive;
        double correction = (error*kp) + (derivative * kd);
        correction += Math.signum(error)*kf;
        lastErrorDrive = error;
        return correction;
    }
    public double strafepdf(double kp, double kd, double kf, double error){
        double derivative = error - lastErrorStrafe;
        double correction = (error*kp) + (derivative * kd);
        correction += Math.signum(error)*kf;
        lastErrorStrafe = error;
        return correction;
    }

    public void resetHeadingIntegral(){
        integral = 0;
    }


    public void gotoPos(double targX, double targY, SparkFunOTOS.Pose2D curPos, IMU imu){
        //REVISE!!!!!!!!!!

        // Gets current bot heading based on angle in RADIANS from field orientation
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        double driveCorrection = pdfTranslate(consts.dP, consts.dD, 0, targY-curPos.y);
        double strafeCorrection = strafepdf(consts.sP, consts.sD, 0, targX-curPos.x);

        driveBot(driveCorrection,strafeCorrection, 0,5);
    }
    public void rotateBot(double targH, SparkFunOTOS.Pose2D curPos){
        double inputTurn = pidHeading(targH, consts.hP, consts.hI, consts.hD, curPos.h);
        driveBot(0, 0, inputTurn, 5);
    }


    public void gotoPos(double targX, double targY, SparkFunOTOS.Pose2D curPos, IMU imu, double angleToMaintain){
        //REVISE!!!!!!!!!!

        // Gets current bot heading based on angle in RADIANS from field orientation
        double botHeading = curPos.h;


        double driveCorrection = pdfTranslate(consts.dP, consts.dD, 0, targY-curPos.y);
        double strafeCorrection = strafepdf(consts.sP, consts.sD, 0, targX-curPos.x);
        double inputTurn = pidHeading(angleToMaintain, consts.hP, consts.hI, consts.hD, -botHeading);

        driveBot(driveCorrection,strafeCorrection, inputTurn,(5));
    }

    public void RobotCentricDrive(double drive, double strafe, double turn){
        double max = Math.max(Math.max(Math.abs(drive+strafe+turn), Math.abs(drive+strafe-turn)),Math.max(Math.abs(drive-strafe+turn),Math.abs(drive-strafe-turn)));
        FrontL.setPower((drive+strafe+turn)/max);
        FrontR.setPower((drive-strafe-turn)/max);
        BackL.setPower((drive-strafe+turn)/max);
        BackR.setPower((drive+strafe-turn)/max);
    }

    public void FieldCentricDrive(double drive, double strafe, double turn, IMU imu){
        // Gets current bot heading based on angle in RADIANS from field orientation
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        // Used to interpret
        // To calculate y rotation,
        // To calculate x rotation,
        double rotY = strafe * Math.sin(-botHeading) - drive * Math.cos(-botHeading);
        double rotX = strafe * Math.cos(-botHeading) + drive * Math.sin(-botHeading);


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        driveBot(rotY, rotX, turn, denominator);
    }


    public void FieldCentricDrive(double drive, double strafe, double turn, SparkFunOTOS OTOS){
        // Gets current bot heading based on angle in RADIANS from field orientation
        double botHeading = Math.toRadians(OTOS.getPosition().h);

        // Rotate the movement direction counter to the bot's rotation
        // Used to interpret
        // To calculate y rotation,
        // To calculate x rotation,
        double rotY = strafe * Math.sin(-botHeading) - drive * Math.cos(-botHeading);
        double rotX = strafe * Math.cos(-botHeading) + drive * Math.sin(-botHeading);


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        driveBot(rotY, rotX, turn, denominator);
    }
    public void FieldCentricDrive(double drive, double strafe, double turn, SparkFunOTOS OTOS, double maxSpeed){
        // Gets current bot heading based on angle in RADIANS from field orientation
        double botHeading = Math.toRadians(OTOS.getPosition().h);

        // Rotate the movement direction counter to the bot's rotation
        // Used to interpret
        // To calculate y rotation,
        // To calculate x rotation,
        double rotY = strafe * Math.sin(-botHeading) - drive * Math.cos(-botHeading);
        double rotX = strafe * Math.cos(-botHeading) + drive * Math.sin(-botHeading);


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        driveBot(rotY, rotX, turn, denominator/maxSpeed);
    }

    public void driveBot(double forward, double strafe, double rotate, double limiter) {
        FrontL.setPower((forward + strafe + rotate)/limiter);
        BackL.setPower((forward - strafe + rotate)/limiter);
        FrontR.setPower((forward - strafe - rotate)/limiter);
        BackR.setPower((forward + strafe - rotate)/limiter);
    }

    public void stopBot(){
        FrontL.setPower(0);
        FrontR.setPower(0);
        BackL.setPower(0);
        BackR.setPower(0);
    }
}
