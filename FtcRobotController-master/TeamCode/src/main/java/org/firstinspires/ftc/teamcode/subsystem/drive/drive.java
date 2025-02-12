package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.subsystem.calculations.Vector2D;
import org.firstinspires.ftc.teamcode.subsystem.constants.constants;

public class drive {
    public DcMotorEx FrontL, FrontR, BackL, BackR;
    private double integral = 0;
    private double lastErrorHeading = 0;
    private double lastErrorDrive = 0;
    constants consts;

    // Constants
    double hP = 0;
    double hI = 0;
    double hD = 0;
    double sP = 0;
    double sD = 0;
    double dP = 0;
    double dD = 0;


    public drive(HardwareMap hardwareMap){
        consts = new constants();

        FrontL = hardwareMap.get(DcMotorEx.class, consts.fl_hm);
        FrontR = hardwareMap.get(DcMotorEx.class, consts.fr_hm);
        BackL = hardwareMap.get(DcMotorEx.class, consts.bl_hm);
        BackR = hardwareMap.get(DcMotorEx.class, consts.br_hm);

        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double pidHeading(double target, double kp, double ki, double kd, double current){
        double error = target - current;
        integral += error;
        double derivative = error - lastErrorHeading;
        if(error > Math.PI){
            error -= Math.PI * 2;
        } else if(error < -Math.PI){
            error += Math.PI * 2;
        }
        double correction = (error * kp) + (integral * ki) + (derivative * kd);
        lastErrorHeading = error;
        return correction;
    }

    public double pdfTranslate(double kp, double kd, double kf, double error){
        double derivative = error - lastErrorDrive;
        double correction = (error * kp) + (derivative * kd);
        correction += Math.signum(error) * kf;
        lastErrorDrive = error;
        return correction;
    }

    public void resetHeadingIntegral(){
        integral = 0;
    }

    public void gotoPos(double targetX, double targetY, double targetH, double speed, SparkFunOTOS.Pose2D curPos){

        Vector2D drive = new Vector2D(0, targetX - curPos.x, targetY - curPos.y);
        Vector2D rotatedDrive = drive.rotateVector(curPos.h);

        double inputTurn = pidHeading(targetH, hP, hI, hD, curPos.h);
        double driveCorrection = pdfTranslate(dP, dD, 0, rotatedDrive.y);
        double strafeCorrection = pdfTranslate(sP, sD, 0, rotatedDrive.x);

        FrontL.setPower((driveCorrection + strafeCorrection + inputTurn) * speed);
        FrontR.setPower((driveCorrection - strafeCorrection - inputTurn) * speed);
        BackL.setPower((driveCorrection - strafeCorrection + inputTurn) * speed);
        BackR.setPower((driveCorrection + strafeCorrection - inputTurn) * speed);
    }

    public void RobotCentricDrive(double drive, double strafe, double turn){
        double max = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1); // Keeps bounds
        FrontL.setPower((drive - strafe - turn) / max);
        FrontR.setPower((drive + strafe + turn) / max);
        BackL.setPower((drive + strafe - turn) / max);
        BackR.setPower((drive - strafe + turn) / max);
    }

    public void FieldCentricDrive(double drive, double strafe, double turn, double botHeading){

        double botHeading_rad = Math.toRadians(botHeading); // Ensures botHeading is in radians
        double rotY = strafe * Math.sin(-botHeading_rad) - drive * Math.cos(-botHeading_rad);
        double rotX = strafe * Math.cos(-botHeading_rad) + drive * Math.sin(-botHeading_rad);
        RobotCentricDrive(rotY, rotX, turn);
        //RobotCentricDrive(Math.cos(Math.atan2(strafe, drive) + Math.toRadians(-botHeading)), Math.sin(Math.atan2(strafe, drive) + Math.toRadians(-botHeading)), turn);
    }

    public void stopBot(){
        FrontL.setPower(0);
        FrontR.setPower(0);
        BackL.setPower(0);
        BackR.setPower(0);
    }
}