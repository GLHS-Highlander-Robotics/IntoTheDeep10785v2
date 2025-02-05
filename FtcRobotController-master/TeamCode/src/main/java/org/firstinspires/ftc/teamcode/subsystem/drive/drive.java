package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    }

    public double pidHeading(double target, double kp, double ki, double kd, double current){
        double error = target - current;
        integral += error;
        double derivative = error - lastErrorHeading;
        if(error > Math.PI){
            error -= Math.PI*2;
        } else if(error < -Math.PI){
            error += Math.PI*2;
        }
        double correction = (error * kp) + (integral * ki) + (derivative * kd);
        lastErrorHeading = error;
        return correction;
    }

    public double pdfTranslate(double kp, double kd, double kf, double error){
        double derivative = error - lastErrorDrive;
        double correction = (error*kp) + (derivative * kd);
        correction += Math.signum(error)*kf;
        lastErrorDrive = error;
        return correction;
    }

    public void resetHeadingIntegral(){
        integral = 0;
    }

    public void gotoPos(double targX, double targY, double targH, double speed, SparkFunOTOS.Pose2D curPos){
        Vector2D drive = new Vector2D(0, targX- curPos.x, targY-curPos.y);
        Vector2D rotatedDrive = drive.rotateVector(curPos.h);

        double inputTurn = pidHeading(targH, hP, hI, hD, curPos.h);
        double driveCorrection = pdfTranslate(dP, dD, 0, rotatedDrive.y);
        double strafeCorrection = pdfTranslate(sP, sD, 0, rotatedDrive.x);

        FrontL.setPower((driveCorrection+strafeCorrection+inputTurn)*speed);
        FrontR.setPower((driveCorrection-strafeCorrection-inputTurn)*speed);
        BackL.setPower((driveCorrection-strafeCorrection+inputTurn)*speed);
        BackR.setPower((driveCorrection+strafeCorrection-inputTurn)*speed);
    }

    public void driveFromController(double drive, double strafe, double turn){
        FrontL.setPower((drive+strafe+turn));
        FrontR.setPower((drive-strafe-turn));
        BackL.setPower((drive-strafe+turn));
        BackR.setPower((drive+strafe-turn));
    }
}
