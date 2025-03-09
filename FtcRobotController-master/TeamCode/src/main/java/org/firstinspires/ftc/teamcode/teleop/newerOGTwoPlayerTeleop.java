package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystem.constants.constants;
import org.firstinspires.ftc.teamcode.subsystem.drive.drive;
import org.firstinspires.ftc.teamcode.subsystem.intake.intake;
import org.firstinspires.ftc.teamcode.subsystem.localization.localization;
import org.firstinspires.ftc.teamcode.subsystem.outtake.outtake;

@Config
@TeleOp(name = "New OG Two Player Teleop >:)")
public class newerOGTwoPlayerTeleop extends OpMode {

    //Subsystems
    drive drive;
    localization find;
    intake in;
    outtake out;
    boolean rt = false, lt = false, lb = false, rt2 = false, rb = false, x = false, y2 = false;
    boolean hang_switch = false;


    enum OuttakeState{
        DEFAULT,
        INIT,
        HANG_BASKET,
        HANG_CHAMBER,
        DROP_SAMPLE,
        DROP_SPECIMEN,
        WALL,
        GRAB,
        HANG_SPECIMEN_SAFETY
    }

    enum IntakeState{
        DEFAULT,
        PICK_EXTEND,
        PICK_RETRACT,
        SPIT_EXTEND,
        SPIT_RETRACT,
        FUNNEL,
        MANUAL_OVERRIDE
    }
    enum SimpleIntakeState{
       DEFAULT, RETRACT
    }


    enum ArmServo{
        DOWN,
        FLAT,
        UP
    }

    enum newOuttakeState{
        
    }

    OuttakeState outstate = OuttakeState.INIT;
    ArmServo armstate = ArmServo.DOWN;
    IntakeState instate = IntakeState.DEFAULT;
    SimpleIntakeState simpstate = SimpleIntakeState.DEFAULT;

    //Constants
    constants consts;

    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new drive(hardwareMap);
        find = new localization(hardwareMap);
        consts = new constants();
        out = new outtake(hardwareMap);
        in = new intake(hardwareMap);
        find.configureOtos(telemetry);
    }

    @Override
    public void loop(){
        updateDrive();
        if(gamepad1.left_bumper && !lb){
            find.resetHeading();
        }
        //updateIntake();
        manualIntake();
        updateOuttake();
        updateSwitches();
        getTelemetry();
        telemetry.update();
    }

    public void manualIntake(){
        switch (simpstate) {
            case DEFAULT:
            if (Math.abs(gamepad2.left_stick_y) <= 0.1) {
                in.setMotorPowers(0);
            }
            else {
                in.setMotorPowers(gamepad2.left_stick_y);
            }
            if (gamepad2.right_trigger >= 0.3 && !rt2) {
                in.switchState();
            }
            in.setServos();
            in.setRollServo(gamepad2.right_stick_y);
            if(gamepad2.y && !y2){
                in.setMotorState(DcMotor.RunMode.RUN_TO_POSITION);
                simpstate = SimpleIntakeState.RETRACT;
            }
            break;
            case RETRACT:
                in.setSlidePosition(0);
                in.setMotorPowers(0.8);
                in.setState(intake.SERVO_STATE.UP);
                in.setServos();
                if((0.5*(in.left_slide.getCurrentPosition()+in.right_slide.getCurrentPosition()))<=5){
                    in.setMotorState(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    in.setState(intake.SERVO_STATE.IN);
                    in.setServos();
                    simpstate = SimpleIntakeState.DEFAULT;
                }
        }
    }

    public void updateIntake(){
        switch (instate){
            case DEFAULT:
                in.setState(intake.SERVO_STATE.IN);
                in.setRollServo(-gamepad2.right_stick_y);
                if(gamepad2.x){
                    instate = IntakeState.FUNNEL;
                }
                if(gamepad2.a){
                    instate = IntakeState.PICK_EXTEND;
                }
                if(gamepad2.b){
                    instate = IntakeState.SPIT_EXTEND;
                }
                break;
            case FUNNEL:
                in.setState(intake.SERVO_STATE.IN);
                in.setServos();
                in.setRollServo(1);
                if(gamepad2.right_bumper){
                    instate = IntakeState.DEFAULT;
                }
                break;
            case PICK_EXTEND:
                in.setState(intake.SERVO_STATE.OUT);
                in.setServos();
                in.setRollServo(-1);
                in.runToPosition(900);
                in.setMotorPowers(0.7);
                if(Math.abs(Math.max(in.left_slide.getCurrentPosition(), in.right_slide.getCurrentPosition())-900)<=10 || !gamepad2.a){
                    instate = IntakeState.PICK_RETRACT;
                }
                break;
            case SPIT_EXTEND:
                in.setState(intake.SERVO_STATE.OUT);
                in.setServos();
                in.runToPosition(900);
                in.setMotorPowers(0.7);
                if(Math.abs(Math.max(in.left_slide.getCurrentPosition(), in.right_slide.getCurrentPosition())-900)<=10 || !gamepad2.b){
                    instate = IntakeState.PICK_RETRACT;
                }
                break;
            case PICK_RETRACT:
                in.setState(intake.SERVO_STATE.OUT);
                in.setServos();
                in.setRollServo(-1);
                in.runToPosition(0);
                in.setMotorPowers(0.7);
                if(Math.abs(Math.max(in.left_slide.getCurrentPosition(), in.right_slide.getCurrentPosition())-0)<=10){
                    instate = IntakeState.DEFAULT;
                }
                break;
            case SPIT_RETRACT:
                in.setState(intake.SERVO_STATE.OUT);
                in.setServos();
                in.setRollServo(1);
                in.runToPosition(0);
                in.setMotorPowers(0.7);
                if(Math.abs(Math.max(in.left_slide.getCurrentPosition(), in.right_slide.getCurrentPosition())-0)<=10) {
                    instate = IntakeState.DEFAULT;
                }
                break;
            case MANUAL_OVERRIDE:
                in.setState(intake.SERVO_STATE.IN);
                if(gamepad2.dpad_left){
                    in.switchState();
                }
                in.setRollServo(-gamepad2.right_stick_y);
                in.setMotorPowers(-gamepad2.left_stick_y);
                if(gamepad2.right_bumper){
                    instate = IntakeState.DEFAULT;
                }
                break;
        }
    }

    public void newUpdateIntake(){

    }

    public void updateOuttake(){
        switch (outstate){
            case INIT:
                out.leftMotor.setTargetPosition(0);
                out.rightMotor.setTargetPosition(0);
                out.leftServo.setPosition(0);
                out.rightServo.setPosition(1);
                if(!gamepad1.atRest()){outstate= OuttakeState.DEFAULT;}
                telemetry.addData("State", "INIT");
                break;
            case DEFAULT:
                out.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                out.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                out.leftMotor.setTargetPosition(100);
                out.rightMotor.setTargetPosition(100);
                if(gamepad1.right_trigger >= 0.3 && !rt){
                    out.clawServo.setPosition(1-out.clawServo.getPosition());
                }
                switch(armstate){
                    case UP:
                        out.setArmPosition(1);
                        if(gamepad1.x){
                            armstate = ArmServo.DOWN;
                        }
                        break;
                    case DOWN:
                        out.setArmPosition(0);
                        if(gamepad1.x){
                            armstate = ArmServo.UP;
                        }
                        break;
                    }
                out.setMotorPowers(0.4);
                if(gamepad1.b){
                    outstate = OuttakeState.WALL;
                }
                if(gamepad1.y){
                    out.clawServo.setPosition(1);
                    outstate = OuttakeState.HANG_BASKET;
                }
                if(gamepad1.a){
                    outstate = OuttakeState.GRAB;
                }
                telemetry.addData("State", "DEFAULT");
                break;
            case HANG_BASKET:
                out.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                out.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                out.leftMotor.setTargetPosition(2000);
                out.rightMotor.setTargetPosition(2000);
                out.setMotorPowers(0.4);
                out.setArmPosition(1);
                if(gamepad1.right_trigger >= 0.3 && !rt){
                    out.clawServo.setPosition(1-out.clawServo.getPosition());
                }
                if(gamepad1.right_bumper){
                    outstate = OuttakeState.DROP_SAMPLE;
                }
                telemetry.addData("State", "HANG_BASKET");
                break;
            case HANG_CHAMBER:
                out.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                out.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                out.setLiftPosition(100);
                out.setArmPosition(1);
                out.setMotorPowers(0.4);
                out.clawServo.setPosition(1);
                if(gamepad1.right_bumper && !rb){
                    outstate = OuttakeState.DROP_SPECIMEN;
                }
                telemetry.addData("State", "HANG_CHAMBER");
                break;
            case DROP_SAMPLE:
                out.clawServo.setPosition(0);
                outstate = OuttakeState.DEFAULT;
                telemetry.addData("State", "DROP_SAMPLE");
                break;
            case DROP_SPECIMEN:
                out.setArmPosition(0.75);
                out.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                out.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                out.setLiftPosition(100);
                out.clawServo.setPosition(1);
                out.setMotorPowers(0.4);
                if(gamepad1.right_bumper && !rb){
                    outstate = OuttakeState.HANG_SPECIMEN_SAFETY;
                }

                telemetry.addData("State", "DROP_SPECIMEN");
                break;
            case WALL:
                out.setLiftPosition(0);
                out.rightServo.setPosition(0.35);
                out.leftServo.setPosition(0.65);
                if(gamepad1.right_trigger >= 0.3 && !rt){
                    out.clawServo.setPosition(1-out.clawServo.getPosition());
                }
                out.setMotorPowers(0.4);
                if(gamepad1.x){
                    out.clawServo.setPosition(1);
                    outstate = OuttakeState.HANG_CHAMBER;
                }
                break;
            case GRAB:
                out.setLiftPosition(0);
                out.setArmPosition(0);
                out.setMotorPowers(0.4);
                if(gamepad1.right_trigger >= 0.3 && !rt){
                    out.clawServo.setPosition(1-out.clawServo.getPosition());
                }
                if(gamepad1.right_bumper){
                    out.clawServo.setPosition(1);
                    outstate = OuttakeState.DEFAULT;
                }
                break;
            case HANG_SPECIMEN_SAFETY:
                hang_switch = true;
                out.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                out.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if(gamepad2.dpad_up){out.setMotorPowers(0.3);}
                else if(gamepad2.dpad_down){out.setMotorPowers(-0.3);}
                else out.setMotorPowers(0);

                if(gamepad1.x && !x) {
                    out.clawServo.setPosition(0);
                    outstate = OuttakeState.DEFAULT;
                }
                break;
        }
    }

    public void updateSwitches(){
        lt=(gamepad1.left_bumper);
        rt=(gamepad1.right_trigger>=0.3);
        lb = (gamepad1.left_trigger>=0.3);
        rt2 = (gamepad2.right_trigger >= 0.3);
        rb = gamepad1.right_bumper;
        x = gamepad1.x;
        y2 = gamepad2.y;
    }

    public void updateDrive(){
        double forward =0;
        double strafe =0;
        double turn =0;
        if(gamepad1.left_stick_x*gamepad1.left_stick_x+gamepad1.left_stick_y*gamepad1.left_stick_y >= consts.gp1dead*consts.gp1dead || Math.abs(gamepad1.right_stick_x) >= consts.gp1dead) {
            forward = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            drive.FieldCentricDrive(forward, strafe, turn, find.myOtos);
        }
        else if(gamepad1.dpad_up) {drive.driveBot(0.3,0,0,1);}
        else if(gamepad1.dpad_down) {drive.driveBot(-0.3,0,0,1);}
        else if(gamepad1.dpad_left) {drive.driveBot(0,0,-0.3,1);}
        else if(gamepad1.dpad_right) {drive.driveBot(0,0,0.3,1);}
        else drive.stopBot();
    }

    public void getTelemetry(){

    }

    public boolean inputSwitch(boolean cond, boolean lastCond){
        return cond == lastCond && cond;
    }
}
