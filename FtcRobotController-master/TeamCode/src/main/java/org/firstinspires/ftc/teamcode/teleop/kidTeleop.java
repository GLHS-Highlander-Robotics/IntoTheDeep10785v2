package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.constants.constants;
import org.firstinspires.ftc.teamcode.subsystem.drive.drive;
import org.firstinspires.ftc.teamcode.subsystem.intake.intake;
import org.firstinspires.ftc.teamcode.subsystem.localization.localization;
import org.firstinspires.ftc.teamcode.subsystem.outtake.outtake;

@Config
@TeleOp(name = "OG Two Player Teleop >:)")
public class kidTeleop extends OpMode {

    //Subsystems
    drive drive;
    localization find;
    intake in;
    outtake out;
    boolean rt = false, lt = false, lb = false, rt2 = false, rb = false, x = false, a = false, y = false;
    boolean hang_switch = false;

    enum OuttakeState{
       DOWN,
       UP
    }

    enum IntakeState{
       IN,
       OUT,
       FUNNEL
    }

    enum ArmServo{
        DOWN,
        FLAT,
        UP
    }
    OuttakeState outstate = OuttakeState.DOWN;
    IntakeState instate = IntakeState.IN;
    ElapsedTime randomTimer;

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
        randomTimer = new ElapsedTime();
        in.setSlidePosition(0);
        in.setMotorState(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop(){
        updateGamepads();
        updateDrive();
        updateIntake();
        //manualIntake();
        //updateOuttake();
        updateSwitches();
        getTelemetry();
        telemetry.update();
    }

    public void updateGamepads(){
    }
    
    public void manualIntake(){
        double power = gamepad2.left_stick_y;
        if(Math.abs(power) <= 0.1){power = 0;}
        in.setMotorPowers(power);
        if(gamepad2.right_trigger >= 0.3 && !rt2){
            in.switchState();
        }
        in.setServos();
        in.setRollServo(gamepad2.right_stick_y);
    }

    public void updateIntake(){
        switch (instate){
            case IN:
                in.setMotorPowers(0.5);
                in.setSlidePosition(0);
                in.setRollServo(0);
                in.setState(intake.SERVO_STATE.UP);
                in.setServos();
                if(gamepad1.y && !y){
                    in.setState(intake.SERVO_STATE.OUT);
                }
                if((0.5*(in.left_slide.getCurrentPosition()+in.right_slide.getCurrentPosition()))<=7){
                    in.setState(intake.SERVO_STATE.IN);
                    in.setServos();
                    if(gamepad1.x && !x){
                        instate = IntakeState.FUNNEL;
                    }
                }
                break;
            case OUT:
                in.setMotorPowers(0.5);
                in.setSlidePosition(900);
                in.setState(intake.SERVO_STATE.OUT);
                in.setServos();
                if(gamepad1.y && !y){
                    instate = IntakeState.IN;
                }
                break;
            case FUNNEL:
                in.setMotorPowers(0);
                in.setRollServo(-1);
                if(gamepad1.x && !x) {instate = IntakeState.IN;}
        }
    }

    public void updateOuttake(){
        out.setMotorPowers(0.5);
        switch(outstate){
           case DOWN:
               out.setLiftPosition(0);
               out.setArmPosition(0);
               if(gamepad1.right_trigger >= 0.3 && !rt){
                   out.clawServo.setPosition(1-out.clawServo.getPosition());
               }
               if(gamepad1.a && !a){
                  out.clawServo.setPosition(0);
                  outstate = OuttakeState.UP;
               }
               break;
           case UP:
               out.setLiftPosition(1000);
               out.setArmPosition(1);
               if(gamepad1.right_trigger >= 0.3 && !rt){
                   out.clawServo.setPosition(1-out.clawServo.getPosition());
               }
               if(gamepad1.a && !a){
                   out.clawServo.setPosition(0);
                   outstate = OuttakeState.DOWN;
               }
               break;
       }
    }

    public void updateSwitches(){
        a = gamepad1.a;
        x = gamepad1.b;
        y = gamepad1.y;
        rt = gamepad1.right_trigger >= 0.3;
    }

    public void updateDrive(){
        double forward =0;
        double strafe =0;
        double turn =0;
        if(gamepad1.left_stick_x*gamepad1.left_stick_x+gamepad1.left_stick_y*gamepad1.left_stick_y >= consts.gp1dead*consts.gp1dead || Math.abs(gamepad1.right_stick_x) >= consts.gp1dead) {
            forward = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            drive.FieldCentricDrive(forward, strafe, turn, find.myOtos, 0.4);
        }
        else if(gamepad1.dpad_up) {drive.driveBot(0.1,0,0,1);}
        else if(gamepad1.dpad_down) {drive.driveBot(-0.1,0,0,1);}
        else if(gamepad1.dpad_left) {drive.driveBot(0,0,-0.1,1);}
        else if(gamepad1.dpad_right) {drive.driveBot(0,0,0.1,1);}
        else drive.stopBot();
    }

    public void getTelemetry(){

    }

    public boolean inputSwitch(boolean cond, boolean lastCond){
        return cond == lastCond && cond;
    }
}
