package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystem.intake.intake;
import org.firstinspires.ftc.teamcode.subsystem.outtake.outtake;

@Config
@Autonomous(name="Working Sample Auto")
public class newer_sample_auto extends OpMode{

    intake in;

    outtake out;

    Follower follower;

    Timer pathTimer;

    ElapsedTime waitTimer;

    int pathState = 0;

    //Poses
    private final Pose start_pos = new Pose(9, 112, 0);

    private final Pose ctrl12_1 = new Pose(37.462, 117.853, 0);

    private final Pose basket = new Pose(15, 127, Math.toRadians(-45));

    private final Pose ctrl23_1 = new Pose(9.234, 127.597, 0);

    private final Pose right_sample = new Pose(35.581, 128, 0);

    private final Pose mid_sample = new Pose(35.993, 138, 0);

    private final Pose left_sample = new Pose(30.170, 133.762, Math.toRadians(45));
    private PathChain preload, pickUpClose, placeClose, pickUpMid, placeMid, pickUpFar, placeFar;

    private void buildPaths(){
        preload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(start_pos), new Point(ctrl12_1), new Point(basket)))
                .setLinearHeadingInterpolation(start_pos.getHeading(), basket.getHeading())
                .build();

        pickUpClose = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(basket), new Point(ctrl23_1), new Point(right_sample)))
                .setLinearHeadingInterpolation(basket.getHeading(), right_sample.getHeading())
                .build();

        placeClose = follower.pathBuilder()
                .addPath(new BezierLine(new Point(right_sample), new Point(basket)))
                .setLinearHeadingInterpolation(right_sample.getHeading(),basket.getHeading())
                .build();

        pickUpMid = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basket), new Point(mid_sample)))
                .setLinearHeadingInterpolation(basket.getHeading(),mid_sample.getHeading())
                .build();

        placeMid = follower.pathBuilder()
                .addPath(new BezierLine(new Point(mid_sample), new Point(basket)))
                .setLinearHeadingInterpolation(mid_sample.getHeading(),basket.getHeading())
                .build();

        pickUpFar = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basket), new Point(left_sample)))
                .setLinearHeadingInterpolation(basket.getHeading(), left_sample.getHeading())
                .build();

        placeFar = follower.pathBuilder()
                .addPath(new BezierLine(new Point(left_sample), new Point(basket)))
                .setLinearHeadingInterpolation(left_sample.getHeading(), basket.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                out.setLiftPosition(2000);
                out.setArmPosition(0.9);
                follower.followPath(preload, true);
                setPathState(1);
                out.clawServo.setPosition(1);
                in.setState(intake.SERVO_STATE.IN);
                in.setServos();
                break;

            case 1: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    if(waitTimer.milliseconds()<=1500) {
                        follower.setMaxPower(0.6);
                        out.clawServo.setPosition(0);
                        in.setState(intake.SERVO_STATE.OUT);
                        in.setServos();
                    }
                    else if(waitTimer.milliseconds()>=1500 && waitTimer.milliseconds()<=2000){
                        out.clawServo.setPosition(1);
                    }
                    else{
                        follower.followPath(pickUpClose, true);
                        in.setRollServo(1);
                        setPathState(2);
                    }
                }
                else{
                    waitTimer.reset();
                }
                break;

            case 2: // Wait until the robot is near the first sample pickup position
                if (!follower.isBusy()) {
                    if(waitTimer.milliseconds()>=50 && waitTimer.milliseconds()<=2500){
                        in.setState(intake.SERVO_STATE.IN);
                        in.setServos();
                        out.setLiftPosition(0);
                        out.setArmPosition(0);
                    }
                    else if(waitTimer.milliseconds()>2500 && waitTimer.milliseconds()<=3500){
                        in.setRollServo(-1);
                        out.clawServo.setPosition(0);
                    }
                    else if(waitTimer.milliseconds()>3500 && waitTimer.milliseconds()<4000){
                        out.clawServo.setPosition(1);
                    }
                    else if(waitTimer.milliseconds()>=4000) {
                        out.setLiftPosition(2000);
                        out.setArmPosition(1);
                        follower.followPath(placeClose, true);
                        setPathState(3);
                    }
                }
                else{
                    waitTimer.reset();

                }
                break;

            case 3: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    if(waitTimer.milliseconds()>=1500) {
                        out.setLiftPosition(0);
                        out.setArmPosition(0);
                        follower.followPath(pickUpMid, true);
                        setPathState(4);
                    }
                }
                else waitTimer.reset();
                break;

            case 4: // Wait until the robot is near the second sample pickup position
                if (!follower.isBusy()) {
                    if(waitTimer.milliseconds()>=1500) {
                        out.setLiftPosition(2000);
                        out.setArmPosition(1);
                        follower.followPath(placeMid, true);
                        setPathState(5);
                    }
                }
                else waitTimer.reset();
                break;

            case 5: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    if(waitTimer.milliseconds()>=1500) {
                        out.setLiftPosition(0);
                        out.setArmPosition(0);
                        follower.followPath(pickUpFar, true);
                        setPathState(6);
                    }
                }
                else waitTimer.reset();
                break;

            case 6: // Wait until the robot is near the third sample pickup position
                if (!follower.isBusy()) {
                    if(waitTimer.milliseconds()>=1500) {
                        out.setLiftPosition(2000);
                        out.setArmPosition(1);
                        follower.followPath(placeFar, true);
                        setPathState(7);
                    }
                }
                else waitTimer.reset();
                break;

            case 7: // Wait until the robot is near the parking position
                if (!follower.isBusy()) {
                    if(waitTimer.milliseconds()>=1500) {
                        out.setLiftPosition(0);
                        out.setArmPosition(0);
                        setPathState(-1); // End the autonomous routine
                    }
                }
                else waitTimer.reset();
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        in = new intake(hardwareMap);
        out = new outtake(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(start_pos);
        buildPaths();
        pathState = 0;
        out.setLiftPosition(0);
        out.setMotorState(DcMotor.RunMode.RUN_TO_POSITION);
        waitTimer = new ElapsedTime();
        out.clawServo.setPosition(1);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        out.setMotorPowers(0.85);
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.addData("Timer", waitTimer.milliseconds());
        telemetry.update();
    }


}
