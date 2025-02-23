package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "Turning")
public class turning_test extends OpMode{

    //Poses
    private final Pose startPose = new Pose(0, 0, 0);

    private final Pose midPose = new Pose(20, 20, Math.toRadians(90));

    private final Pose endPose = new Pose(0, 0, 0);

    private PathChain auto1, auto2;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
    }


    //Build Paths
    public void buildPaths(){
        auto1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(midPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), midPose.getHeading())
                .build();

        auto2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(midPose), new Point(endPose)))
                .setLinearHeadingInterpolation(midPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(auto1, true);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(auto2, true);
                    setPathState(-1);
                }
                break;
        }

    }



}
