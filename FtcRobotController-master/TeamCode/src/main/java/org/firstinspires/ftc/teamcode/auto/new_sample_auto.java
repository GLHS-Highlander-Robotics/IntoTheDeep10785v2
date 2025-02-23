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

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;

@Config
@Autonomous(name = "New Sample Autonomous")
public class new_sample_auto extends OpMode{

    //Poses
    private final Pose start_pos = new Pose(9, 112, 0);

    private final Pose ctrl12_1 = new Pose(37.462, 112.853, 0);

    private final Pose basket = new Pose(14.290, 130.369, -45);

    private final Pose ctrl23_1 = new Pose(9.234, 119.597, 0);

    private final Pose close_sample = new Pose(41.581, 118.502, 0);

    private final Pose mid_sample = new Pose(40.993, 130.035, 0);

    private final Pose far_sample = new Pose(42.170, 133.762, 45);



    private final Pose[] itinerary = {start_pos, basket, close_sample, basket, mid_sample, basket, far_sample, basket};

    private PathChain auto1, auto2, auto3, auto4, auto5, auto6, auto7;

    private ArrayList<PathChain> PathList;

    private double[] times = {3,3,3,3,3,3,3,3,3,3};

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(start_pos);
        buildPaths();
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate(times);
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
    }


    public ArrayList<PathChain> improvedbuildPathsFromPoseList(Pose[] list){
        ArrayList<PathChain> autoChain = new ArrayList<PathChain>();
        for(int i = 0; i < list.length-1; i++){
            PathChain addedPath = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(list[i]),new Point(list[i+1])))
                    .setLinearHeadingInterpolation(list[i].getHeading(), list[i+1].getHeading())
                    .build();
            autoChain.add(addedPath);
        }
        return autoChain;
    }

    //Build Paths
    public void buildPaths(){
        auto1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(start_pos), new Point(ctrl12_1), new Point(basket)))
                .setLinearHeadingInterpolation(start_pos.getHeading(), basket.getHeading())
                .build();

        auto2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(basket), new Point(ctrl23_1), new Point(close_sample)))
                .setLinearHeadingInterpolation(basket.getHeading(), close_sample.getHeading())
                .build();

        auto3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(close_sample), new Point(basket)))
                .setLinearHeadingInterpolation(close_sample.getHeading(), basket.getHeading())
                .build();

        auto4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basket), new Point(mid_sample)))
                .setLinearHeadingInterpolation(basket.getHeading(), mid_sample.getHeading())
                .build();

        auto5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(mid_sample), new Point(basket)))
                .setLinearHeadingInterpolation(mid_sample.getHeading(), basket.getHeading())
                .build();

        auto6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basket), new Point(far_sample)))
                .setLinearHeadingInterpolation(basket.getHeading(), far_sample.getHeading())
                .build();

        auto7 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(far_sample), new Point(basket)))
                .setLinearHeadingInterpolation(far_sample.getHeading(), basket.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate(double[] timelimits) {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(auto1, true);
                if(pathTimer.getElapsedTimeSeconds() >= timelimits[0]) {
                    setPathState(1);
                }
                break;

            case 1: // Wait until the robot is near the scoring position
                follower.followPath(auto2, true);
                if(pathTimer.getElapsedTimeSeconds() >= timelimits[1]) {
                    setPathState(2);
                }
                break;

            case 2: // Wait until the robot is near the scoring position
                follower.followPath(auto3, true);
                if(pathTimer.getElapsedTimeSeconds() >= timelimits[2]) {
                    setPathState(-1);
                }
                break;
            case 3: // Move from start to scoring position
                follower.followPath(auto4, true);
                if(pathTimer.getElapsedTimeSeconds() >= timelimits[3]) {
                    setPathState(4);
                }
                break;

            case 4: // Wait until the robot is near the scoring position
                follower.followPath(auto5, true);
                if(pathTimer.getElapsedTimeSeconds() >= timelimits[4]) {
                    setPathState(5);
                }
                break;

            case 5: // Wait until the robot is near the scoring position
                follower.followPath(auto6, true);
                if(pathTimer.getElapsedTimeSeconds() >= timelimits[5]) {
                    setPathState(6);
                }
                break;
            case 6: // Wait until the robot is near the scoring position
                follower.followPath(auto7, true);
                if(pathTimer.getElapsedTimeSeconds() >= timelimits[6]) {
                    setPathState(-1);
                }
                break;
        }

    }


}
