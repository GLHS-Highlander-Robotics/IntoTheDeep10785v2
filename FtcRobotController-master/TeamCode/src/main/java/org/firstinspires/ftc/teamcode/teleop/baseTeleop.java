package teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystem.numbers.PID;

@Config
@TeleOp (name = "v2 Teleop")
public class baseTeleop extends OpMode{
    //Initialize Variables
    //Pedro Pathing Follower
    private Follower follower;
    private final Pose startPose= new Pose(0, 0,0);

    //Outtake Lift PID
    private DcMotor liftMotor;
    public PID liftPID = new PID(0.1,0,0);

    //

    @Override
    public void init(){
        // Initialize Follower
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        //Initialize hardwareMap
        liftMotor=hardwareMap.get(DcMotor.class, "lift");
    }

    @Override
    public void start(){
        follower.startTeleopDrive();
    }

    @Override
    public void loop(){
        //Information

        //Drive
        //drive();
        //Intake
        //Outtake
        outtake();
    }

    public void drive(){

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();
    }

    public void outtake(){
        //Outtake Components:
        //Slide Motor x2
        //Slide Servo x1
        //Slide Claw x1

        
    }
}
