package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.drive.drive;
import org.firstinspires.ftc.teamcode.subsystem.localization.localization;

@TeleOp(name = "Field Centric", group = "Teleop")
public class FCTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        drive drive;
        localization find;

        // Sets up settings for telemetry
        // Sets the display order for telemetry, in this case, has newest entries displayed first
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        // Sets the amount of lines that are displayed by the driver hub
        telemetry.log().setCapacity(8);

        waitForStart();

        drive = new drive(hardwareMap);
        find = new localization(hardwareMap);

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Uses joysticks to control x-y-z axis
            // blank_RSy currently unused
            double drive_LSy = gamepad1.left_stick_y;
            double strafe_LSx = gamepad1.left_stick_x;
            double blank_RSy = gamepad1.right_stick_y;
            double rotate_RSx = gamepad1.right_stick_x;

            // Displays data from joysticks
            telemetry.addLine("left joystick | ")
                    .addData("x", gamepad1.left_stick_x)
                    .addData("y", gamepad1.left_stick_y);
            telemetry.addLine("right joystick | ")
                    .addData("x", gamepad1.right_stick_x)
                    .addData("y", gamepad1.right_stick_y);

            // Precision Mode, uses dpad to control x / y axis
            // When dpad pressed, sets to 30% power or 0.3/1
            if (gamepad1.dpad_up) {
                drive_LSy = -0.3;
            }
            if (gamepad1.dpad_down) {
                drive_LSy = 0.3;
            }
            if (gamepad1.dpad_left) {
                strafe_LSx = -0.3;
            }
            if (gamepad1.dpad_right) {
                strafe_LSx = 0.3;
            }

            //Displays data from dpad
            telemetry.addLine("dpad | ")
                    .addData("up", gamepad1.dpad_up)
                    .addData("down", gamepad1.dpad_down)
                    .addData("left", gamepad1.dpad_left)
                    .addData("right", gamepad1.dpad_right);

            // Uses x button to reset heading to 0
            // readjusts orientation parameters to current robot orientation
            if (gamepad1.x) {
                find.imu.resetYaw();
            }

            // Gets current bot heading based on angle in RADIANS from field orientation
            double botHeading = find.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Displays current bot heading
            telemetry.addData("Bot heading", botHeading);

            // Rotate the movement direction counter to the bot's rotation
            // Used to interpret
            // To calculate y rotation,
            // To calculate x rotation,
            double rotY = strafe_LSx * Math.sin(-botHeading) - drive_LSy * Math.cos(-botHeading);
            double rotX = strafe_LSx * Math.cos(-botHeading) + drive_LSy * Math.sin(-botHeading);


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate_RSx), 1);
            double frontLeftPower = (rotY + rotX - rotate_RSx) / denominator;
            double backLeftPower = (rotY - rotX - rotate_RSx) / denominator;
            double frontRightPower = (rotY - rotX + rotate_RSx) / denominator;
            double backRightPower = (rotY + rotX + rotate_RSx) / denominator;

            // Displays data of motor powers
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backRightPower", backRightPower);

            // Updates telemetry with prior data
            telemetry.update();
        }
    }
}