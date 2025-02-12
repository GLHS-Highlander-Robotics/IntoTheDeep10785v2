package org.firstinspires.ftc.teamcode.subsystem.localization;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.constants.constants;

public class localization_SparkFunOTOS {
    //Components
    public SparkFunOTOS myOtos;
    constants consts;

    //Variables

    //Constructor
    public localization_SparkFunOTOS(HardwareMap hardwareMap) {

        consts = new constants();
        myOtos = hardwareMap.get(SparkFunOTOS.class, consts.otos_hm);

    }

    public void configureOtos(Telemetry telemetry) {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements.
        // Can be either meters or inches for linear, and radians or degrees for angular.
        // Units default to inches and degrees.
        // Setting is not persisted in the sensor, so needs to set at the start of all OpModes if using the non-default value.
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // If sensor is mounted to a robot without being centered, offset can be specified for the sensor to center it.
        // Different units can be used by specifying them before setting offset.
        // As of firmware version 1.0, these values will be lost after a power cycle, so they must be set every time power is set back on.
        // For example, if the sensor is mounted 5 inches to the left (negative X) and 10 inches forward (positive Y) of the center of the robot, and mounted 90 degrees clockwise (negative rotation) from the robot's orientation, the offset would be {-5, 10, -90}.
        // These can be any value, even the angle can be tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for scaling issues with the sensor measurements.
        // Note that as of firmware version 1.0, these values will be lost after a power cycle, so you will need to set them each time you power up the sensor.
        // They can be any value from 0.872 to 1.127 in increments of 0.001 (0.1%).
        // It is recommended to first set both scalars to 1.0, then calibrate the angular scalar, then the linear scalar.
        // To calibrate the angular scalar, spin the robot by multiple rotations (eg. 10) to get a precise error, then set the scalar to the inverse of the error.
        // Remember that the angle wraps from -180 to 180 degrees.
        // For example, if after 10 rotations counterclockwise (positive rotation), the sensor reports -15 degrees, the required scalar would be 3600/3585 = 1.004.
        // To calibrate the linear scalar, move the robot a known distance and measure the error; do this multiple times at multiple speeds to get an average, then set the linear scalar to the inverse of the error.
        // For example, if you move the robot 100 inches and the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(consts.linearOTOSconst);
        myOtos.setAngularScalar(consts.angularOTOSconst);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could have an offset.
        // Note that as of firmware version 1.0, the calibration will be lost after a power cycle; the OTOS performs a quick calibration when it powers up, but it is recommended to perform a more thorough calibration at the start of all your OpModes.
        // Note that the sensor must be completely stationary and flat during calibration! When calling calibrateImu(), you can specify the number of samples to take and whether to wait until the calibration is complete.
        // If no parameters are provided, it will take 255 samples and wait until done; each sample takes about 2.4ms, so about 612ms total.
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin, but can also be used to recover from certain rare tracking errors
        myOtos.resetTracking();

        // After resetting tracking, OTOS will report that robot is at origin.
        // If robot does not start at origin, or another source of location information (eg. vision odometry) is available, the OTOS location can be set to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Gets hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
    public SparkFunOTOS.Pose2D findPosition(){
        return myOtos.getPosition();
    }
}
