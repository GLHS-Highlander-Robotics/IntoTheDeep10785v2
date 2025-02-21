package org.firstinspires.ftc.teamcode.laboratory;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.subsystem.constants.constants;
import org.firstinspires.ftc.teamcode.subsystem.localization.localization_SparkFunOTOS;
import org.firstinspires.ftc.teamcode.subsystem.sensor.distance_sensor;

@Config
@TeleOp(name = "Distance Sensor lab", group = "Laboratory")
public class distanceSensor_lab extends OpMode {

    constants consts;
    distance_sensor sensor;

    public double distance1;
    public double distance2;

    @Override
    public void init() {
        sensor = new distance_sensor(hardwareMap);
        consts = new constants();
    }

    @Override
    public void loop(){
        findDistances();
        getTelemetry();
        telemetry.update();
    }

    public void findDistances(){
        distance1 = sensor.getDistance(sensor.d_Sensor1);
        distance2 = sensor.getDistance(sensor.d_Sensor2);
    }

    public void getTelemetry(){
        telemetry.addData("distance 1", distance1);
        telemetry.addData("distance 2", distance2);
    }

}
