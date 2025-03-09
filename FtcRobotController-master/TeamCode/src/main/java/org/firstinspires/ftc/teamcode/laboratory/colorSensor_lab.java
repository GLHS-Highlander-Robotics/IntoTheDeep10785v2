package org.firstinspires.ftc.teamcode.laboratory;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.subsystem.constants.constants;
import org.firstinspires.ftc.teamcode.subsystem.sensor.color_sensor;

@Config
@TeleOp(name = "Color Sensor lab", group = "Laboratory")
public class colorSensor_lab extends OpMode{

    constants consts;
    color_sensor sensor;

    public double[] colorArray;

    @Override
    public void init() {
        sensor = new color_sensor(hardwareMap);
        consts = new constants();
    }
    @Override
    public void loop(){
        findColor();
        getTelemetry();
        telemetry.update();
    }
    public void findColor(){
        colorArray = sensor.getColorValues(sensor.c_Sensor1);
    }

    public void getTelemetry(){
        telemetry.addData("Red Value", colorArray[0]);
        telemetry.addData("Green Value", colorArray[1]);
        telemetry.addData("Blue Value", colorArray[2]);

    }
}
