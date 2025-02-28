package org.firstinspires.ftc.teamcode.subsystem.sensor;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.constants.constants;

public class color_sensor {

    public ColorRangeSensor c_Sensor1;
    private constants consts;

    public color_sensor(HardwareMap hardwareMap) {
        consts = new constants();

        c_Sensor1 = hardwareMap.get(ColorRangeSensor.class, consts.cSensor_1_hm);
    }

    public double[] getColorValues(ColorRangeSensor c_sensor){
        int redVal = c_sensor.red();
        int greenVal = c_sensor.green();
        int blueVal = c_sensor.blue();
        double[] colorArray = {redVal, greenVal, blueVal};
        return colorArray;
    }
}
