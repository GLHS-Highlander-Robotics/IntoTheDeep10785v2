package org.firstinspires.ftc.teamcode.subsystem.sensor;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.constants.constants;

public class distance_sensor {
    public DistanceSensor d_Sensor1, d_Sensor2;
    private constants consts;

    public distance_sensor(HardwareMap hardwareMap) {
        consts = new constants();

        //d_Sensor1 = hardwareMap.get(DistanceSensor.class, consts.dSensor1_hm);
        //d_Sensor2 = hardwareMap.get(DistanceSensor.class, consts.dSensor2_hm);
    }

    public double getDistance(DistanceSensor d_sensor){
        return d_sensor.getDistance(DistanceUnit.INCH);

    }



}
