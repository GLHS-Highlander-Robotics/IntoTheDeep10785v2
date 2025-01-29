package laboratory;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp (name = "Intake Lab", group="Laboratory")
public class intake extends OpMode{
    //Initialize Variables
    private Servo intake_left, intake_right, intake_roller;
    private boolean rt;
    constants constants;


    @Override
    public void init(){
        // Initialize Servo
        intake_roller=hardwareMap.get(Servo.class, constants.intake_roller_hm);
        intake_left=hardwareMap.get(Servo.class, constants.intake_left_hm);
        intake_right=hardwareMap.get(Servo.class, constants.intake_right_hm);

        intake_right.setPosition(0);
        intake_right.setPosition(1);
    }


    @Override
    public void loop(){
        if(gamepad1.right_trigger>0.3&&!rt){
            intake_right.setPosition(1-intake_right.getPosition());
            intake_left.setPosition(1-intake_left.getPosition());
            rt=true;
        }
        else rt=false;
        if(Math.abs(gamepad1.right_stick_y)>=0.3){
            intake_roller.setPosition(Math.signum(gamepad1.right_stick_y)*0.5+0.5);
        }    }

}
