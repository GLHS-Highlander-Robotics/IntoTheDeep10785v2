package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drive.drive;
import org.firstinspires.ftc.teamcode.subsystem.localization.localization;
import org.firstinspires.ftc.teamcode.subsystem.intake.intake;

@Config
@TeleOp(name = "OG Two Player Teleop v2 >:)")
public class OldDriveTwoPlayerTeleop_v2 extends OpMode {

    //Subsystems
    drive drive;
    localization find;
    intake intake;

    @Override
    public void init(){

    }

    @Override
    public void loop(){

    }
}
