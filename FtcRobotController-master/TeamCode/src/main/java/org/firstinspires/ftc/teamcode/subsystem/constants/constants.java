package org.firstinspires.ftc.teamcode.subsystem.constants;

public class constants {
    //HardwareMap Definitions
    public String intake_roller_hm, intake_right_hm, intake_left_hm, otos_hm, fl_hm, fr_hm, bl_hm, br_hm, intake_rightslide_hm, intake_leftslide_hm, outtake_rightslide_hm, outtake_leftslide_hm, outtake_claw_hm, outtake_right_hm, outtake_left_hm;
    public double linearOTOSconst, angularOTOSconst;
    public double REV_TicksPerRevolution, GoBilda_TicksPerRevolution;
    public double iP, iD; // Input PD
    public double oP, oD; // Output PD
    public double hP, hI, hD; // Heading PID
    public double dP, dD, dF; // Drive PID
    public double sP, sD, sF; // Strafe PDF

    public constants(){

        intake_left_hm = "intake_lt"; //PIN S5-CH

        intake_right_hm= "intake_rt"; //PIN S5-EH

        intake_roller_hm = "intake_roll"; //PIN S0-EH

        otos_hm = "my_otos"; //PIN I2C0-CH

        linearOTOSconst = 1;

        angularOTOSconst = 1;

        fl_hm = "front_left";

        fr_hm = "front_right";

        bl_hm = "back_left";

        br_hm = "back_right";

        intake_leftslide_hm = "intake_lslide";

        intake_rightslide_hm = "intake_rslide";

        outtake_leftslide_hm = "outtake_lslide";

        outtake_rightslide_hm = "outtake_rslide";

        outtake_right_hm = "outtake_rt";

        outtake_left_hm = "outtake_lt";

        outtake_claw_hm = "outtake_claw";

        REV_TicksPerRevolution = 288;

        iP = 0;

        iD = 0;

        oP = 0;

        oD = 0;

    }
}
