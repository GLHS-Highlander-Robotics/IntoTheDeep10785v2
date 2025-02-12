package org.firstinspires.ftc.teamcode.subsystem.constants;

public class constants {
    //HardwareMap Definitions
    public String intake_roller_hm, intake_right_hm, intake_left_hm, otos_hm,  intake_rightslide_hm, intake_leftslide_hm, outtake_rightslide_hm, outtake_leftslide_hm, outtake_claw_hm, outtake_right_hm, outtake_left_hm;
    public String fl_hm, fr_hm, bl_hm, br_hm;
    public double linearOTOSconst, angularOTOSconst;
    public double REV_TicksPerRevolution, GoBilda_TicksPerRevolution;
    public double iP, iD; // Input PD
    public double oP, oD; // Output PD
    public double hP, hI, hD; // Heading PID
    public double dP, dD, dF; // Drive PID
    public double sP, sD, sF; // Strafe PDF
    public double gp1dead; // Gamepad 1 Deadzones
    public double gp2dead; // Gamepad 2 Deadzones
    public boolean useFieldCentric; // Controls if using field or robot centric drive

    public double dpadSpeed;
    public int outMotorTolerances;
    public double currentDrawThreshold;

    public constants(){

        intake_left_hm = "intake_lt"; //PIN S5-CH

        intake_right_hm= "intake_rt"; //PIN S5-EH

        intake_roller_hm = "intake_roll"; //PIN S0-EH

        otos_hm = "myOTOS"; //PIN I2C0-CH

        linearOTOSconst = 1.0771;

        angularOTOSconst = 1.0005;

        fl_hm = "front_left"; //PIN M0-CH

        fr_hm = "front_right"; // PIN M1-CH

        bl_hm = "back_left"; //PIN M2-CH

        br_hm = "back_right"; //PIN M3-CH

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

        gp1dead = 0.3;

        useFieldCentric = false;

        dpadSpeed = 0.3;

        outMotorTolerances = 3;

        currentDrawThreshold = 1000;
    }
}
