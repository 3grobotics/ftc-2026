package org.firstinspires.ftc.teamcode.Subsystems;

public class varSub {


    public double tx = 72; // Target X
    public double ty = -72;  // Target Y
    public double var = 0;
    public double AIM_OFFSET_DEG_LOCAL = 1;
    public double visionOffsetDeg = 0.0; // Mocked for this test file since there's no Limelight mapped
    public double axial;
    public double lateral;
    public double yawCmd;
    public boolean emergencyTogglePressed = false;
    public boolean emergencyStopActive = false;
    public double samOffset = 5;
    public boolean swingHigh = false;

    public varSub() {

    }

}