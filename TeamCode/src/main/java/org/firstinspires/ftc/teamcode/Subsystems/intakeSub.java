package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intakeSub {
    private final DcMotorEx intake, gecko;

    // commanded powers (set by runnables, applied in loop)
    private double intakeCmd = 0.0;
    private double geckoCmd  = 0.0;

    public boolean loopActive = false;

    public intakeSub(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        gecko  = hardwareMap.get(DcMotorEx.class, "gecko");
    }

    public void loop() {
        // apply commanded power every loop
        intake.setPower(intakeCmd);
        gecko.setPower(geckoCmd);

        if (loopActive) {
            //ig do loop stuff in here
        }
    }

    public Runnable IntakeInGeckoOut() {
        return () -> {
            loopActive = false;
            intakeCmd = 1;
            geckoCmd  = 1;
        };
    }

    public Runnable intakeOutGeckoIn() {
        return () -> {
            loopActive = false;
            intakeCmd = -1;
            geckoCmd  = -1;
        };
    }

    public Runnable intakeInGeckoIn() {
        return () -> {
            loopActive = false;
            intakeCmd = 1;
            geckoCmd  = -1;
        };
    }

    public Runnable intakeOutGeckoOut() {
        return () -> {
            loopActive = false;
            intakeCmd = -1;
            geckoCmd  = 1;
        };
    }

    public Runnable intakeAndGeckoStop() {
        return () -> {
            loopActive = false;
            intakeCmd = 0;
            geckoCmd  = 0;
        };
    }
}