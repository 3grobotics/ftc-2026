package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class newintakeSub {
    private final DcMotorEx intake, indexer;
    private final Servo swingArm, gate;


    // commanded powers (set by runnables, applied in loop)
    private double intakeCmd = 0.0;
    private double indexerCmd  = 0.0;

    public boolean loopActive = false;

    public newintakeSub(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        indexer  = hardwareMap.get(DcMotorEx.class, "indexer");
        swingArm = hardwareMap.get(Servo.class, "swingArm");
        gate = hardwareMap.get(Servo.class, "gate");

    }

    public void loop() {
        // apply commanded power every loop
        intake.setPower(intakeCmd);
        indexer.setPower(indexerCmd);

        if (loopActive) {
            //ig do loop stuff in here
        }
    }

    public Runnable IntakeInindexerOut() {
        return () -> {
            loopActive = false;
            intakeCmd = 1;
            indexerCmd  = 1;
        };
    }

    public Runnable intakeOutindexerIn() {
        return () -> {
            loopActive = false;
            intakeCmd = -1;
            indexerCmd  = -1;
        };
    }

    public Runnable intakeInindexerIn() {
        return () -> {
            loopActive = false;
            intakeCmd = 1;
            indexerCmd  = -1;
            swingArm.setPosition(.5);
            gate.setPosition(.65);
        };
    }

    public Runnable intakeOutindexerOut() {
        return () -> {
            loopActive = false;
            intakeCmd = -1;
            indexerCmd  = 1;
        };
    }

    public Runnable intakeAndindexerStop() {
        return () -> {
            loopActive = false;
            intakeCmd = 0;
            indexerCmd  = 0;
            gate.setPosition(1);
        };
    }
}
