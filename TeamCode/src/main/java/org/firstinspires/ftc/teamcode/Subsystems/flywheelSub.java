package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class flywheelSub {
    private final DcMotorEx flywheel1, flywheel2;

    // commanded powers (set by runnables, applied in loop)
    private double flywheel1Vel = 0.0;
    private double flywheel2Vel  = 0.0;

    public boolean loopActive = false;

    public flywheelSub(HardwareMap hardwareMap) {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));

    }

    public void loop() {
        // apply commanded power every loop
        flywheel1.setVelocity(flywheel1Vel);
        flywheel2.setVelocity(flywheel2Vel);

        if (loopActive) {
            //ig do loop stuff in here
        }
    }

    public Runnable autoFlywheelFar() {
        return () -> {
            loopActive = false;
            flywheel1Vel = 1800;
            flywheel2Vel  = 1800;
        };
    }

    public Runnable autoFlywheelClose() {
        return () -> {
            loopActive = false;
            flywheel1Vel = (double) (2800 * 28) / 60;
            flywheel2Vel  = (double) (2800 * 28) / 60;
        };
    }

}