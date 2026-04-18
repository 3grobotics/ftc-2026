package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.ColorCamera2;

/**
 * Enhanced arm subsystem that now provides real‑time height estimation based on
 * • arm rotation (potentiometer)
 * • slide extension (encoder)
 * <p>
 * expectedHeight = sin(getRotDeg · π / 180) · (ARM_BASE_LEN + getExtIn) + EXP_HEIGHT_OFFSET
 */
public class turretSub {
    public final ServoImplEx turret1, turret2;
    public boolean loopActive = false;
    public hardwareSubNewBot h;
    public varSub v;
    boolean left, right, prevleft, prevright, aa, bb;
    double target;
    double robotX;
    double robotY;

    double xl;
    double yl;
    double hypot;

    double angleToGoal;
    double robotHeading;

    double targetTurretRad;


    double baseServoDegrees;
    double finalServoDegrees;

    public turretSub(HardwareMap hardwareMap) {
      
        turret1 = hardwareMap.get(ServoImplEx.class, "turret");
        turret2 = hardwareMap.get(ServoImplEx.class, "turret2");
        turret1.setDirection(Servo.Direction.FORWARD);
        turret2.setDirection(Servo.Direction.FORWARD);
        turret1.setPwmRange(new PwmControl.PwmRange(550, 2450));
        turret2.setPwmRange(new PwmControl.PwmRange(550, 2450));
    }

    /* ───────────────────────────────────────────────────────────────────────
     * Control loop for the main arm (call inside OpMode loop)               */
    public void loop() {
        if (!loopActive) return;

        h.pip.update();

        robotX           = h.pip.getPosX(DistanceUnit.INCH);
        robotY           = h.pip.getPosY(DistanceUnit.INCH);

        xl               = v.tx - robotX;
        yl               = v.ty - robotY;
        hypot            = Math.sqrt((xl * xl) + (yl * yl));


        angleToGoal      = Math.atan2(yl, xl);
        robotHeading     = h.pip.getHeading(AngleUnit.RADIANS);


        targetTurretRad  = angleToGoal - robotHeading + Math.PI;


        while (targetTurretRad > Math.PI) targetTurretRad -= 2 * Math.PI;
        while (targetTurretRad < -Math.PI) targetTurretRad += 2 * Math.PI;


        baseServoDegrees = Math.toDegrees(targetTurretRad) - (314.6112145 / 2.0);
        finalServoDegrees = baseServoDegrees + v.visionOffsetDeg + ColorCamera2.samOffset;


        h.turret1.setPosition(Math.abs((target) / 314.6112145));
        h.turret2.setPosition(Math.abs((target) / 314.6112145));
    }

    

    public void runTurret() {
        loopActive = true;
        target = finalServoDegrees;
    }

    public void setStraight() {
        loopActive = false;
        target = 0;
    }


}