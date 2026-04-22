package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// For samOffset
import org.firstinspires.ftc.teamcode.pedroPathing.ColorCamera2testing;
import org.firstinspires.ftc.teamcode.pedroPathing.ColorCamera2testingoldWithTread;

public class turretSub {
    public final ServoImplEx turret1, turret2;
    public boolean loopActive = false;

    // IMPORTANT: h and v must be assigned from the main OpMode thread
    // before the turret thread starts and runs loop().
    public hardwareSubNewBot h;
    public varSub v;

    // --- Thread-Safe Telemetry Variables ---
    public volatile double currentFly1Vel = 0;
    public volatile double currentFly2Vel = 0;
    public volatile double currentTurretCommand = 0;

    // Original variables
    double robotX;
    double robotY;
    double xl;
    double yl;
    double hypot;

    double vx;
    double vy;
    double vTarget                   = 0;
    double tx;
    double ty;
    double t;
    double hoodTarget;
    double flywheelTarget;

    double hpos;
    double velocityreal;
    double velDiff;

    double angleToGoal;
    double robotHeading;

    double targetTurretRad;

    double baseServoDegrees;
    volatile double target;
    volatile double finalServoDegrees;

    public turretSub(HardwareMap hardwareMap) {
        turret1 = hardwareMap.get(ServoImplEx.class, "turret");
        turret2 = hardwareMap.get(ServoImplEx.class, "turret2");
        turret1.setDirection(Servo.Direction.FORWARD);
        turret2.setDirection(Servo.Direction.FORWARD);
        turret1.setPwmRange(new PwmControl.PwmRange(550, 2450));
        turret2.setPwmRange(new PwmControl.PwmRange(550, 2450));
        h = new hardwareSubNewBot(hardwareMap);
        v = new varSub();
    }

    public void loop() {
        if (!loopActive) return;

        // The data in h.pip is safely updated by the main thread.
        vy                         = h.pip.getVelY(DistanceUnit.INCH);
        vx                         = h.pip.getVelX(DistanceUnit.INCH);

        tx                         = -72 - (vx * t);
        ty                         =  72 - (vy * t);

        robotX                     = h.pip.getPosX(DistanceUnit.INCH);
        robotY                     = h.pip.getPosY(DistanceUnit.INCH);

        xl                         = tx - robotX;
        yl                         = ty - robotY;
        hypot                      = Math.sqrt((xl * xl) + (yl * yl));

        if ( hypot < 89){
            t                      =  0.004 * hypot + 0.468;
        } else if ( hypot > 89 && hypot < 121)  {
            t                      =  0.833;
        } else if ( hypot > 121) {
            t                      = -0.004 * hypot + 1.317;
        }

        angleToGoal       = Math.atan2(yl, xl);
        robotHeading      = h.pip.getHeading(AngleUnit.RADIANS);

        targetTurretRad   = angleToGoal - robotHeading + Math.PI;

        while (targetTurretRad > Math.PI) targetTurretRad -= 2 * Math.PI;
        while (targetTurretRad < -Math.PI) targetTurretRad += 2 * Math.PI;

        baseServoDegrees  = Math.toDegrees(targetTurretRad) - (314.6112145 / 2.0);
        finalServoDegrees = baseServoDegrees + v.visionOffsetDeg + ColorCamera2testing.samOffset;

        // Calculate and save to volatile variable for main thread telemetry
        currentTurretCommand = Math.abs((target) / 314.6112145);
        h.turret1.setPosition(currentTurretCommand);
        h.turret2.setPosition(currentTurretCommand);

        if (hypot < 75) {
            vTarget = 9.938 * hypot + 1022.646;
        } else if(hypot > 75 && hypot < 96.3){
            vTarget = 12.066 * hypot + 863.05;
        } else if(hypot > 96.3 && hypot < 129) {
            vTarget = 9.817 * hypot + 1079.623;
        } else if(hypot > 151) {
            vTarget = 11.727 * hypot + 833.217;
        }

        vTarget = Range.clip(vTarget, 0, 6000);

        if (ColorCamera2testingoldWithTread.var == 1) {
            h.flywheel1.setVelocity(((flywheelTarget) * 37.33) / 60);
            h.flywheel2.setVelocity(((flywheelTarget) * 37.33) / 60);
        } else {
            h.flywheel1.setVelocity(0);
            h.flywheel2.setVelocity(0);
        }

        //  HOOD LINEAR REGRESSION
        if (hypot < 75) {
            hpos = -0.006 * hypot + 1.053;
        } else if(hypot > 75 && hypot < 96.3){
            hpos = -0.005 * hypot + 0.975;
        } else if(hypot > 96.3 && hypot < 129){
            hpos = -0.015 * hypot + 1.944;
        } else if(hypot > 151){
            hpos = 0;
        }

        // Read hardware ONCE into volatile variables for the main thread to use
        currentFly1Vel = h.flywheel1.getVelocity();
        currentFly2Vel = h.flywheel2.getVelocity();

        velocityreal = (currentFly1Vel + currentFly2Vel) / 2;
        velDiff      = vTarget - velocityreal;

        hpos = Range.clip(hpos, 0, .7);
        h.hood.setPosition(hoodTarget);
    }

    public void runTurret() {
        loopActive        = true;
        target            = finalServoDegrees;
        flywheelTarget    = vTarget;
        hoodTarget        = hpos;
    }

    public void setStraight() {
        loopActive        = false;
        target            = 0;
        flywheelTarget    = 0;
        hoodTarget        = 0;
    }
}