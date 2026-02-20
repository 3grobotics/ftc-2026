package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * @author Gage Ross
 * this is the turret subsystem. it controls the turret throught all other opmodes that use the turret.
 */
public class turretSub {

    
    public double target = 1;// volts –> target arm angle via pot
    public double target2 = 1;
    private final Servo turret1, turret2;

    public boolean loopActive = false;

    public turretSub(HardwareMap hardwareMap) {
        turret1 = hardwareMap.get(Servo.class, "turret");
        turret2 = hardwareMap.get(Servo.class, "turret2");
    }

    /* ───────────────────────────────────────────────────────────────────────
     * Control loop for the main arm (call inside OpMode loop)               */
    public void loop() {
        if (!loopActive) return;
        turret1.setDirection(Servo.Direction.REVERSE);
        
        turret1.setPosition(target); 
        turret2.setPosition(target2);
    }


    public void turretPointFive() {
        loopActive = true;
        target = .5;
        target2 = .5;
    }


    }