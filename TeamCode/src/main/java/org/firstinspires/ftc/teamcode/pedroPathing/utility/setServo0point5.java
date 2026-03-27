
package org.firstinspires.ftc.teamcode.pedroPathing.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="set servo 0.5", group="Linear OpMode")
//@Disabled
public class setServo0point5 extends LinearOpMode {

    Servo h, t1, t2;


    @Override
    public void runOpMode() {

        t1 = hardwareMap.get(Servo.class,"turret");
        t2 = hardwareMap.get(Servo.class,"turret2");
        h = hardwareMap.get(Servo.class,"hood");
        h.setDirection(Servo.Direction.REVERSE);
        t1.setDirection(Servo.Direction.REVERSE);
        t2.setDirection(Servo.Direction.REVERSE);



        waitForStart();

        while (opModeIsActive()) {
        t1.setPosition(0.5);
        t2.setPosition(0.5);
        h.setPosition(0.5);
        telemetry.addData("hood position", h.getPosition());
        telemetry.update();
        }
    }
}