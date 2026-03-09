
package org.firstinspires.ftc.teamcode.pedroPathing.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="set servo 0", group="Linear OpMode")
//@Disabled
public class setServo0 extends LinearOpMode {

    Servo h, t1, t2;

    @Override
    public void runOpMode() {

        t1 = hardwareMap.get(Servo.class,"turret");
        t2 = hardwareMap.get(Servo.class,"turret2");
        h = hardwareMap.get(Servo.class,"hood");
        h.setDirection(Servo.Direction.REVERSE);
        t1.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
        t1.setPosition(0.0);
        t2.setPosition(0.0);
        h.setPosition(0.0);
            telemetry.addData("hood position", h.getPosition());
            telemetry.update();
        }
    }
}