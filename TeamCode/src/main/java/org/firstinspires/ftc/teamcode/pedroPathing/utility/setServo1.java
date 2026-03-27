
package org.firstinspires.ftc.teamcode.pedroPathing.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="set servo 1", group="Linear OpMode")
//@Disabled
public class setServo1 extends LinearOpMode {

    ServoImplEx h, t1, t2;

    @Override
    public void runOpMode() {

        t1 = hardwareMap.get(ServoImplEx.class,"turret");
        t2 = hardwareMap.get(ServoImplEx.class,"turret2");
        h = hardwareMap.get( ServoImplEx.class,"hood");
        h.setDirection(Servo.Direction.REVERSE);
        t1.setDirection(Servo.Direction.REVERSE);
        t2.setDirection(Servo.Direction.REVERSE);



        waitForStart();

        while (opModeIsActive()) {
            t1.setPwmRange(new PwmControl.PwmRange(550, 2450));
            t2.setPwmRange(new PwmControl.PwmRange(550, 2450));

            t1.setPosition(1.0);
            t2.setPosition(1.0);
            h.setPosition( 1.0);
            telemetry.addData("hood position", h.getPosition());
            telemetry.update();
        }
    }
}