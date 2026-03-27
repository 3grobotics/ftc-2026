package org.firstinspires.ftc.teamcode.pedroPathing.utility;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;

@TeleOp(name="the opmode for tuning linear equations", group="linear equations test")
//@Disabled
public class tuninglinearequations extends LinearOpMode {
    public hardwareSubNewBot h;
    public varSub var;
    public Timer swingTimer;
    Servo t, t2, hood;
    DcMotorEx f1, f2;

    @Override
    public void runOpMode() {
        h = new hardwareSubNewBot(hardwareMap);
        var = new varSub();
        swingTimer = new Timer();

        hood  = hardwareMap.get(Servo.class, "hood");
        t  = hardwareMap.get(Servo.class, "turret");
        t2 = hardwareMap.get(Servo.class, "turret2");
        f1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        f2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        hood.setDirection(Servo.Direction.REVERSE);
        f1.setDirection(DcMotorEx.Direction.REVERSE);
        f2.setDirection(DcMotorEx.Direction.REVERSE);

        double sp = 0.5;
        double v  = 0;

        boolean prevxx = false;
        boolean prevbb = false;
        boolean prevaa = false;
        boolean prevyy = false;

        double target_y = -72;
        double target_x = 72;
        boolean swingHigh = false;

        GoBildaPinpointDriver.Register[] defaultRegisters = {
                GoBildaPinpointDriver.Register.DEVICE_STATUS,
                GoBildaPinpointDriver.Register.LOOP_TIME,
                GoBildaPinpointDriver.Register.X_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.Y_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.X_POSITION,
                GoBildaPinpointDriver.Register.Y_POSITION,
                GoBildaPinpointDriver.Register.H_ORIENTATION,
                GoBildaPinpointDriver.Register.X_VELOCITY,
                GoBildaPinpointDriver.Register.Y_VELOCITY,
                GoBildaPinpointDriver.Register.H_VELOCITY,
        };

        GoBildaPinpointDriver pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        DcMotor intake     = hardwareMap.get(DcMotor.class, "intake");
        DcMotor indexer    = hardwareMap.get(DcMotor.class, "indexer");
        Servo gate        = hardwareMap.get(Servo.class, "gate");
        Servo swingArm    = hardwareMap.get(Servo.class, "swingArm");



        pip.setOffsets(48.591, -129.909, DistanceUnit.MM);
        pip.setBulkReadScope(defaultRegisters);
        pip.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pip.resetPosAndIMU();
        //cc issue? VV
        hood.setDirection(Servo.Direction.REVERSE);
        t.setDirection(Servo.Direction.REVERSE);
        t2.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            t.setPosition(0.5);

            /*hood*/ {
                boolean xx = gamepad1.x;
                boolean bb = gamepad1.b;

                if (xx && !prevxx && !bb) {
                    sp = Range.clip(sp - 0.1, 0, .6);
                }
                if (bb && !prevbb && !xx) {
                    sp = Range.clip(sp + 0.1, 0, .6);
                }

                // update prev AFTER using them
                prevxx = xx;
                prevbb = bb;

                hood.setPosition(sp);
            }

            /*flywheel*/ {
                boolean aa = gamepad1.a;
                boolean yy = gamepad1.y;

                if (aa && !prevaa && !yy) {
                    v = Range.clip(v + 50, -7000, 7000.0);
                }
                if (yy && !prevyy && !aa) {
                    v = Range.clip(v - 50, -7000, 7000.0);
                }

                // update prev AFTER using them
                prevaa = aa;
                prevyy = yy;

                // apply velocity after updating v so it changes immediately
                f1.setVelocity(v);
                f2.setVelocity(v);
            }
            /*intake */{
                double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
            /*if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakeCmd = 1.0;
                h.sickle.setPosition(0.9);
            }
            else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intakeCmd = -1.0;

            }*/



                intake.setPower(-intakeCmd);
                indexer.setPower(-intakeCmd);

                if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                    swingArm.setPosition(.5);
                    gate.setPosition(.65);
                }
        }


                if (gamepad1.right_bumper) {
                    h.gate.setPosition(1);
                    h.indexer.setPower(-1);
                    h.intake.setPower(-1);

                    if (swingTimer.getElapsedTime() >= 100) {
                        swingHigh = !swingHigh;
                        swingTimer.resetTimer();
                    }

                    if (swingHigh) {
                        h.swingArm.setPosition(1);
                    } else {
                        h.swingArm.setPosition(.7);
                    }

                } else if (!gamepad1.right_bumper){
                    swingHigh = false;
                    swingTimer.resetTimer();
                }

        pip.update();

            double robotX = -pip.getPosX(DistanceUnit.INCH);
            double robotY = pip.getPosY(DistanceUnit.INCH);

            double xl = target_x - pip.getPosX(DistanceUnit.INCH);
            double yl = target_y - pip.getPosY(DistanceUnit.INCH);
            double hypot = Math.sqrt((xl * xl) + (yl * yl));

            telemetry.addData("x leg", xl);
            telemetry.addData("y leg", yl);
            telemetry.addData("hypot", hypot);
            telemetry.addData("servo pos", sp);
            telemetry.addData("vels", v);
            telemetry.addData("RPM flywheel 1", "%.3f", ((f1.getVelocity() * 60) / 37.333));
            telemetry.addData("RPM flywheel 2", "%.3f", ((f2.getVelocity() * 60) / 37.333));
            telemetry.addData("vel ticks flywheel 1", "%.3f", (f1.getVelocity()));
            telemetry.addData("vel ticks flywheel 2", "%.3f", (f2.getVelocity()));
            telemetry.addData("pip x in", pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip y in", pip.getPosY(DistanceUnit.INCH));
            telemetry.update();
        }
    }

}
