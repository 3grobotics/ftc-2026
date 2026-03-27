package org.firstinspires.ftc.teamcode.pedroPathing.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp(name="the opmode for testing linear equations", group="linear equations test")
//@Disabled
public class testinglinearequations extends LinearOpMode {

    Servo h;
    Servo t;
    DcMotorEx f1;
    DcMotorEx f2;

    @Override
    public void runOpMode() {
        h   = hardwareMap.get(Servo.class,     "hood");
        t   = hardwareMap.get(Servo.class,     "turret");
        f1  = hardwareMap.get(DcMotorEx.class, "flywheel1");
        f2  = hardwareMap.get(DcMotorEx.class, "flywheel2");

        f1.setDirection(DcMotorEx.Direction.REVERSE);
        f2.setDirection(DcMotorEx.Direction.REVERSE);

        double v = 0;

        double hypot;
        double hposClose = 0;
        double hposFar   = 0;

        // Flywheel linear eq outputs (velocity units = motor ticks/sec for setVelocity)
        double vClose = 0;
        double vFar   = 0;

        double ty = 72;
        double tx = -72;

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
        DcMotorEx gecko = hardwareMap.get(DcMotorEx.class, "gecko");

        pip.setOffsets(-42, -90, DistanceUnit.MM);

        pip.setBulkReadScope(defaultRegisters);
        pip.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        t.setDirection(Servo.Direction.REVERSE);
        pip.resetPosAndIMU();
        waitForStart();

        while (opModeIsActive()) {
            t.setPosition(.5);

            double xl = tx - pip.getPosX(DistanceUnit.INCH);
            double yl = ty - pip.getPosY(DistanceUnit.INCH);
            hypot = Math.sqrt((xl * xl) + (yl * yl));

            /* hood */ {
                // compute first (so we don't use stale values / first-loop 0)
                hposClose = 0.0054099  * hypot + 0.226097;
                hposFar   = 0.00399002 * hypot + 0.345731;

                hposClose = Range.clip(hposClose, .5, 1.0);
                hposFar   = Range.clip(hposFar,   .5, 1.0);




                if (hypot < 100) {
                    h.setPosition(hposClose);
                } else {
                    h.setPosition(hposFar);
                }


            }

            /* flywheel (NO STEPPING; distance-based) */ {
                // Two-region linear equations (match your hood structure)
                // Replace these with your tuned flywheel equations:


                double CfarD   = 125.9;
                double CcloseD = 56.82;

                double distDiffC = CfarD - CcloseD;
                // ^ 69.08

                double CfarV   = 1773.33;
                double CcloseV = 1353.33;

                double velDiffC = CfarV - CcloseV;
                // ^ 420

                double CstepV = velDiffC / distDiffC;
                // ^ 6.079907353792704

                vClose =  CstepV * hypot;
                vFar   = 12.8014 * hypot + 219.62;

                if (hypot < 130) {
                    v = vClose;
                } else {
                    v = vFar;
                }

                v = Range.clip(v, 0.0, 7000.0);
                f1.setVelocity(v);
                f2.setVelocity(v);
            }

            /*intake */{
                double rt = gamepad1.right_trigger + gamepad2.right_trigger;
                double lt = gamepad1.left_trigger + gamepad2.left_trigger;
                double intakeCmd = rt - lt;

                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    intakeCmd = 1.0;
                } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    intakeCmd = -1.0;
                }

                intakeCmd = Range.clip(intakeCmd, -1, 1);

                if (intakeCmd > 0 && intakeCmd <= 1) {
                    if (gamepad1.left_bumper) {
                        gecko.setPower(-1);
                    } else {
                        gecko.setPower(1);
                    }
                } else if (intakeCmd < 0 && intakeCmd >= -1) {
                    if (gamepad1.right_bumper) {
                        gecko.setPower(1);
                    } else {
                        gecko.setPower(-1);
                    }
                } else {
                    gecko.setPower(0);
                }

                intake.setPower(intakeCmd);
            }
            pip.update();

            telemetry.addData("x leg", xl);
            telemetry.addData("y leg", yl);
            telemetry.addData("hypot", hypot);
            telemetry.addData("hood close", hposClose);
            telemetry.addData("hood far", hposFar);
            telemetry.addData("vel cmd", v);
            telemetry.addData("RPM flywheel 1", "%.3f", ((f1.getVelocity() * 60) / 28));
            telemetry.addData("RPM flywheel 2", "%.3f", ((f2.getVelocity() * 60) / 28));
            telemetry.addData("pip x in", pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip y in", pip.getPosY(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}