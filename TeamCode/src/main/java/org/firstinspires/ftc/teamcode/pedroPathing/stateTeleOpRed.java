package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
//import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;

@Config
@TeleOp(name = "state TeleOp Red", group = "linear equations test")
public class stateTeleOpRed extends LinearOpMode {

    Servo hood, turret1, turret2, PTO1, PTO2;
    DcMotorEx f1, f2, gecko;
    DcMotor intake, frontLeft, frontRight, backLeft, backRight;
    GoBildaPinpointDriver pip;

    public static double farSlope =  1750 ;
    @Override
    public void runOpMode() {
        // Hardware Mapping
        hood = hardwareMap.get(Servo.class, "hood");
        turret1 = hardwareMap.get(Servo.class, "turret");
        turret2 = hardwareMap.get(Servo.class, "turret2");


        f1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        f2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        intake = hardwareMap.get(DcMotor.class, "intake");
        gecko = hardwareMap.get(DcMotorEx.class, "gecko");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class,  "backRight ");
        backLeft = hardwareMap.get(DcMotor.class,   "backLeft  ");
        frontLeft = hardwareMap.get(DcMotor.class,  "frontLeft ");



        // Directions
        frontLeft.setDirection(DcMotorSimple.Direction .FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction  .FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction .REVERSE);

        f1.setDirection(DcMotorSimple.Direction.REVERSE);
        f2.setDirection(DcMotorSimple.Direction.REVERSE);
        hood.setDirection(Servo.Direction.REVERSE);
        turret1.setDirection(Servo.Direction.REVERSE);

        // Pinpoint Config
        pip.setOffsets(-42, -90, DistanceUnit.MM);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pip.resetPosAndIMU();

        double tx = -72; // Target X
        double ty = 72;  // Target Y
        double var = 0;
        double AIM_OFFSET_DEG_LOCAL = 1;
        double visionOffsetDeg = 0.0; // Mocked for this test file since there's no Limelight mapped
        double axial;
        double lateral;
        double yawCmd;
        boolean emergencyTogglePressed = false;
        boolean emergencyStopActive = false;
        f1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        f2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
        FtcDashboard dashboard = FtcDashboard.getInstance();



        //Pose2d savedPose = PoseStorage.currentPose;
        // 3. FORCE THE HARDWARE TO THE CORRECT SPOT
        // This overwrites whatever "wrong" numbers the hardware has
        //if (savedPose.position.x == 0 && savedPose.position.y == 0 && savedPose.heading.toDouble() == 0) {
        //    // Case: We didn't run Auto (or it crashed), so assume start of match
        //    // You might want to change this to your known start pose if testing teleop alone
        //    // odo.resetPosAndIMU();
        //} else {
        //    // Case: We came from Auto! Load the data.
        //    pip.resetPosAndIMU();
//
        //}


        double samOffset = 5;
        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;

        boolean aa = gamepad2.a;
        boolean bb = gamepad2.b;
        boolean prevaa = aa;
        boolean prevbb = bb;

        waitForStart();
        //pip.setPosition(new Pose2D(
        //        DistanceUnit.INCH,
        //        savedPose.position.x,
        //        savedPose.position.y,
        //        AngleUnit.RADIANS,
        //        savedPose.heading.toDouble()
        //));
        telemetry.addLine(">>> AUTO POSITION LOADED <<<");
        while (opModeIsActive()) {

            pip.update(); // leftdate position first!

            double robotX = pip.getPosX(DistanceUnit.INCH);
            double robotY = pip.getPosY(DistanceUnit.INCH);

            double xl = tx - robotX;
            double yl = ty - robotY;
            double hypot = Math.sqrt((xl * xl) + (yl * yl));

            // =========================================================================
            //  TURRET APPLICATION (EXACTLY LIKE OLDER CODE)
            // =========================================================================
            double angleToGoal = Math.atan2(yl, xl);
            double robotHeading = pip.getHeading(AngleUnit.RADIANS);

            // Base Turret Angle Calculation
            double calculatedTurretRad = angleToGoal - robotHeading;

            // 1. Wrap angle (-180 to 180)
            while (calculatedTurretRad > Math.PI) calculatedTurretRad -= 2 * Math.PI;
            while (calculatedTurretRad < -Math.PI) calculatedTurretRad += 2 * Math.PI;

            // 2. Convert to Degrees and apply center offset (159 deg from comment)
            double finalServoDegrees = Math.toDegrees(calculatedTurretRad) + 151.5;

            // 4. Combine Physics + Vision
             left = gamepad2.dpad_left;
             right = gamepad2.dpad_right;

            aa = gamepad2.a;
            bb = gamepad2.b;
            
            
            if (left && !prevleft && !right) {
                samOffset = Range.clip(samOffset + 2.5, -40, 40);
            }
            if (right && !prevright && !left) {
                samOffset = Range.clip(samOffset - 2.5, -40, 40);
            }

            // leftdate prev AFTER using them
            prevleft = left;
            prevright = right;

            /*if (aa && !prevaa && !bb) {
                samOffset = Range.clip(samOffset + 2.5, -40, 40);
            }
            if (bb && !prevbb && !aa) {
                samOffset = Range.clip(samOffset - 2.5, -40, 40);
            }

            // leftdate prev AFTER using them
            prevaa = aa;
            prevbb = bb;*/
            
            finalServoDegrees += visionOffsetDeg + samOffset;

            // 5. Clamp and Set
            finalServoDegrees = Range.clip(finalServoDegrees, 0, 303);
            if (gamepad1.ps || gamepad2.ps) {
                turret1.setPosition(.5);
                turret2.setPosition(.5);
            } else {
                turret1.setPosition(finalServoDegrees / 303);
                turret2.setPosition(finalServoDegrees / 303);
            }

            // =========================================================================
            //  HOOD LINEAR REGRESSION
            // =========================================================================
            double hpos;
            if (hypot < 130) {
                // Zone: Close
                double d1 = 57.5, d2 = 97.3;
                double v1 = 0.5, v2 = 0.7;
                double slope = (v2 - v1) / (d2 - d1);
                hpos = v1 + (slope * (hypot - d1));
            } else {
                // Zone: Far
                double d1 = 136.5, d2 = 158.1;
                double v1 = 0.7, v2 = 1.0;
                double slope = (v2 - v1) / (d2 - d1);
                hpos = v1 + (slope * (hypot - d1));
            }
            hood.setPosition(Range.clip(hpos, 0.5, 1.0));

            // =========================================================================
            //  FLYWHEEL LINEAR REGRESSION
            // =========================================================================
            double vTarget;
            if (hypot < 130) {
                // Zone: Close
                double d1 = 57.5, d2 = 97.3;
                double v1 = 1310, v2 = 1660;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = 1150 + (slope * (hypot - d1));
            } else {
                // Zone: Far
                double d1 = 136.5, d2 = 158.1;
                double v1 = 1900, v2 = 1940;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = farSlope + (slope * (hypot - d1));
            }
            vTarget = Range.clip(vTarget, 0, 2500); // Adjust max based on motor


            if (gamepad1.x) {
                var = 1;
            }
            else if (gamepad1.y) {
                var = 0;
            }

            if (var == 1) {
                f1.setVelocity(vTarget);
                f2.setVelocity(vTarget);
            } else {
                f1.setVelocity(0);
                f2.setVelocity(0);
            }










            // =========================================================================
            //  INTAKE LOGIC
            // =========================================================================

            double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakeCmd = 1.0;
            }
            else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intakeCmd = -1.0;
            }

            intake.setPower(intakeCmd);

            if (intakeCmd != 0) {
                gecko.setPower(gamepad1.left_bumper || gamepad1.right_bumper ? -1.0 : 1.0);
            } else {
                gecko.setPower(0);
            }





            // =========================================================================
            //  DRIVE LOGIC
            // =========================================================================
            /*if (gamepad1.a) {
                if (!emergencyTogglePressed) {
                    emergencyStopActive = !emergencyStopActive;
                    emergencyTogglePressed = true;
                }
            } else {
                emergencyTogglePressed = false;
            }
            if (emergencyStopActive) {
                PTO1.setPosition(.9);
                PTO2.setPosition(.9);
                lateral = 0;
                continue;
            }  else {
                lateral = -gamepad1.left_stick_x;
            }
            if (opModeIsActive()) {
                PTO1.setPosition(0);
                PTO2.setPosition(0);
            }*/
             axial   = gamepad1.left_stick_y;
             lateral = -gamepad1.left_stick_x;
             yawCmd  = -gamepad1.right_stick_x;

            double fl = axial + lateral + yawCmd;
            double fr = axial - lateral - yawCmd;
            double bl = axial - lateral + yawCmd;
            double br = axial + lateral - yawCmd;

            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            frontLeft.setPower(fl / max);
            frontRight.setPower(fr / max);
            backLeft.setPower(bl / max);
            backRight.setPower(br / max);

            // =========================================================================
            //  ODOMETRY RESET (EXACTLY LIKE OLDER CODE)
            // =========================================================================
            if (gamepad1.dpad_up) {
                pip.setPosition(new Pose2D(DistanceUnit.INCH, -68.02, 28.72, AngleUnit.DEGREES, 91.2));
                samOffset = 0;
            } else if (gamepad1.dpad_down) {
                pip.resetPosAndIMU();
                samOffset = 0;
            } else if (gamepad1.dpad_left){
                pip.setPosition(new Pose2D(DistanceUnit.INCH, 62, -65, AngleUnit.DEGREES, 0));
                samOffset = 0;
            }

            // =========================================================================
            //  TELEMETRY
            // =========================================================================

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("farSlope", farSlope);

            dashboard.sendTelemetryPacket(packet);


            telemetry.addData("Distance (Hypot)", hypot);
            telemetry.addData("Target Velocity", vTarget);
            telemetry.addData("Hood Position", hpos);
            telemetry.addData("Turret Angle (Deg)", finalServoDegrees);
            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.update();
        }
    }
}