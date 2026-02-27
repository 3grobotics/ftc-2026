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
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
//import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;

@Config
@TeleOp(name = "state TeleOp Red", group = "linear equations test")
public class stateTeleOpRed extends LinearOpMode {

    Servo hood, turret1, turret2, PTO1, PTO2;
    DcMotorEx f1, f2, gecko;
    DcMotor intake, frontLeft, frontRight, backLeft, backRight;
    GoBildaPinpointDriver pip;

    public static double farSlope = 1750;
    public static double BALL_VELOCITY = 200.0;
    public static double LEAD_MULTIPLIER = 1.0;
    public static double HEADING_LEAD_GAIN = .3;
    public static double LATENCY_SEC = 0.05;
    public static double VEL_DEADZONE = 0.25;
    public static double HEAD_DEADZONE = 0.02;

    // Pedro Pathing Max Velocities
    public static double MAX_AXIAL_VEL = 91.05;
    public static double MAX_LATERAL_VEL = 65.68;
    public static double FEEDFORWARD_GAIN = 0.6;

    // NEW: Low-Pass Filter Alpha (0.0 = Infinite Smoothing, 1.0 = Raw Jitter)
    public static double HEADING_FILTER_ALPHA = 0.2;

    @Override
    public void runOpMode() {
        hood = hardwareMap.get(Servo.class, "hood");
        turret1 = hardwareMap.get(Servo.class, "turret");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        f1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        f2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        intake = hardwareMap.get(DcMotor.class, "intake");
        gecko = hardwareMap.get(DcMotorEx.class, "gecko");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight ");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft ");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft ");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        f1.setDirection(DcMotorSimple.Direction.REVERSE);
        f2.setDirection(DcMotorSimple.Direction.REVERSE);
        hood.setDirection(Servo.Direction.REVERSE);
        turret1.setDirection(Servo.Direction.REVERSE);

        pip.setOffsets(-42, -90, DistanceUnit.MM);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pip.resetPosAndIMU();

        double tx = -72;
        double ty = 72;
        double var = 0;
        double AIM_OFFSET_DEG_LOCAL = 1;
        double visionOffsetDeg = 0.0;
        boolean emergencyTogglePressed = false;
        boolean emergencyStopActive = false;
        f1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        f2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
        FtcDashboard dashboard = FtcDashboard.getInstance();

        pip.resetPosAndIMU();

        double samOffset = 5;
        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;

        boolean aa = gamepad2.a;
        boolean bb = gamepad2.b;
        boolean prevaa = aa;
        boolean prevbb = bb;

        // Persists across loops to calculate the moving average
        double filteredVHeading = 0.0;

        waitForStart();
        telemetry.addLine(">>> AUTO POSITION LOADED <<<");
        while (opModeIsActive()) {

            pip.update();

            // 1. Get Joystick Intent Early
            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yawCmd = -gamepad1.right_stick_x;

            // 2. Get Measured State from Pinpoint
            double robotX = pip.getPosX(DistanceUnit.INCH);
            double robotY = pip.getPosY(DistanceUnit.INCH);
            double robotHeading = pip.getHeading(AngleUnit.RADIANS);

            double measuredVx = pip.getVelX(DistanceUnit.INCH);
            double measuredVy = pip.getVelY(DistanceUnit.INCH);
            double rawVHeading = pip.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

            // Apply Low-Pass Filter to smooth out drivetrain vibrations
            filteredVHeading = (HEADING_FILTER_ALPHA * rawVHeading) + ((1.0 - HEADING_FILTER_ALPHA) * filteredVHeading);

            if (Math.abs(measuredVx) < VEL_DEADZONE) {
                measuredVx = 0;
            }
            if (Math.abs(measuredVy) < VEL_DEADZONE) {
                measuredVy = 0;
            }
            if (Math.abs(filteredVHeading) < HEAD_DEADZONE) {
                filteredVHeading = 0;
            }

            // 3. Calculate "Intended" Velocity (Robot-Centric) Using Pedro Pathing Constants
            double intentVxRobot = lateral * MAX_LATERAL_VEL;
            double intentVyRobot = axial * MAX_AXIAL_VEL;

            // 4. Rotate Intent to Field-Centric
            double intentVxField = (intentVxRobot * Math.cos(robotHeading)) - (intentVyRobot * Math.sin(robotHeading));
            double intentVyField = (intentVxRobot * Math.sin(robotHeading)) + (intentVyRobot * Math.cos(robotHeading));

            // 5. KINEMATIC FEEDFORWARD BLENDING
            double vx = (measuredVx * (1.0 - FEEDFORWARD_GAIN)) + (intentVxField * FEEDFORWARD_GAIN);
            double vy = (measuredVy * (1.0 - FEEDFORWARD_GAIN)) + (intentVyField * FEEDFORWARD_GAIN);

            double realXl = tx - robotX;
            double realYl = ty - robotY;
            double hypot = Math.sqrt((realXl * realXl) + (realYl * realYl));

            double timeOfFlight = hypot / BALL_VELOCITY;

            double compensatedTx = tx - (vx * timeOfFlight * LEAD_MULTIPLIER);
            double compensatedTy = ty - (vy * timeOfFlight * LEAD_MULTIPLIER);

            double xlComp = compensatedTx - robotX;
            double ylComp = compensatedTy - robotY;
            double angleToGoal = Math.atan2(ylComp, xlComp);

            // Predict future heading using the smooth, filtered vHeading
            double futureHeading = robotHeading + (filteredVHeading * (timeOfFlight * LEAD_MULTIPLIER + LATENCY_SEC) * HEADING_LEAD_GAIN);
            double calculatedTurretRad = angleToGoal - futureHeading;

            while (calculatedTurretRad > Math.PI) {
                calculatedTurretRad -= 2 * Math.PI;
            }
            while (calculatedTurretRad < -Math.PI) {
                calculatedTurretRad += 2 * Math.PI;
            }

            double finalServoDegrees = Math.toDegrees(calculatedTurretRad) + 151.5;

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

            prevleft = left;
            prevright = right;

            finalServoDegrees += visionOffsetDeg + samOffset;
            finalServoDegrees = Range.clip(finalServoDegrees, 0, 303);

            if (gamepad1.ps || gamepad2.ps) {
                turret1.setPosition(.5);
                turret2.setPosition(.5);
            } else {
                turret1.setPosition(finalServoDegrees / 303.0);
                turret2.setPosition(finalServoDegrees / 303.0);
            }

            // =========================================================================
            // HOOD LINEAR REGRESSION
            // =========================================================================
            double hpos;
            if (hypot < 130) {
                double d1 = 57.5, d2 = 97.3;
                double v1 = 0.5, v2 = 0.7;
                double slope = (v2 - v1) / (d2 - d1);
                hpos = v1 + (slope * (hypot - d1));
            } else {
                double d1 = 136.5, d2 = 158.1;
                double v1 = 0.7, v2 = 1.0;
                double slope = (v2 - v1) / (d2 - d1);
                hpos = v1 + (slope * (hypot - d1));
            }
            hood.setPosition(Range.clip(hpos, 0.5, 1.0));

            // =========================================================================
            // FLYWHEEL LINEAR REGRESSION
            // =========================================================================
            double vTarget;
            if (hypot < 130) {
                double d1 = 57.5, d2 = 97.3;
                double v1 = 1310, v2 = 1660;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = 1150 + (slope * (hypot - d1));
            } else {
                double d1 = 136.5, d2 = 158.1;
                double v1 = 1900, v2 = 1940;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = farSlope + (slope * (hypot - d1));
            }
            vTarget = Range.clip(vTarget, 0, 2500);

            if (gamepad1.x) {
                var = 1;
            } else if (gamepad1.y) {
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
            // INTAKE LOGIC
            // =========================================================================
            double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakeCmd = 1.0;
            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intakeCmd = -1.0;
            }

            intake.setPower(intakeCmd);

            if (intakeCmd != 0) {
                gecko.setPower(gamepad1.left_bumper || gamepad1.right_bumper ? -1.0 : 1.0);
            } else {
                gecko.setPower(0);
            }

            // =========================================================================
            // DRIVE LOGIC
            // =========================================================================
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
            // ODOMETRY RESET
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
            // TELEMETRY
            // =========================================================================
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("farSlope", farSlope);
            dashboard.sendTelemetryPacket(packet);

            double leadDistX = vx * timeOfFlight * LEAD_MULTIPLIER;
            double leadDistY = vy * timeOfFlight * LEAD_MULTIPLIER;
            double totalLeadInches = Math.sqrt(leadDistX * leadDistX + leadDistY * leadDistY);

            telemetry.addData("Heading Filter Alpha", HEADING_FILTER_ALPHA);
            telemetry.addData("Raw vHeading", rawVHeading);
            telemetry.addData("Filtered vHeading", filteredVHeading);
            telemetry.addData("Feedforward Gain", FEEDFORWARD_GAIN);
            telemetry.addData("Total Lead (Inches)", totalLeadInches);
            telemetry.addData("Distance (Hypot)", hypot);
            telemetry.addData("Target Velocity", vTarget);
            telemetry.addData("Hood Position", hpos);
            telemetry.addData("Turret Angle (Deg)", finalServoDegrees);
            telemetry.addData("turret servo 1 pos", turret1.getPosition());
            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.update();
        }
    }
}