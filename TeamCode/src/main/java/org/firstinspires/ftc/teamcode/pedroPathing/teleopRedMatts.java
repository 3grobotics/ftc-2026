package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;


import java.util.List;

@Config
@TeleOp(name = "teleop Red, but matt made it", group = "TeleOp")
public class teleopRedMatts extends LinearOpMode {

    // ================== DASH TUNABLES ==================
    public static boolean REVERSE_TURRET = false;

    // ---- Hood ----
    public static double HOOD_START_POS = .8;

    // ---- Flywheel ----
    public static double TARGET_RPM = 3600;
    public static double RPM_TO_TPS = (TARGET_RPM * 28) / 60;
    public static double VEL_START = (RPM_TO_TPS);
    public static double VEL_STEP = 46.6666666667;
    public static double VEL_MAX = 100000;

    // ---- gecko ----
    public static double GECKO_INTAKE_POWER = 1.0;
    public static double GECKO_REVERSE_POWER = -1.0;
    public static double GECKO_IDLE_POWER = 0.0;

    // ---- Intake ----
    public static double INTAKE_MAX = 1.0;

    private static final int APRILTAG_PIPELINE = 0;

    // ---- Pipeline toggle state ----
    private int currentPipeline = APRILTAG_PIPELINE;
    private boolean prevDpadLeft = false;

    // ---- Quick snap-turn on pipeline switch ----
    private static final double TOGGLE_TURN_POWER = -0.45;
    private static final double TOGGLE_TURN_MS    = 550;

    // --------- flywheel pid ----------------------
    public static double aFLY_KP = 400 ;
    public static double bFLY_KI =   0 ;
    public static double cFLY_KD =   0 ;
    public static double dFLY_KF =  20 ;

    private final ElapsedTime runtime  = new ElapsedTime();

    // Tag 24 trigger memory
    private boolean prevSaw24 = false;

    // Accumulator variable
    private double visionOffsetDeg = 0.0;

    // ===== Intake (gamepad2) =====

    // =========================================================
    // NEW SHOOTER CONSTANTS (Physics Engine)
    // =========================================================
    public static class ShooterConstants {
        public static double G = 32.174 * 12; // Gravity in in/s^2
        public static double SCORE_HEIGHT = 32; // inches (Goal height - Launch height)
        public static double SCORE_ANGLE = Math.toRadians(-30); // Entry angle (radians)
        public static double PASS_THROUGH_POINT_RADIUS = 5; // inches

        // Hardware Limits
        public static double HOOD_MAX_ANGLE = Math.toRadians(55);
        public static double HOOD_MIN_ANGLE = Math.toRadians(35);

        // Linear Regression for Flywheel (Ticks vs Velocity)
        public static double FLYWHEEL_OFFSET = 0;
        public static double FLYWHEEL_MIN_SPEED = 0;
        public static double FLYWHEEL_MAX_SPEED = 6000;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        //Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.setPollRateHz(100);
        //limelight.pipelineSwitch(currentPipeline);
        //limelight.start();

        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor intake     = hardwareMap.get(DcMotor.class, "intake");

        DcMotorEx flywheel1  = hardwareMap.get(DcMotorEx.class, "flywheel1");
        DcMotorEx flywheel2  = hardwareMap.get(DcMotorEx.class, "flywheel2");

        Servo hood = hardwareMap.get(Servo.class, "hood");
        DcMotorEx gecko = hardwareMap.get(DcMotorEx.class, "indexer");
        //Servo   light      = hardwareMap.get(Servo.class, "light");
        Servo   turret1     = hardwareMap.get(Servo.class, "turret");
        Servo   turret2     = hardwareMap.get(Servo.class, "turret2");
        turret1.setDirection(Servo.Direction.REVERSE);
        turret2.setDirection(Servo.Direction.REVERSE);
        hood.setDirection(Servo.Direction.REVERSE);


        FtcDashboard dashboard = FtcDashboard.getInstance();

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);

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

        double oldTime = 0;

        // RED TARGET (Field Centric)
        double target_y = 72;
        double target_x = -72;

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        // 2. RETRIEVE DATA FROM AUTO
        // We use the full path to ensure no import errors
        // 3. FORCE THE HARDWARE TO THE CORRECT SPOT
        // This overwrites whatever "wrong" numbers the hardware has

        // Pinpoint Config
        odo.setOffsets(-42, -90, DistanceUnit.MM);
        odo.setBulkReadScope(defaultRegisters);
        odo.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //odo.resetPosAndIMU();


        double vel = VEL_START;

        boolean prevxx = false, prevbb = false;
        double var = 0;
        double AIM_OFFSET_DEG_LOCAL = 1;
        boolean saw24 = false;

        double aimErrorDeg = 0.0;
        int twistState = 0;
        boolean dPressed = false;
        double v = 0; // Target Velocity for display

        // Physics variables for display
        double debugFlywheelSpeed = 0;
        double debugHoodAngle = 0;
        double debugTurretOffset = 0;

        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(aFLY_KP, bFLY_KI, cFLY_KD, dFLY_KF));
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(aFLY_KP, bFLY_KI, cFLY_KD, dFLY_KF));

        while (opModeInInit()){
            odo.update();
            telemetry.addData("odo x", odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("odo y", odo.getPosY(DistanceUnit.INCH));

            telemetry.update();
        }

        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, -68.02, 28.72, AngleUnit.DEGREES,  270));
            } else if (gamepad1.dpad_down) {
                odo.resetPosAndIMU();
            }


            // =========================================================================
            //  ODOMETRY & PHYSICS PRE-CALCULATIONS
            // =========================================================================
            odo.update();
            Pose2D pos = odo.getPosition();

            // FIX: Get X and Y velocity separately instead of using getVelocity()
            double rVelX = odo.getVelX(DistanceUnit.INCH);
            double rVelY = odo.getVelY(DistanceUnit.INCH);

            // 1. Vector to Goal
            double dx = target_x - pos.getX(DistanceUnit.INCH);
            double dy = target_y - pos.getY(DistanceUnit.INCH);
            double distToGoal = Math.hypot(dx, dy); // hypotenuse
            double angleToGoal = Math.atan2(dy, dx); // Field Centric angle to goal (radians)

            // 2. Robot Velocity Vector (Corrected for Field Centricity)
            double robotVelMag = Math.hypot(rVelX, rVelY);

            // This is the direction the robot is moving relative to itself
            double robotRelativeVelHeading = Math.atan2(rVelY, rVelX);

            // We must add the Robot's Field Heading to get the vector in Field Coordinates
            double currentRobotHeading = odo.getHeading(AngleUnit.RADIANS);
            double robotFieldVelHeading = robotRelativeVelHeading + currentRobotHeading;

            // 3. Coordinate Theta (Angle between Field Velocity and Goal Vector)
            double coordinateTheta = robotFieldVelHeading - angleToGoal;

            // 4. Decompose Velocity (Parallel vs Perpendicular to Goal)
            // Note: Parallel is Negative if moving TOWARDS goal (reducing distance)
            double parallelComponent = -Math.cos(coordinateTheta) * robotVelMag;
            double perpendicularComponent = Math.sin(coordinateTheta) * robotVelMag;

            // =========================================================================
            //  LIMELIGHT & VISION (Keep existing logic)
            // =========================================================================
            /*saw24 = false;
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    for (LLResultTypes.FiducialResult fr : tags) {
                        if (fr.getFiducialId() == 24) {
                            saw24 = true;
                            aimErrorDeg = result.getTx();
                        }
                    }
                }
            }*/

            // =========================================================================
            //  SHOOTER LOGIC (Replaced with Physics)
            // =========================================================================

            // Buttons to arm shooter
            boolean xx = gamepad1.x;
            boolean bb = gamepad1.b;
            if (gamepad1.x) { var = 1; }
            else if (gamepad1.y) { var = 0; }

            // Toggle Logic
            if (gamepad1.right_stick_button && gamepad1.left_stick_button && !dPressed) {
                twistState = (twistState + 1) % 2;
                dPressed = true;
            } else if (!gamepad2.dpad_up) {
                dPressed = false;
            }

            double calculatedTurretRad = 0; // Will hold the physics-based turret angle

            /*switch (twistState){
                case 0:*/ // === PHYSICS MODE (VELOCITY COMPENSATED) ===

                    // A. Initial Launch Components
                    double x_physics = distToGoal - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
                    double y_physics = ShooterConstants.SCORE_HEIGHT;
                    double a_physics = ShooterConstants.SCORE_ANGLE;

                    // Hood Angle (Alpha)
                    double term1 = (2 * y_physics) / x_physics;
                    double term2 = Math.tan(a_physics);
                    double hoodAngle = Math.atan(term1 - term2);
                    hoodAngle = Range.clip(hoodAngle, ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);

                    // Flywheel Speed (v0)
                    double numerator = ShooterConstants.G * x_physics * x_physics;
                    double denom = 2 * Math.pow(Math.cos(hoodAngle), 2) * (x_physics * Math.tan(hoodAngle) - y_physics);
                    double flywheelSpeed = Math.sqrt(numerator / denom);

                    // B. Velocity Compensation
                    double vz = flywheelSpeed * Math.sin(hoodAngle);
                    double timeByAir = x_physics / (flywheelSpeed * Math.cos(hoodAngle));

                    // Adjusted X velocity: (x / t) + parallel_vel
                    double ivr = (x_physics / timeByAir) + parallelComponent;
                    double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
                    double ndr = nvr * timeByAir;

                    // C. Re-Calculate with Compensation
                    hoodAngle = Math.atan(vz / nvr);
                    hoodAngle = Range.clip(hoodAngle, ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);

                    double numComp = ShooterConstants.G * ndr * ndr;
                    double denomComp = 2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y_physics);
                    flywheelSpeed = Math.sqrt(numComp / denomComp);

                    // D. Calculate Turret Offset (Leading the target)
                    double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);

                    // Apply to Hardware
                    double targetTPS = getFlywheelTicksFromVelocity(flywheelSpeed);
                    v = targetTPS; // For telemetry

                    if (var == 1) {
                        flywheel1.setVelocity(targetTPS);
                        flywheel2.setVelocity(targetTPS);
                    } else {
                        flywheel1.setVelocity(0);
                        flywheel2.setVelocity(0);
                    }

                    hood.setPosition(getHoodTicksFromDegrees(Math.toDegrees(hoodAngle)));

                    // Base Turret Angle Calculation for later application
                    double robotHeading = odo.getHeading(AngleUnit.RADIANS);
                    calculatedTurretRad = angleToGoal - robotHeading + turretVelCompOffset;

                    // Debug data
                    debugFlywheelSpeed = flywheelSpeed;
                    debugHoodAngle = Math.toDegrees(hoodAngle);
                    debugTurretOffset = Math.toDegrees(turretVelCompOffset);
                    //break;

                /*case 1: // === MANUAL TUNING MODE ===
                    flywheel1.setVelocity(vel);
                    flywheel2.setVelocity(vel);
                    if (xx && !prevxx && !bb) {
                        vel = Range.clip(vel + VEL_STEP, 0.0, VEL_MAX);
                    }
                    if (bb && !prevbb && !xx) {
                        vel = Range.clip(vel - 46.6666666667, 0.0, VEL_MAX);
                    }

                    // Recalculate basic turret angle without physics offset for Manual mode
                    double rH = odo.getHeading(AngleUnit.RADIANS);
                    calculatedTurretRad = angleToGoal - rH;
                    break;
            }
            prevxx = xx;
            prevbb = bb;*/

            // =========================================================================
            //  TURRET APPLICATION (Physics Angle + Vision Integration)
            // =========================================================================

            // 1. Wrap angle (-180 to 180)
            while(calculatedTurretRad > Math.PI) calculatedTurretRad -= 2*Math.PI;
            while(calculatedTurretRad < -Math.PI) calculatedTurretRad += 2*Math.PI;

            // 2. Convert to Degrees and apply center offset (159 deg)
            double finalServoDegrees = Math.toDegrees(calculatedTurretRad) + 151.5;

            // 3. Vision Integration (Keep your original logic)
            /*if (saw24) {
                double TRACKING_SPEED = 0.2;
                double DIRECTION_FLIP = -1.0;
                visionOffsetDeg += (aimErrorDeg * TRACKING_SPEED * DIRECTION_FLIP);
                visionOffsetDeg = Range.clip(visionOffsetDeg, -27.5, 27.5);
            }*/

            // 4. Combine Physics + Vision
            finalServoDegrees += visionOffsetDeg + AIM_OFFSET_DEG_LOCAL;

            // 5. Clamp and Set
            finalServoDegrees = Range.clip(finalServoDegrees, 0, 303);
            //turret1.setPosition(finalServoDegrees / 250.0);
            //turret2.setPosition(finalServoDegrees / 250.0);
            if (gamepad1.ps || gamepad2.ps) {
                turret1.setPosition(.5);
                turret2.setPosition(.5);
            } else {
                turret1.setPosition(finalServoDegrees / 303);
                turret2.setPosition(finalServoDegrees / 303);
            }


            // =========================================================================
            //  DRIVE (Robot Centric - kept as is)
            // =========================================================================
            {
                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yawCmd = gamepad1.right_stick_x;

                double fl = axial + lateral + yawCmd;
                double fr = axial - lateral - yawCmd;
                double bl = axial - lateral + yawCmd;
                double br = axial + lateral - yawCmd;

                double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
                frontLeft.setPower(fl / max);
                frontRight.setPower(fr / max);
                backLeft.setPower(bl / max);
                backRight.setPower(br / max);
            }

            // =========================================================================
            //  PIPELINE TOGGLE & SNAP TURN (Kept as is)
            // =========================================================================
            /*{
                boolean dpadLeft = gamepad1.dpad_left;
                if (dpadLeft && !prevDpadLeft) {
                    currentPipeline = (currentPipeline == 0) ? 1 : 0;
                    limelight.pipelineSwitch(currentPipeline);
                    double turnDir = (currentPipeline == 1) ? 1.0 : -1.0;
                    double endMs = runtime.milliseconds() + TOGGLE_TURN_MS;
                    while (opModeIsActive() && runtime.milliseconds() < endMs) {
                        double yawSpin = TOGGLE_TURN_POWER * turnDir;
                        frontLeft.setPower(yawSpin);
                        frontRight.setPower(-yawSpin);
                        backLeft.setPower(-yawSpin);
                        backRight.setPower(yawSpin);
                    }
                    frontLeft.setPower(0); frontRight.setPower(0); backLeft.setPower(0); backRight.setPower(0);
                }
                prevDpadLeft = dpadLeft;
            }*/

            // =========================================================================
            //  INTAKE LOGIC (Kept as is)
            // =========================================================================
            {
                double rt = gamepad1.right_trigger + gamepad2.right_trigger;
                double lt = gamepad1.left_trigger + gamepad2.left_trigger;
                double intakeCmd = rt - lt;

                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    intakeCmd = 1.0;
                } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    intakeCmd = -1.0;
                }
                intakeCmd = Range.clip(intakeCmd, -INTAKE_MAX, INTAKE_MAX);
                if (intakeCmd > 0 && intakeCmd <= 1) {
                    gecko.setPower(gamepad1.left_bumper ? GECKO_REVERSE_POWER : GECKO_INTAKE_POWER);
                } else if (intakeCmd < 0 && intakeCmd >= -1) {
                    gecko.setPower(gamepad1.right_bumper ? GECKO_INTAKE_POWER : GECKO_REVERSE_POWER);
                } else {
                    gecko.setPower(GECKO_IDLE_POWER);
                }
                intake.setPower(intakeCmd);
            }


             if (gamepad1.dpad_up) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, -46.5, 52.5, AngleUnit.DEGREES, 127));
             } else if (gamepad1.dpad_down) {
                 odo.resetPosAndIMU();
             }

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            oldTime = newTime;

            // =========================================================================
            //  TELEMETRY
            // =========================================================================
            {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("hood", hood.getPosition());
                packet.put("turret", turret1.getPosition());
                packet.put("Vision Offset", visionOffsetDeg);
                packet.put("Target Vel", v);
                packet.put("Act Flywheel", ((flywheel1.getVelocity() * 60) / 28));
                packet.put("Phys HoodAng", debugHoodAngle);
                packet.put("Phys TurretOff", debugTurretOffset);

                dashboard.sendTelemetryPacket(packet);

                telemetry.addData("Mode", twistState == 0 ? "PHYSICS" : "MANUAL");
                telemetry.addData("Hood", "%.2f", hood.getPosition());
                telemetry.addData("Turret", "%.2f", getHoodTicksFromDegrees(Math.toDegrees(hoodAngle)) );
                telemetry.addData("Vision Offset", "%.2f", visionOffsetDeg);
                telemetry.addData("Dist to Goal", "%.1f", distToGoal);
                telemetry.addData("Launch Speed (in/s)", "%.1f", debugFlywheelSpeed);
                telemetry.addData("Launch Angle", "%.1f", debugHoodAngle);
                telemetry.addData("Turret Offset", "%.1f", debugTurretOffset);
                telemetry.addData("Loop Time", "%.1f ms", loopTime * 1000);
                telemetry.addData("odo x", odo.getPosX(DistanceUnit.INCH));
                telemetry.addData("odo y", odo.getPosY(DistanceUnit.INCH));
                telemetry.update();
            }
        }
    }

    // =========================================================================
    //  HELPER FUNCTIONS
    // =========================================================================

    public static double getFlywheelTicksFromVelocity(double velocity) {
        // TUNED VALUES from data collected at 35 degrees
        // Equation: Ticks/s = 8.2 * velocity(in/s) - 350
        double tps = 8.2 * velocity - 350;

        // Safety: Never command negative speed, and clamp to Max
        return Range.clip(tps, 0, ShooterConstants.FLYWHEEL_MAX_SPEED);
    }

    public static double getHoodTicksFromDegrees(double degrees) {
        // Linear regression: y = .025x - .875
        return .025 * degrees - .875;
    }
}