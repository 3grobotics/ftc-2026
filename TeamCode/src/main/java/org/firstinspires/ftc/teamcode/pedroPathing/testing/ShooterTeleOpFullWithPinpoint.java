package org.firstinspires.ftc.teamcode.pedroPathing.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;
import org.firstinspires.ftc.teamcode.pedroPathing.ColorCamera2;

/**
 * FULL SHOOTER TELEOP — ALL SECTIONS INCLUDED (NO DASHBOARD)
 * Exact hood mapping (32° @ 0.7 → 55° @ 0.0)
 * PDF Section A physics + Section B velocity compensation
 * GoBILDA pip for live position + velocities
 * Flywheel gear-ratio support
 * Ready for manual cc TUNING in the constants section below
 */
@TeleOp(name = "TeleOp - Full Shooter (pip + No Dashboard)", group = "FTC Team")
public class ShooterTeleOpFullWithPinpoint extends LinearOpMode {

    // ==================== HARDWARE ====================
    hardwareSubNewBot h;
    varSub v;

    // ==================== TUNING CONSTANTS ====================
    // Change these numbers directly in the code and redeploy.
    // All values have comments with recommended starting points and cc TUNING advice.

    // PHYSICS
    private static final double GRAVITY_IN_PER_S2 = 386.1; // g in inches/s² (do not change)

    // HOOD SERVO MAPPING (your exact request)
    private static final double HOOD_SERVO_POS_AT_32_DEG = 0.7;
    private static final double HOOD_SERVO_POS_AT_55_DEG = 0.0;

    // DESIRED BALL ENTRY ANGLE AT GOAL
    private static double DESIRED_ENTRY_ANGLE_DEG = -38.0; // cc TUNING: most teams start -30 to -45
    // High shots → make more negative
    // Low shots → make less negative

    // GOAL HEIGHT OFFSET (vertical distance from hood exit to goal center)
    private static double GOAL_HEIGHT_OFFSET_INCHES = 25; // cc TUNING: measure precisely on your robot

    // FLYWHEEL GEAR RATIO SECTION
    private static double MOTOR_RATED_RPM = 5800; // cc TUNING: change to your motor spec (312, 435, 1620, etc.)
    private static double FLYWHEEL_GEAR_RATIO = 1.33; // cc TUNING: 1.0 = direct drive, 2.0 = 2:1 speedup, etc.
    private static double FLYWHEEL_V0_TO_RPM_SCALE = 11; // cc TUNING: fine-tune after gear ratio (usually 0.95–1.15)

    // FIELD GOAL POSITION (in inches)
    private static double GOAL_FIELD_X_INCHES = -72; // cc TUNING: set to actual goal center
    private static double GOAL_FIELD_Y_INCHES = 72;

    // SAFETY / LIMITS
    private static final double MIN_DISTANCE_INCHES = 20.0;
    private static final double FLYWHEEL_POWER_MAX = 1.0;


    // TURRET TUNING
    private static double TURRET_OFFSET_DEG = 31; // TUNING: adjust ±5° if turret points slightly off when facing goal

    double samOffset;

    @Override
    public void runOpMode() {
        // ==================== HARDWARE INIT ====================
        h = new hardwareSubNewBot(hardwareMap);
        v = new varSub();

        h.pip.resetPosAndIMU();

        telemetry.addData("Status", "pip + Shooter Ready");
        telemetry.update();


        double deltaX;
        double deltaY;
        double x;
        double y;

        // Robot velocity aligned with shot direction (for Section B)
        double shotAngleRad;
        double robotVx;
        double robotVy;

        double robotX_in;
        double robotY_in;
        double headingRad;

        double vx_local;
        double vy_local;

        // Local velocities → field-frame
        double vx_field;
        double vy_field;

        double[] compensated;

        double alphaDeg;
        double v0;

        double hoodPos;
        double power;
        double velocityTicksPerSec;


        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;

        boolean aa = gamepad2.a;
        boolean bb = gamepad2.b;
        boolean prevaa = aa;
        boolean prevbb = bb;

        waitForStart();

        while (opModeIsActive()) {

            h.pip.update(); // CRITICAL — call every loop

            // ==================== LIMELIGHT CORRECTION PLACEHOLDER ====================
            // Uncomment and replace with your existing Limelight code if you want corrections
            // if (yourLimelightHasValidTag()) {
            // Pose2D limelightPose = getLimelightRobotPose();
            // if (limelightPose != null) pip.setPosition(limelightPose);
            // }

            // ==================== LIVE pip DATA ====================
            

             robotX_in = h.pip.getPosX(DistanceUnit.INCH);
             robotY_in = h.pip.getPosY(DistanceUnit.INCH);
             headingRad = h.pip.getHeading(AngleUnit.RADIANS);

             vx_local =  h.pip.getVelX(DistanceUnit.INCH);
             vy_local =  h.pip.getVelY(DistanceUnit.INCH);

            // Local velocities → field-frame
             vx_field = vx_local * Math.cos(headingRad) - vy_local * Math.sin(headingRad);
             vy_field = vx_local * Math.sin(headingRad) + vy_local * Math.cos(headingRad);

            // ==================== SHOOTER CALCULATION ====================
            if (gamepad1.a) {
                // Real distance to your goal (-72, 72)
                deltaX = GOAL_FIELD_X_INCHES - robotX_in;
                deltaY = GOAL_FIELD_Y_INCHES - robotY_in;
                x = Math.hypot(deltaX, deltaY);
                y = GOAL_HEIGHT_OFFSET_INCHES;

                if (x < MIN_DISTANCE_INCHES) {
                    x = 80.0; // safety fallback
                }

               compensated = calculateCompensatedLaunch(x, y, DESIRED_ENTRY_ANGLE_DEG, 0.0, 0.0);

               alphaDeg = compensated[0];
               v0 = compensated[1];

               hoodPos = hoodAngleToServo(alphaDeg);
               velocityTicksPerSec = calculateFlywheelVelocity(v0);

                h.hood.setPosition(hoodPos);
                h.flywheel1.setVelocity(velocityTicksPerSec);
                h.flywheel2.setVelocity(velocityTicksPerSec);

                telemetry.addData("Distance to Goal", "%.1f in", x);
                telemetry.addData("Launch Angle α", "%.1f°", alphaDeg);
                telemetry.addData("Flywheel Velocity Command (ticks/sec)", "%.0f", velocityTicksPerSec);
            } else {
                h.flywheel1.setVelocity(0);
                h.flywheel2.setVelocity(0);
            }

            // 1. Get the angle from your method
            double turretTargetRad = getCompensatedTurretAngleRad();

            // 2. Normalize it (Standard Math)
            while (turretTargetRad > Math.PI) turretTargetRad -= 2 * Math.PI;
            while (turretTargetRad < -Math.PI) turretTargetRad += 2 * Math.PI;

            // 3. Convert to your servo scale
            double baseServoDegrees = Math.toDegrees(turretTargetRad) + (314.6112145 / 2.0);
            // Note: Changed from minus to plus depending on your turret's "center" position

            // Gamepad trim logic
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

            if (aa && !prevaa && !bb) {
                samOffset = Range.clip(samOffset + 2.5, -40, 40);
            }
            if (bb && !prevbb && !aa) {
                samOffset = Range.clip(samOffset - 2.5, -40, 40);
            }

            prevaa = aa;
            prevbb = bb;

            double finalServoDegrees = baseServoDegrees + v.visionOffsetDeg + samOffset;

            /*if(h.pip.getHeading(AngleUnit.DEGREES) >= 110 || h.pip.getHeading(AngleUnit.DEGREES) <= 156){
                if (finalServoDegrees < 180){
                    h.frontLeft.setPower(1);
                    h.backLeft.setPower( 1);
                    h.frontRight.setPower(-1);
                    h.backRight.setPower(-1);

                } else {
                    h.frontLeft.setPower(-1);
                    h. backLeft.setPower(-1);
                    h.frontRight.setPower(1);
                    h.backRight.setPower(1);

                }
            }*/

                h.turret1.setPosition(Math.abs((finalServoDegrees) / 314.6112145));
                h.turret2.setPosition(Math.abs((finalServoDegrees) / 314.6112145));

            telemetry.update();
        }
    }

    // ==================== SECTION B: VELOCITY COMPENSATION ====================
    private double[] calculateCompensatedLaunch(double x, double y, double thetaDeg,
                                                double robotVx, double robotVy) {

        double alphaStat = calculateLaunchAngle(x, y, thetaDeg);
        double v0Stat = calculateLaunchVelocity(x, y, thetaDeg, alphaStat);

        double alphaRad = Math.toRadians(alphaStat);
        double t = (Math.cos(alphaRad) > 0) ? x / (v0Stat * Math.cos(alphaRad)) : 0.0;

        double xPrime = x - robotVx * t;
        double yPrime = y - robotVy * t;
        if (xPrime < MIN_DISTANCE_INCHES) xPrime = MIN_DISTANCE_INCHES;

        double alphaComp = calculateLaunchAngle(xPrime, yPrime, thetaDeg);
        double v0Comp = calculateLaunchVelocity(xPrime, yPrime, thetaDeg, alphaComp);

        return new double[]{alphaComp, v0Comp};
    }

    // ==================== PDF SECTION A PHYSICS ====================
    private double calculateLaunchAngle(double x, double y, double thetaDeg) {
        double thetaRad = Math.toRadians(thetaDeg);
        double tanAlpha = 2 * y / x - Math.tan(thetaRad);
        return Math.toDegrees(Math.atan(tanAlpha));
    }

    private double calculateLaunchVelocity(double x, double y, double thetaDeg, double alphaDeg) {
        double alphaRad = Math.toRadians(alphaDeg);
        double thetaRad = Math.toRadians(thetaDeg);
        double cosAlphaSq = Math.cos(alphaRad) * Math.cos(alphaRad);
        double tanDiff = Math.tan(alphaRad) - Math.tan(thetaRad);
        double v0Sq = (GRAVITY_IN_PER_S2 * x) / (cosAlphaSq * tanDiff);
        return (v0Sq > 0) ? Math.sqrt(v0Sq) : 0.0;
    }

    // ==================== HOOD SERVO MAPPING (your exact request) ====================
    private double hoodAngleToServo(double angleDeg) {
        double m = (55.0 - 32.0) / (0.0 - 0.7); // slope from 32° @ 0.7 to 55° @ 0.0
        return Math.max(0.0, Math.min(.7, (angleDeg - 55.0) / (-m)));
    }

    // ==================== FLYWHEEL POWER WITH GEAR RATIO ====================
    private double calculateFlywheelVelocity(double v0) {
        double maxRPM = MOTOR_RATED_RPM * FLYWHEEL_GEAR_RATIO;
        double rpmNeeded = v0 * FLYWHEEL_V0_TO_RPM_SCALE;
        return (rpmNeeded / 60) * 28;


    }

    /*private double calculateFlywheelPower(double v0) {
        double maxRPM = MOTOR_RATED_RPM * FLYWHEEL_GEAR_RATIO;
        double rpmNeeded = v0 * FLYWHEEL_V0_TO_RPM_SCALE;
        return Math.min(FLYWHEEL_POWER_MAX, rpmNeeded / maxRPM);


    }*/

    /**
     * NEW: Velocity-compensated turret angle in radians (front of robot = 0)
     * Uses Section B virtual-goal math so the turret leads the target correctly while moving.
     * Returns normalized angle (-π to +π).
     *
     * TUNING NOTES:
     * - TURRET_OFFSET_DEG: adjust if the turret is slightly off when robot faces the goal (±5° steps)
     * - The method re-uses your existing DESIRED_ENTRY_ANGLE_DEG and GOAL_HEIGHT_OFFSET_INCHES
     * - Once hood is tuned, this turret angle will automatically be correct
     */
    private double getCompensatedTurretAngleRad() {

        // Real distance and direction to goal
        double  deltaX = GOAL_FIELD_X_INCHES - h.pip.getPosX(DistanceUnit.INCH);
        double deltaY = GOAL_FIELD_Y_INCHES - h.pip.getPosY(DistanceUnit.INCH);
        double x = Math.hypot(deltaX, deltaY);
        double y = GOAL_HEIGHT_OFFSET_INCHES;

        // Safety fallback
        if (x < MIN_DISTANCE_INCHES) x = 80.0;

        // 1. Stationary launch parameters (same as hood)
        double alphaStat = calculateLaunchAngle(x, y, DESIRED_ENTRY_ANGLE_DEG);
        double v0Stat = calculateLaunchVelocity(x, y, DESIRED_ENTRY_ANGLE_DEG, alphaStat);

        // 2. Approximate flight time
        double alphaRad = Math.toRadians(alphaStat);
        double t = (Math.cos(alphaRad) > 0) ? x / (v0Stat * Math.cos(alphaRad)) : 0.0;

        // 3. Virtual goal (compensated for robot velocity)
        double xPrime = x - h.pip.getVelX(DistanceUnit.INCH) * t;
        double yPrime = y - h.pip.getVelY(DistanceUnit.INCH) * t;
        if (xPrime < MIN_DISTANCE_INCHES) xPrime = MIN_DISTANCE_INCHES;

        // 4. Compensated heading to virtual goal
        double compensatedHeadingRad = Math.atan2(yPrime, xPrime);

        // 5. Relative turret angle (0 = front of robot)
         double turretTargetRad = compensatedHeadingRad + h.pip.getHeading(AngleUnit.RADIANS) + Math.toRadians(TURRET_OFFSET_DEG);

        // Normalize to -π ... +π
        //turretTargetRad = ((turretTargetRad + Math.PI) % (2 * Math.PI)) - Math.PI;

        return turretTargetRad;
    }
}