package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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

@Configurable
@TeleOp(name = "Shoot While Move Red", group = "kinematics")
public class stateTeleOpRedbeforeApple extends LinearOpMode {

    Servo hood, turret1, turret2, PTO1, PTO2;
    DcMotorEx f1, f2, gecko;
    DcMotor intake, frontLeft, frontRight, backLeft, backRight;
    GoBildaPinpointDriver pip;

    // Kinematics Constants (From Matt's V2)
    public static double GRAVITY_IN_S2 = 386.088; // 32.174 * 12
    public static double SCORE_HEIGHT_INCHES = 32.0;
    public static double ENTRY_ANGLE_DEG = -30.0;
    public static double TARGET_RADIUS_INCHES = 5.0;

    public static double VEL_DEADZONE = 0.25;
    public static double HEAD_DEADZONE = 0.02;

    public static double MAX_AXIAL_VEL = 91.05;
    public static double MAX_LATERAL_VEL = 65.68;
    public static double FEEDFORWARD_GAIN = 0.6;
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
        double visionOffsetDeg = 0.0;
        double samOffset = 5;

        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevLeft = left;
        boolean prevRight = right;

        waitForStart();

        while (opModeIsActive()) {
            pip.update();

            // 1. Get Robot Inputs & Odometry
            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yawCmd = -gamepad1.right_stick_x;

            double robotX = pip.getPosX(DistanceUnit.INCH);
            double robotY = pip.getPosY(DistanceUnit.INCH);
            double robotHeading = pip.getHeading(AngleUnit.RADIANS);

            // 2. Determine Field-Centric Robot Velocity
            double measuredVx = pip.getVelX(DistanceUnit.INCH);
            double measuredVy = pip.getVelY(DistanceUnit.INCH);
            if (Math.abs(measuredVx) < VEL_DEADZONE) measuredVx = 0.0;
            if (Math.abs(measuredVy) < VEL_DEADZONE) measuredVy = 0.0;

            double intentVxRobot = axial * MAX_AXIAL_VEL;
            double intentVyRobot = lateral * MAX_LATERAL_VEL;

            double blendedVxRobot = (measuredVx * (1.0 - FEEDFORWARD_GAIN)) + (intentVxRobot * FEEDFORWARD_GAIN);
            double blendedVyRobot = (measuredVy * (1.0 - FEEDFORWARD_GAIN)) + (intentVyRobot * FEEDFORWARD_GAIN);

            double fieldVx = (blendedVxRobot * Math.cos(robotHeading)) - (blendedVyRobot * Math.sin(robotHeading));
            double fieldVy = (blendedVxRobot * Math.sin(robotHeading)) + (blendedVyRobot * Math.cos(robotHeading));

            double vMag = Math.sqrt((fieldVx * fieldVx) + (fieldVy * fieldVy));
            double thetaVel = Math.atan2(fieldVy, fieldVx);

            // 3. Stationary Math (Section A with Matt's Constants)
            double realXl = tx - robotX;
            double realYl = ty - robotY;

            // Apply Matt's pass-through point radius offset
            double distanceToGoal = Math.sqrt((realXl * realXl) + (realYl * realYl)) - TARGET_RADIUS_INCHES;
            double thetaLine = Math.atan2(realYl, realXl);

            double thetaEntryRad = Math.toRadians(ENTRY_ANGLE_DEG);

            // Calculate stationary alpha and v0
            double alphaStationary = Math.atan((2 * SCORE_HEIGHT_INCHES / distanceToGoal) - Math.tan(thetaEntryRad));
            double v0Stationary = Math.sqrt((GRAVITY_IN_S2 * Math.pow(distanceToGoal, 2)) /
                    (2 * Math.pow(Math.cos(alphaStationary), 2) * (distanceToGoal * Math.tan(alphaStationary) - SCORE_HEIGHT_INCHES)));

            // 4. Velocity Compensation (Section B)
            double thetaDiff = thetaVel - thetaLine;
            double vrr = -Math.cos(thetaDiff) * vMag;
            double vrt = Math.sin(thetaDiff) * vMag;

            double timeOfFlight = distanceToGoal / (v0Stationary * Math.cos(alphaStationary));

            double vxCompensated = (distanceToGoal / timeOfFlight) + vrr;
            double vxNew = Math.sqrt(Math.pow(vxCompensated, 2) + Math.pow(vrt, 2));
            double vyStationary = v0Stationary * Math.sin(alphaStationary);

            // New Launch Variables
            double alphaNewRad = Math.atan(vyStationary / vxNew);
            double alphaNewDeg = Range.clip(Math.toDegrees(alphaNewRad), 35.0, 55.0);

            double xNew = vxNew * timeOfFlight; // Velocity compensated distance

            // 5. Turret Offset Calculation
            double turretOffsetRad = Math.atan2(vrt, vxCompensated);
            double targetTurretRad = thetaLine - turretOffsetRad - robotHeading;

            while (targetTurretRad > Math.PI) targetTurretRad -= 2.0 * Math.PI;
            while (targetTurretRad < -Math.PI) targetTurretRad += 2.0 * Math.PI;

            // Turret Zero Set Back to 151.5
            double baseServoDegrees = Math.toDegrees(targetTurretRad) + 151.5;

            // Update manual turret offsets
            left = gamepad2.dpad_left;
            right = gamepad2.dpad_right;
            if (left && !prevLeft && !right) samOffset = Range.clip(samOffset + 2.5, -40.0, 40.0);
            if (right && !prevRight && !left) samOffset = Range.clip(samOffset - 2.5, -40.0, 40.0);
            prevLeft = left;
            prevRight = right;

            double finalServoDegrees = Range.clip(baseServoDegrees + visionOffsetDeg + samOffset, 0.0, 303.0);
            double turretPos = finalServoDegrees / 303.0;

            if (gamepad1.ps || gamepad2.ps) {
                turret1.setPosition(0.5);
                turret2.setPosition(0.5);
            } else {
                turret1.setPosition(turretPos);
                turret2.setPosition(turretPos);
            }

            // 6. Map Alpha to Hood Servo (Matt's Linear Regression)
            double hpos = Range.clip((0.025 * alphaNewDeg) - 0.875, 0.0, 1.0);
            hood.setPosition(hpos);

            // 7. Map xNew to Flywheel Ticks (Your Piecewise logic kept intact)
            double vTarget;
            if (xNew < 130.0) {
                double d1 = 57.5, d2 = 97.3;
                double val1 = 1310.0, val2 = 1660.0;
                double slope = (val2 - val1) / (d2 - d1);
                vTarget = val1 + (slope * (xNew - d1));
            } else {
                double d1 = 136.5, d2 = 158.1;
                double val1 = 1900.0, val2 = 1940.0;
                double slope = (val2 - val1) / (d2 - d1);
                vTarget = val1 + (slope * (xNew - d1));
            }
            vTarget = Range.clip(vTarget, 0.0, 2500.0);

            // Firing Controls
            if (gamepad1.x) var = 1;
            else if (gamepad1.y) var = 0;

            if (var == 1) {
                f1.setVelocity(vTarget);
                f2.setVelocity(vTarget);
            } else {
                f1.setVelocity(0.0);
                f2.setVelocity(0.0);
            }

            // Intake & Gecko Controls
            double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
            if (gamepad1.left_bumper || gamepad2.left_bumper) intakeCmd = 1.0;
            else if (gamepad1.right_bumper || gamepad2.right_bumper) intakeCmd = -1.0;
            intake.setPower(intakeCmd);

            if (intakeCmd != 0.0) gecko.setPower((gamepad1.left_bumper || gamepad1.right_bumper) ? -1.0 : 1.0);
            else gecko.setPower(0.0);

            // Drivetrain Controls
            double fl = axial + lateral + yawCmd;
            double fr = axial - lateral - yawCmd;
            double bl = axial - lateral + yawCmd;
            double br = axial + lateral - yawCmd;

            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            frontLeft.setPower(fl / max);
            frontRight.setPower(fr / max);
            backLeft.setPower(bl / max);
            backRight.setPower(br / max);

            // Odometry / Pose Resets
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

            // Telemetry
            telemetry.addData("Compensated Dist (xNew)", xNew);
            telemetry.addData("New Launch Angle", alphaNewDeg);
            telemetry.addData("Turret Offset (rad)", turretOffsetRad);
            telemetry.addData("Flywheel Target", vTarget);
            telemetry.update();
        }
    }
}