package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Config
@TeleOp(name = " old state TeleOp Red", group = "linear equations test")
public class oldStateTeleOpRed extends LinearOpMode {

    Servo hood, turret1, turret2, PTO1, PTO2;
    DcMotorEx f1, f2, gecko;
    DcMotor intake;

    // PedroPathing Follower
    Follower follower;

    // RGB Indicator Light
    ServoImplEx light;

    // goBILDA RGB PWM tuning values (Recalculated for the 500-2500 expanded range)
    private final double COLOR_GOOD = 0.50; // Exactly 1500µs = Green
    private final double COLOR_RED = 0.30;  // Exactly 1100µs = Red

    // --- LIMELIGHT & FILTER VARIABLES ---
    private Limelight3A camera;
    private final SelfTuningKalmanFilter filterX = new SelfTuningKalmanFilter();
    private final SelfTuningKalmanFilter filterY = new SelfTuningKalmanFilter();
    private final SelfTuningKalmanFilter filterH = new SelfTuningKalmanFilter();
    private final double DISTANCE_TOLERANCE_INCHES = 2.5;
    private final double HEADING_TOLERANCE_RADIANS = Math.toRadians(5.0);

    public static double farSlope = 1750;

    @Override
    public void runOpMode() {
        // Hardware Mapping
        hood = hardwareMap.get(Servo.class, "hood");
        turret1 = hardwareMap.get(Servo.class, "turret");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        f1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        f2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        gecko = hardwareMap.get(DcMotorEx.class, "gecko");

        // Map Limelight & Light
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        light = hardwareMap.get(ServoImplEx.class, "light");
        light.setPwmRange(new PwmControl.PwmRange(500, 2500));
        light.setPosition(COLOR_GOOD);

        // Map PedroPathing Follower correctly via Constants
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Component Directions
        f1.setDirection(DcMotorSimple.Direction.REVERSE);
        f2.setDirection(DcMotorSimple.Direction.REVERSE);
        hood.setDirection(Servo.Direction.REVERSE);
        turret1.setDirection(Servo.Direction.REVERSE);

        double tx = 144; // Target X
        double ty = 144;  // Target Y
        double var = 0;
        double visionOffsetDeg = 0.0;
        double axial = 0;
        double lateral = 0;
        double yawCmd = 0;

        f1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        f2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        FtcDashboard dashboard = FtcDashboard.getInstance();

        double samOffset = 5;
        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;

        waitForStart();

        // Start Limelight once play is pressed
        camera.start();
        camera.pipelineSwitch(0);

        telemetry.addLine(">>> AUTO POSITION LOADED <<<");
        follower.startTeleopDrive();

        while (opModeIsActive()) {

            follower.update(); // Update odometry first

            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotH = follower.getPose().getHeading();

            // =========================================================================
            //  LIMELIGHT POSE UPDATE & RGB LIGHT LOGIC
            // =========================================================================
            Pose camPose = getFilteredPoseFromCamera();
            if (camPose != null) {
                double distanceError = Math.hypot(
                        camPose.getX() - robotX,
                        camPose.getY() - robotY
                );

                double headingError = Math.abs(camPose.getHeading() - robotH);
                while (headingError > Math.PI) headingError -= 2 * Math.PI;
                headingError = Math.abs(headingError);

                // Calculate a ratio from 0.0 (perfect) to 1.0 (max error tolerance)
                double errorRatio = Math.min(1.0, distanceError / DISTANCE_TOLERANCE_INCHES);

                // Interpolate the servo position between the GOOD color and RED color based on the error
                double currentColorPos = COLOR_GOOD + (errorRatio * (COLOR_RED - COLOR_GOOD));
                light.setPosition(currentColorPos);

                if (distanceError > DISTANCE_TOLERANCE_INCHES || headingError > HEADING_TOLERANCE_RADIANS) {
                    follower.setPose(camPose);
                    // Snap light to fully red since it breached tolerance and forced an update
                    light.setPosition(COLOR_RED);

                    robotX = follower.getPose().getX();
                    robotY = follower.getPose().getY();
                    robotH = follower.getPose().getHeading();
                }
            } else {
                // Fully off / no tag visible
                light.setPosition(COLOR_RED);
            }

            // =========================================================================
            //  TURRET APPLICATION
            // =========================================================================
            double xl = tx - robotX;
            double yl = ty - robotY;
            double hypot = Math.sqrt((xl * xl) + (yl * yl));

            double angleToGoal = Math.atan2(yl, xl);
            double calculatedTurretRad = angleToGoal - robotH;
            double finalServoDegrees = Math.toDegrees(calculatedTurretRad) + 151.5;

            while (finalServoDegrees < 0)   finalServoDegrees += 360.0;
            while (finalServoDegrees >= 360.0) finalServoDegrees -= 360.0;

            left = gamepad2.dpad_left;
            right = gamepad2.dpad_right;

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
                turret1.setPosition(finalServoDegrees / 303);
                turret2.setPosition(finalServoDegrees / 303);
            }

            // =========================================================================
            //  HOOD & FLYWHEEL LOGIC
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

            if (gamepad1.x) var = 1;
            else if (gamepad1.y) var = 0;

            // Ensure the robot is stationary before allowing the flywheel to spin

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
            //  DRIVE LOGIC (PedroPathing Native TeleOp)
            // =========================================================================
            axial   = -gamepad1.left_stick_y;
            lateral = -gamepad1.left_stick_x;
            yawCmd  = -gamepad1.right_stick_x;

            follower.setTeleOpDrive(axial, lateral, yawCmd, true);

            // =========================================================================
            //  ODOMETRY RESET (Mapped to PedroCoordinates)
            // =========================================================================
            if (gamepad1.dpad_up) {
                follower.setPose(new Pose(110, 137, 0));
                samOffset = 0;
            } else if (gamepad1.dpad_down) {
                follower.setPose(new Pose(0, 0, 0, FTCCoordinates.INSTANCE)
                        .getAsCoordinateSystem(PedroCoordinates.INSTANCE));
                samOffset = 0;
            } else if (gamepad1.dpad_left){
                follower.setPose(new Pose(62, -65, 0, FTCCoordinates.INSTANCE)
                        .getAsCoordinateSystem(PedroCoordinates.INSTANCE));
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
            if (camPose != null) {
                telemetry.addData("Cam X (Filtered)", camPose.getX());
                telemetry.addData("Cam Y (Filtered)", camPose.getY());
            }
            telemetry.update();
        }

        camera.stop();
    }

    // =========================================================================
    //  LIMELIGHT HELPER METHODS & FILTERS
    // =========================================================================
    Pose getFilteredPoseFromCamera() {
        LLResult llResult = camera.getLatestResult();

        if (llResult == null || !llResult.isValid()) return null;

        Pose3D botpose = llResult.getBotpose();
        if (botpose == null || botpose.getPosition() == null) return null;

        Position thePosition = botpose.getPosition();

        double rawXInches = thePosition.x * 39.3700787;
        double rawYInches = thePosition.y * 39.3700787;

        double rawHeadingRad;
        try {
            rawHeadingRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        } catch (Exception e) {
            rawHeadingRad = follower.getPose().getHeading();
        }

        double filteredX = filterX.update(rawXInches);
        double filteredY = filterY.update(rawYInches);
        double filteredH = filterH.updateAngle(rawHeadingRad);

        return new Pose(filteredX, filteredY, filteredH, FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    public static class SelfTuningKalmanFilter {
        private final double Q = 0.1;
        private final double BASE_R = 0.5;
        private double x;
        private double p = 1.0;
        private boolean initialized = false;

        public double update(double measurement) {
            if (!initialized) {
                x = measurement;
                p = 1.0;
                initialized = true;
                return x;
            }

            double residual = Math.abs(measurement - x);
            double dynamicR = BASE_R + (residual * residual * 0.5);

            p = p + Q;
            double k = p / (p + dynamicR);
            x = x + k * (measurement - x);
            p = (1 - k) * p;

            return x;
        }

        public double updateAngle(double measurement) {
            if (!initialized) {
                x = measurement;
                p = 1.0;
                initialized = true;
                return x;
            }

            double diff = measurement - x;
            while (diff > Math.PI) diff -= 2 * Math.PI;
            while (diff < -Math.PI) diff += 2 * Math.PI;

            double residual = Math.abs(diff);
            double dynamicR = BASE_R + (residual * residual * 2.0);

            p = p + Q;
            double k = p / (p + dynamicR);

            x = x + k * diff;

            while (x > Math.PI) x -= 2 * Math.PI;
            while (x < -Math.PI) x += 2 * Math.PI;

            p = (1 - k) * p;
            return x;
        }
    }
}