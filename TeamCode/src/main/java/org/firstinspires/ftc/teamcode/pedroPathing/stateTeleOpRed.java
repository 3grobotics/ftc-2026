package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Base64;
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.BadAppleData;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.Arrays;

@Configurable
@TeleOp(name = "animation + shoot while move test red tele", group = "linear equations test")
public class stateTeleOpRed extends LinearOpMode {

    Servo hood, turret1, turret2;
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

    public static double MAX_AXIAL_VEL = 91.05;
    public static double MAX_LATERAL_VEL = 65.68;
    public static double FEEDFORWARD_GAIN = 0.6;
    public static double HEADING_FILTER_ALPHA = 0.2;

    // Animation Config
    final int VIDEO_WIDTH = 64;
    final int VIDEO_HEIGHT = 64;
    final double FRAME_TIME_MS = 100.0; // 10 FPS

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
        backRight = hardwareMap.get(DcMotor.class, "backRight ");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft ");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft ");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        turret1.setDirection(Servo.Direction.REVERSE);

        pip.setOffsets(-42, -90, DistanceUnit.MM);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Renderer and Display Setup
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        //Renderer animationRenderer = new Renderer(VIDEO_WIDTH, VIDEO_HEIGHT);
        ElapsedTime animationTimer = new ElapsedTime();

        pip.resetPosAndIMU();
        double tx = -72, ty = 72;
        double visionOffsetDeg = 0.0, samOffset = 5, var = 0;
        f1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        f2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));

        double filteredVHeading = 0.0;
        int currentFrameIdx = 0;

        waitForStart();
        animationTimer.reset();

        while (opModeIsActive()) {
            pip.update();

            // Drive Controls
            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yawCmd = -gamepad1.right_stick_x;
            double robotX = pip.getPosX(DistanceUnit.INCH);
            double robotY = pip.getPosY(DistanceUnit.INCH);
            double robotHeading = pip.getHeading(AngleUnit.RADIANS);

            // Physics and Lead Calculations
            double measuredVx = pip.getVelX(DistanceUnit.INCH);
            double measuredVy = pip.getVelY(DistanceUnit.INCH);
            double rawVHeading = pip.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            filteredVHeading = (HEADING_FILTER_ALPHA * rawVHeading) + ((1.0 - HEADING_FILTER_ALPHA) * filteredVHeading);

            double fieldVx = ((measuredVx * (1.0 - FEEDFORWARD_GAIN)) + (axial * MAX_AXIAL_VEL * FEEDFORWARD_GAIN)) * Math.cos(robotHeading);
            double fieldVy = ((measuredVy * (1.0 - FEEDFORWARD_GAIN)) + (lateral * MAX_LATERAL_VEL * FEEDFORWARD_GAIN)) * Math.sin(robotHeading);

            double hypot = Math.hypot(tx - robotX, ty - robotY);
            double timeOfFlight = hypot / BALL_VELOCITY;
            double xlComp = (tx - (fieldVx * timeOfFlight * LEAD_MULTIPLIER)) - robotX;
            double ylComp = (ty - (fieldVy * timeOfFlight * LEAD_MULTIPLIER)) - robotY;

            double targetTurretRad = Math.atan2(ylComp, xlComp) - (robotHeading + (filteredVHeading * (timeOfFlight * LEAD_MULTIPLIER + LATENCY_SEC) * HEADING_LEAD_GAIN));
            double finalServoDegrees = Range.clip(Math.toDegrees(Math.atan2(Math.sin(targetTurretRad), Math.cos(targetTurretRad))) + 202 + visionOffsetDeg + samOffset, 0, 404);

            turret1.setPosition(finalServoDegrees / 404.0);
            turret2.setPosition(finalServoDegrees / 404.0);

            // Motor Power
            double max = Math.max(1.0, Math.max(Math.abs(axial + lateral + yawCmd), Math.abs(axial + lateral - yawCmd)));
            frontLeft.setPower((axial + lateral + yawCmd) / max);
            frontRight.setPower((axial - lateral - yawCmd) / max);
            backLeft.setPower((axial - lateral + yawCmd) / max);
            backRight.setPower((axial + lateral - yawCmd) / max);

            /* --- ANIMATION ENGINE --- */ /*{
            if (animationTimer.milliseconds() >= FRAME_TIME_MS && BadAppleData.FRAMES.length > 0) {
                animationTimer.reset();
                animationRenderer.clear();

                byte[] frameData = Base64.decode(BadAppleData.FRAMES[currentFrameIdx], Base64.DEFAULT);
                int bitIndex = 0;
                for (int goalHeight = 0; goalHeight < VIDEO_HEIGHT; goalHeight++) {
                    for (int x = 0; x < VIDEO_WIDTH; x++) {
                        int bytePos = bitIndex / 8;
                        int bitPos = 7 - (bitIndex % 8);
                        if (bytePos < frameData.length && ((frameData[bytePos] >> bitPos) & 1) == 1) {
                            animationRenderer.setPixel(x, goalHeight);
                        }
                        bitIndex++;
                    }
                }
                currentFrameIdx = (currentFrameIdx + 1) % BadAppleData.FRAMES.length;
            } }

            // --- TELEMETRY ---
            telemetry.addLine(animationRenderer.renderHtml());
            telemetry.addData("Status", "Running Red TeleOp");
            telemetry.update();*/

            // --- PANELS FIELD RENDERING ---
            try {
                // Ensure field features are drawn
                PanelsField.INSTANCE.getField().setStyle("rgba(255,0,0,1.0)", "rgba(0,0,0,1.0)", 1.0);
                PanelsField.INSTANCE.getField().moveCursor(tx, ty);
                PanelsField.INSTANCE.getField().circle(3.0);

                // Note: Binary frame data cannot be rendered as a background image in Panels.
                // It is rendered as Braille characters in the telemetry instead.

                telemetryM.update();
            } catch (Exception e) {}
        }
    }

    // --- HELPER CLASSES ---
    /*public static class Renderer {
        private int width, height;
        private boolean[][] pixels;
        public Renderer(int w, int h) { width = w; height = h; pixels = new boolean[h][w]; }
        public void clear() { for (int goalHeight = 0; goalHeight < height; goalHeight++) Arrays.fill(pixels[goalHeight], false); }
        public void setPixel(int x, int goalHeight) { if (x >= 0 && x < width && goalHeight >= 0 && goalHeight < height) pixels[goalHeight][x] = true; }
        private boolean isTrue(int x, int goalHeight) { return (x >= 0 && x < width && goalHeight >= 0 && goalHeight < height) && pixels[goalHeight][x]; }
        private char braille(int x, int goalHeight) {
            int code = 0;
            if (isTrue(x, goalHeight)) code |= 1; if (isTrue(x, goalHeight + 1)) code |= 2;
            if (isTrue(x, goalHeight + 2)) code |= 4; if (isTrue(x + 1, goalHeight)) code |= 8;
            if (isTrue(x + 1, goalHeight + 1)) code |= 16; if (isTrue(x + 1, goalHeight + 2)) code |= 32;
            if (isTrue(x, goalHeight + 3)) code |= 64; if (isTrue(x + 1, goalHeight + 3)) code |= 128;
            return (char) (0x2800 + code);
        }
        public String renderHtml() {
            StringBuilder out = new StringBuilder();
            out.append("<tt><pre style='line-height:1; letter-spacing:0;'>");
            for (int goalHeight = 0; goalHeight < height; goalHeight += 4) {
                for (int x = 0; x < width; x += 2) out.append(braille(x, goalHeight));
                out.append('\n');
            }
            return out.append("</pre></tt>").toString();
        }
    }*/
}