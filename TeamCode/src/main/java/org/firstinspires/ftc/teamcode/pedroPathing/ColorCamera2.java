package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSub;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import static org.firstinspires.ftc.teamcode.pedroPathing.stateTeleOpRedNOTMODDED.farSlope;

@TeleOp(name = "ColorCamera 2")
public class ColorCamera2 extends LinearOpMode {

    public hardwareSubNewBot h;
    public varSub v;
    public Timer swingTimer;

    // --- PTO Control Variables ---
    private boolean ptoButtonWasPressed = false; // For debouncing the gamepad button
    private boolean ptoIsEngaged = false;           // True if PTO is "on", false if "off"

    // For non-blocking timed sequence
    private ElapsedTime ptoDeploymentTimer = new ElapsedTime();
    private PtoDeploymentState currentPtoDeploymentState = PtoDeploymentState.RETRACTED;

    // Enum to define the sequence of servo movements for both engagement and retraction
    enum PtoDeploymentState {
        RETRACTED,                  // Both servos are in final retracted position
        DEPLOYING_R_SERVO,          // Right servo starts moving to engaged
        WAITING_FOR_DEPLOY_DELAY,   // Waiting for delay before left servo deploys
        DEPLOYING_L_SERVO,          // Left servo starts moving to engaged
        ENGAGED,                    // Both servos are in final engaged position
        RETRACTING_L_SERVO,         // Left servo starts moving to retracted
        WAITING_FOR_RETRACT_DELAY,  // Waiting for delay before right servo retracts
        RETRACTING_R_SERVO          // Right servo starts moving to retracted
    }

    // PTO Servo Positions and Delay Constants
    private static final double PTO_R_RETRACTED_POSITION = 1.0;
    private static final double PTO_L_RETRACTED_POSITION = 1.0;
    private static final double PTO_R_ENGAGED_POSITION = 0.25;
    private static final double PTO_L_ENGAGED_POSITION = 0.3;
    private static final long PTO_DEPLOYMENT_DELAY_MS = 500; // Delay between servo movements
    // --- END PTO Control Variables ---

    /**
     * This OpMode illustrates how to use a video source (camera) as a color sensor
     */
    @Override
    public void runOpMode() {

        /* these are the subsystems */
        {
            h = new hardwareSubNewBot(hardwareMap);
            v = new varSub();
        }

        /* these are the timers */
        {
            swingTimer = new Timer();
        }

        /* this is the camera stuff */
        PredominantColorProcessor.Builder frontProcessorBuilder;
        PredominantColorProcessor.Builder middleProcessorBuilder;
        PredominantColorProcessor.Builder backProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        PredominantColorProcessor frontPredominantColorProcessor;
        PredominantColorProcessor middlePredominantColorProcessor;
        PredominantColorProcessor backPredominantColorProcessor;
        VisionPortal myVisionPortal;
        PredominantColorProcessor.Result frontResult;
        PredominantColorProcessor.Result middleResult;
        PredominantColorProcessor.Result backResult;


        // Build a "Color Sensor" vision processor based on the PredominantColorProcessor class.
        frontProcessorBuilder = new PredominantColorProcessor.Builder();
        middleProcessorBuilder = new PredominantColorProcessor.Builder();
        backProcessorBuilder = new PredominantColorProcessor.Builder();
        frontProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(60, 60, 200, 200));
        middleProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(300, 100, 400, 300));
        backProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(700, 100, 800, 300));

        frontProcessorBuilder.setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.BLUE,
                PredominantColorProcessor.Swatch.YELLOW,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.WHITE);
        frontPredominantColorProcessor = frontProcessorBuilder.build();

        middleProcessorBuilder.setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.BLUE,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.WHITE);
        middlePredominantColorProcessor = middleProcessorBuilder.build();

        backProcessorBuilder.setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.BLUE,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.WHITE);
        backPredominantColorProcessor = backProcessorBuilder.build();

        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.addProcessor(frontPredominantColorProcessor);
        myVisionPortalBuilder.addProcessor(middlePredominantColorProcessor);
        myVisionPortalBuilder.addProcessor(backPredominantColorProcessor);
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.setCameraResolution(new Size(800, 448));
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortal = myVisionPortalBuilder.build();

        /* telemetry stuff */
        {
            telemetry.setMsTransmissionInterval(50);
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        }

        /* gamepad debounce variables for turret trim (original) */
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
            frontResult = frontPredominantColorProcessor.getAnalysis();
            middleResult = middlePredominantColorProcessor.getAnalysis();
            backResult = backPredominantColorProcessor.getAnalysis();
            h.topDistSensor.setMode(DigitalChannel.Mode.INPUT);
            h.sickle.setPosition(1);
            /* setting pos*/
            {
                if (gamepad1.dpad_right) {
                    h.gate.setPosition(0.8);
                } else if (gamepad1.dpad_down) {
                    h.gate.setPosition(0.65);
                }
                if (gamepad1.dpad_up) {
                    h.gate.setPosition(1);
                }
            }

            /* intake */
            {
                double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                h.intake.setPower(-intakeCmd);
                h.indexer.setPower(-intakeCmd);

                if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                    h.swingArm.setPosition(.6);
                    h.gate.setPosition(.65);
                }
            }

            /* cycles once */
            {
                if (gamepad1.dpad_left) {
                    h.sickle.setPosition(0.85);
                    h.swingArm.setPosition(0.55);
                    h.gate.setPosition(0.82);
                    sleep(250);
                    h.intake.setPower(-1);
                    h.indexer.setPower(-1);
                    sleep(250);
                    h.sickle.setPosition(0.9);
                    sleep(250);
                    h.gate.setPosition(0.85);
                    h.swingArm.setPosition(0.2);
                    h.intake.setPower(0);
                    sleep(250);
                    h.intake.setPower(0.5);
                    h.indexer.setPower(-1);
                    sleep(200);
                    h.indexer.setPower(1);
                    sleep(100);
                    h.gate.setPosition(0.65);
                    h.indexer.setPower(0);
                    h.swingArm.setPosition(0.55);
                    h.intake.setPower(-1);
                    sleep(100);
                    h.intake.setPower(0);
                }
            }

            /* fire */
            {
                if (gamepad1.right_bumper) {
                    h.gate.setPosition(.97);
                    h.indexer.setPower(-1);
                    h.intake.setPower(-1);
                    h.swingArm.setPosition(.95);
                }
            }

            /* intake till full */
            {
                if (gamepad1.ps) {
                    h.gate.setPosition(0.65);
                    while ((h.topDistSensor.getState() && h.midDistSensor.getState() && h.frontDistSensor.getState()) == false) {
                        h.intake.setPower(-1);
                        h.indexer.setPower(-1);
                    }
                    h.indexer.setPower(0);
                    h.intake.setPower(0);
                }
            }

            // cycles till ppg
            if (gamepad1.right_stick_button) {
                while (opModeIsActive() && !isStopRequested()) {
                    frontResult = frontPredominantColorProcessor.getAnalysis();

                    if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) {
                        break;
                    }

                    h.sickle.setPosition(0.85);
                    h.swingArm.setPosition(0.6);
                    h.gate.setPosition(0.82);
                    sleep(250);
                    h.intake.setPower(-1);
                    h.indexer.setPower(-1);
                    sleep(250);
                    h.sickle.setPosition(0.9);
                    sleep(250);
                    h.gate.setPosition(0.85);
                    h.swingArm.setPosition(0.2);
                    h.intake.setPower(0);
                    sleep(250);
                    h.intake.setPower(0.5);
                    h.indexer.setPower(-1);
                    sleep(200);
                    h.indexer.setPower(1);
                    sleep(100);
                    h.gate.setPosition(0.65);
                    h.indexer.setPower(0);
                    h.swingArm.setPosition(0.6);
                    h.intake.setPower(-1);
                    sleep(100);
                    h.intake.setPower(0);

                    h.swingArm.setPosition(1);
                    sleep(500);

                }

            }

            // cycles till pgp
            if (gamepad1.left_stick_button) {
                while (opModeIsActive() && !isStopRequested()) {
                    middleResult = middlePredominantColorProcessor.getAnalysis();

                    if (middleResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) {
                        break;
                    }

                    h.sickle.setPosition(0.85);
                    h.swingArm.setPosition(0.6);
                    h.gate.setPosition(0.82);
                    sleep(250);
                    h.intake.setPower(-1);
                    h.indexer.setPower(-1);
                    sleep(250);
                    h.sickle.setPosition(0.9);
                    sleep(250);
                    h.gate.setPosition(0.85);
                    h.swingArm.setPosition(0.2);
                    h.intake.setPower(0);
                    sleep(250);
                    h.intake.setPower(0.5);
                    h.indexer.setPower(-1);
                    sleep(200);
                    h.indexer.setPower(1);
                    sleep(100);
                    h.gate.setPosition(0.65);
                    h.indexer.setPower(0);
                    h.swingArm.setPosition(0.6);
                    h.intake.setPower(-1);
                    sleep(100);
                    h.intake.setPower(0);

                    h.swingArm.setPosition(1);
                    sleep(500);
                }

                h.intake.setPower(0);
                h.indexer.setPower(0);
            }

            // cycles till gpp
            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                while (opModeIsActive() && !isStopRequested()) {
                    backResult = middlePredominantColorProcessor.getAnalysis();

                    if (backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) {
                        break;
                    }

                    h.sickle.setPosition(0.85);
                    h.swingArm.setPosition(0.6);
                    h.gate.setPosition(0.82);
                    sleep(250);
                    h.intake.setPower(-1);
                    h.indexer.setPower(-1);
                    sleep(250);
                    h.sickle.setPosition(0.9);
                    sleep(250);
                    h.gate.setPosition(0.85);
                    h.swingArm.setPosition(0.2);
                    h.intake.setPower(0);
                    sleep(250);
                    h.intake.setPower(0.5);
                    h.indexer.setPower(-1);
                    sleep(200);
                    h.indexer.setPower(1);
                    sleep(100);
                    h.gate.setPosition(0.65);
                    h.indexer.setPower(0);
                    h.swingArm.setPosition(0.6);
                    h.intake.setPower(-1);
                    sleep(100);
                    h.intake.setPower(0);

                    h.swingArm.setPosition(1);
                    sleep(500);
                }

                h.intake.setPower(0);
                h.indexer.setPower(0);
            }

            h.pip.update();

            double robotX = h.pip.getPosX(DistanceUnit.INCH);
            double robotY = h.pip.getPosY(DistanceUnit.INCH);

            double xl = v.tx - robotX;
            double yl = v.ty - robotY;
            double hypot = Math.sqrt((xl * xl) + (yl * yl));

            // =========================================================================
            //  TURRET APPLICATION (UNWRAPPING BOUNDARY FIX)
            // =========================================================================
            double angleToGoal = Math.atan2(yl, xl);
            double robotHeading = h.pip.getHeading(AngleUnit.RADIANS);

            // Absolute target angle relative to the robot chassis
            // ADDED + Math.PI here to invert the facing direction by 180 degrees
            double targetTurretRad = angleToGoal - robotHeading + Math.PI;

            // unwrap to [-pi, pi]
            while (targetTurretRad > Math.PI) targetTurretRad -= 2 * Math.PI;
            while (targetTurretRad < -Math.PI) targetTurretRad += 2 * Math.PI;

            // Convert to degrees and apply center offset
            double baseServoDegrees = Math.toDegrees(targetTurretRad) - (337.25 / 2.0);

            // Gamepad trim logic
            left = gamepad2.dpad_left;
            right = gamepad2.dpad_right;

            aa = gamepad2.a;
            bb = gamepad2.b;

            if (left && !prevleft && !right) {
                v.samOffset = Range.clip(v.samOffset + 2.5, -40, 40);
            }
            if (right && !prevright && !left) {
                v.samOffset = Range.clip(v.samOffset - 2.5, -40, 40);
            }

            prevleft = left;
            prevright = right;

            if (aa && !prevaa && !bb) {
                v.samOffset = Range.clip(v.samOffset + 2.5, -40, 40);
            }
            if (bb && !prevbb && !aa) {
                v.samOffset = Range.clip(v.samOffset - 2.5, -40, 40);
            }

            prevaa = aa;
            prevbb = bb;

            double finalServoDegrees = baseServoDegrees + v.visionOffsetDeg + v.samOffset;

            if (gamepad1.ps || gamepad2.ps) {
                h.turret1.setPosition(0.5);
                h.turret2.setPosition(0.5);
            } else {
                h.turret1.setPosition(Math.abs((finalServoDegrees) / 337.25));
                h.turret2.setPosition(Math.abs((finalServoDegrees) / 337.25));
            }

            if (gamepad1.dpad_up) {
                h.pip.setPosition(new Pose2D(DistanceUnit.INCH, -68.02, 36, AngleUnit.DEGREES, 180));
                v.samOffset = 0;
            } else if (gamepad1.dpad_down) {
                h.pip.resetPosAndIMU();
                v.samOffset = 0;
            }


            // =========================================================================
            //  HOOD LINEAR REGRESSION
            // =========================================================================
            {
                double hpos;
                double d1 = 57.5, d2 = 83.5;
                double v1 = .6, v2 = .4;
                double slope = (v2 - v1) / (d2 - d1);
                hpos = v1 + (slope * (hypot - d1));
                h.hood.setPosition(Range.clip(hpos, 0, .6));
            }

            // =========================================================================
            //  FLYWHEEL LINEAR REGRESSION
            // =========================================================================
            double vTarget;
            double d1 = 57.5, d2 = 83.5;
            double v1 = 1550, v2 = 1900;
            double slope = (v2 - v1) / (d2 - d1);
            vTarget = 1400 + (slope * (hypot - d1));
            vTarget = Range.clip(vTarget, 0, 2500);


            /* flywheel turn on/off */
            {
                if (gamepad1.x) {
                    v.var = 1;
                } else if (gamepad1.y) {
                    v.var = 0;
                }

                if (v.var == 1) {
                    h.flywheel1.setVelocity((vTarget * 37.333) / 60);
                    h.flywheel2.setVelocity((vTarget * 37.333) / 60);
                } else {
                    h.flywheel1.setVelocity(0);
                    h.flywheel2.setVelocity(0);
                }
            }

            /*driving */
            {
                v.axial = -gamepad1.left_stick_y;
                v.lateral = gamepad1.left_stick_x;
                v.yawCmd = gamepad1.right_stick_x;

                double fl = v.axial + v.lateral + v.yawCmd;
                double fr = v.axial - v.lateral - v.yawCmd;
                double bl = v.axial - v.lateral + v.yawCmd;
                double br = v.axial + v.lateral - v.yawCmd;

                double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
                h.frontLeft.setPower(fl / max);
                h.frontRight.setPower(fr / max);
                h.backLeft.setPower(bl / max);
                h.backRight.setPower(br / max);
            }

            // --- NEW PTO Control Logic (replaces original commented out and blocking PTO block) ---
            // 1. Gamepad Input Handling (Toggle and Debounce)
            if (gamepad1.options && !ptoButtonWasPressed) {
                ptoIsEngaged = !ptoIsEngaged; // Toggle the main PTO state
                ptoButtonWasPressed = true;   // Mark button as pressed for debouncing

                // Initiate the appropriate sequence (deploy or retract)
                if (ptoIsEngaged) {
                    currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_R_SERVO;
                    ptoDeploymentTimer.reset(); // Start the timer for the deployment sequence
                } else {
                    currentPtoDeploymentState = PtoDeploymentState.RETRACTING_L_SERVO; // Start retraction sequence
                    ptoDeploymentTimer.reset(); // Start the timer for the retraction sequence
                }
            } else if (!gamepad1.options) {
                ptoButtonWasPressed = false; // Reset debounce when button is released
            }

            // 2. Servo Control based on the State Machine (NON-BLOCKING)
            switch (currentPtoDeploymentState) {
                case RETRACTED:
                    // Ensure both servos are in their retracted positions
                    h.ptoR.setPosition(PTO_R_RETRACTED_POSITION);
                    h.ptoL.setPosition(PTO_L_RETRACTED_POSITION);
                    break;

                case DEPLOYING_R_SERVO:
                    // Move the right PTO servo to its engaged position
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    // Transition to the waiting state immediately after commanding the first servo
                    currentPtoDeploymentState = PtoDeploymentState.WAITING_FOR_DEPLOY_DELAY;
                    break;

                case WAITING_FOR_DEPLOY_DELAY:
                    // Keep the right servo commanded to its engaged position
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    // Check if enough time has passed without blocking the robot's main loop
                    if (ptoDeploymentTimer.milliseconds() >= PTO_DEPLOYMENT_DELAY_MS) {
                        currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_L_SERVO;
                    }
                    break;

                case DEPLOYING_L_SERVO:
                    // Move the left PTO servo to its engaged position
                    h.ptoL.setPosition(PTO_L_ENGAGED_POSITION);
                    // Both servos have now been commanded; transition to the ENGAGED state
                    currentPtoDeploymentState = PtoDeploymentState.ENGAGED;
                    break;

                case ENGAGED:
                    // Continuously command both servos to stay in their engaged positions
                    // This is important to hold their position against external forces
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    h.ptoL.setPosition(PTO_L_ENGAGED_POSITION);
                    break;

                case RETRACTING_L_SERVO:
                    // Move the left PTO servo to its retracted position
                    h.ptoL.setPosition(PTO_L_RETRACTED_POSITION);
                    currentPtoDeploymentState = PtoDeploymentState.WAITING_FOR_RETRACT_DELAY;
                    break;

                case WAITING_FOR_RETRACT_DELAY:
                    // Keep the left servo commanded
                    h.ptoL.setPosition(PTO_L_RETRACTED_POSITION);
                    if (ptoDeploymentTimer.milliseconds() >= PTO_DEPLOYMENT_DELAY_MS) {
                        currentPtoDeploymentState = PtoDeploymentState.RETRACTING_R_SERVO;
                    }
                    break;

                case RETRACTING_R_SERVO:
                    // Move the right PTO servo to its retracted position
                    h.ptoR.setPosition(PTO_R_RETRACTED_POSITION);
                    currentPtoDeploymentState = PtoDeploymentState.RETRACTED; // Both servos retracted, sequence complete
                    break;
            }
            // --- END NEW PTO Control Logic ---

            telemetry.addData("Best Match front", frontResult.closestSwatch);
            telemetry.addData("Best Match middle", middleResult.closestSwatch);
            telemetry.addData("Best Match back", backResult.closestSwatch);
            telemetry.addLine("RGB   (" + JavaUtil.formatNumber(frontResult.RGB[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.RGB[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.RGB[2], 3, 0) + ")");
            telemetry.addLine("HSV   (" + JavaUtil.formatNumber(frontResult.HSV[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.HSV[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.HSV[2], 3, 0) + ")");
            telemetry.addLine("YCrCb (" + JavaUtil.formatNumber(frontResult.YCrCb[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.YCrCb[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.YCrCb[2], 3, 0) + ")");

            telemetry.addData("x leg", xl);
            telemetry.addData("y leg", yl);
            telemetry.addData("hypot", hypot);
            telemetry.addData("RPM flywheel 1", "%.3f", ((h.flywheel1.getVelocity() * 60) / 37.333));
            telemetry.addData("RPM flywheel 2", "%.3f", ((h.flywheel2.getVelocity() * 60) / 37.333));
            telemetry.addData("vel ticks flywheel 1", "%.3f", (h.flywheel1.getVelocity()));
            telemetry.addData("vel ticks flywheel 2", "%.3f", (h.flywheel2.getVelocity()));
            telemetry.addData("pip x in", h.pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", h.pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip y in", h.pip.getPosY(DistanceUnit.INCH));
            telemetry.addData("turret1", h.turret1.getPosition());
            telemetry.addData("turret2", h.turret2.getPosition());
            telemetry.addData("finalServoDegrees", finalServoDegrees);
            telemetry.addData("baseServoDegrees", baseServoDegrees);
            telemetry.addData("PTO State", currentPtoDeploymentState.name()); // New telemetry for PTO state


            telemetry.update();
        }
    }
}