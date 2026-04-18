package org.firstinspires.ftc.teamcode.pedroPathing.autos;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.newintakeSub;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("unused")
@Autonomous(name = "12 sorted new bot red auto far", group = "Autonomous")
@Configurable
public class twelvesortednewbotredautofar extends LinearOpMode {

    public Follower follower;

    private int pathState = 0;
    private Paths paths;

    private Timer pathTimer, actionTimer;
    private hardwareSubNewBot h;
    public static newintakeSub intakeSub;

    private Limelight3A limelight;
    private GoBildaPinpointDriver pip;

    // Camera Processors and Vision Portal
    private PredominantColorProcessor frontPredominantColorProcessor;
    private PredominantColorProcessor backPredominantColorProcessor;
    private VisionPortal myVisionPortal;

    // PredominantColorProcessor results, accessible by both main and background thread
    private PredominantColorProcessor.Result frontResult;
    private PredominantColorProcessor.Result backResult;

    // Stores the single Tag ID detected on the flat face
    private int activeTag = 0;

    // Background Threading Variables
    private Thread cycleThread = null;
    private volatile boolean isCycling = false;

    // Enum for the background sorting thread
    public enum IntakeState {
        IDLE,
        SINGLE_NOTE_CYCLE_INIT,
        SINGLE_NOTE_CYCLE_STEP_1,
        SINGLE_NOTE_CYCLE_STEP_2,
        SINGLE_NOTE_CYCLE_STEP_3,
        SINGLE_NOTE_CYCLE_STEP_4,
        SINGLE_NOTE_CYCLE_STEP_5,
        SINGLE_NOTE_CYCLE_STEP_6,
        SINGLE_NOTE_CYCLE_STEP_7
    }

    public static double farSlope = 1750;
    public static double targetX = 144;
    public static double targetY = 144;
    public static double aimOffsetDeg = 0.0;
    public static double samOffset = 5.0;
    public static double visionOffsetDeg = 0.0;
    boolean state5 = false;
    boolean state8 = false;
    boolean state11 = false;

    @Override
    public void runOpMode() {
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        Timer opmodeTimer = new Timer();

        h = new hardwareSubNewBot(hardwareMap);
        intakeSub = new newintakeSub(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(4);
        limelight.start();

        pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pip.setOffsets(48.591, -129.909, DistanceUnit.MM);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        follower = Constants.createFollower(hardwareMap);

        Pose pedroStart = new Pose(100.000, 8.000, Math.toRadians(-90), PedroCoordinates.INSTANCE);
        Pose ftcStart = pedroStart.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        double ftcHeadingDeg = ftcStart.getHeading();

        pip.setPosition(new Pose2D(
                DistanceUnit.INCH,
                ftcStart.getX(),
                ftcStart.getY(),
                AngleUnit.DEGREES,
                ftcHeadingDeg
        ));

        follower.setStartingPose(pedroStart);

        paths = new Paths(follower);

        pathTimer = new Timer();
        actionTimer = new Timer();

        pathTimer.resetTimer();
        actionTimer.resetTimer();
        opmodeTimer.resetTimer();

        h.flywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        h.flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        /* this is the camera stuff */
        PredominantColorProcessor.Builder frontProcessorBuilder;
        PredominantColorProcessor.Builder backProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        frontProcessorBuilder = new PredominantColorProcessor.Builder();
        backProcessorBuilder = new PredominantColorProcessor.Builder();

        frontProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(80, 200, 250, 400));
        backProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(600, 100, 800, 300));

        frontProcessorBuilder.setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.BLUE,
                PredominantColorProcessor.Swatch.YELLOW,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.WHITE);
        frontPredominantColorProcessor = frontProcessorBuilder.build();

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
        myVisionPortalBuilder.addProcessor(backPredominantColorProcessor);
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.setCameraResolution(new Size(800, 448));
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortal = myVisionPortalBuilder.build();

        // This loop runs ONCE before Start is pressed to detect the SINGLE tag and verify colors
        while (!isStarted() && !isStopRequested()) {

            // --- 1. TAG DETECTION ---
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials.size() == 1) {
                    activeTag = fiducials.get(0).getFiducialId();
                    telemetry.addData("Looking at flat face. Detected Tag", activeTag);

                    if (activeTag == 23) {
                        telemetry.addLine("Config: Need PURPLE in BOTH");
                    } else if (activeTag == 22) {
                        telemetry.addLine("Config: Need PURPLE in FRONT, GREEN in BACK");
                    } else if (activeTag == 21) {
                        telemetry.addLine("Config: Need GREEN in FRONT, PURPLE in BACK");
                    } else {
                        telemetry.addLine("Unknown tag ID detected!");
                    }
                } else if (fiducials.size() > 1) {
                    telemetry.addLine("Seeing multiple tags! Please align to a FLAT FACE (1 tag).");
                    activeTag = 0;
                } else {
                    telemetry.addLine("No tags found. Centering turret to 151.5...");
                    activeTag = 0;
                }
            }

            // --- 2. PRE-START COLOR CHECK ---
            frontResult = frontPredominantColorProcessor.getAnalysis();
            backResult = backPredominantColorProcessor.getAnalysis();

            boolean frontSeesTarget = (frontResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) ||
                    (frontResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE);

            boolean backSeesTarget = (backResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) ||
                    (backResult.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE);

            // If neither camera sees a valid color, throw a massive warning
            telemetry.addLine("==============================");
            if (!frontSeesTarget && !backSeesTarget) {
                telemetry.addLine("⚠️ WARNING: CANNOT SEE PURPLE OR GREEN! ⚠️");
            } else {
                telemetry.addData("✅ Front Camera Sees", frontResult.closestSwatch);
                telemetry.addData("✅ Back Camera Sees", backResult.closestSwatch);
            }
            telemetry.addLine("==============================");

            telemetry.update();
        }

        // After OpMode starts
        boolean all = false;
        waitForStart();
        if (isStopRequested()) {
            limelight.stop();
            if (cycleThread != null && cycleThread.isAlive()) cycleThread.interrupt();
            return;
        }

        setPathState(0);
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();

        // Main OpMode loop
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            pip.update();
            autonomousPathUpdate();

            h.flywheel1.setVelocity((double) (3200 * 28) / 60);
            h.flywheel2.setVelocity((double) (3200 * 28) / 60);

            h.hood.setPosition(0);

            h.turret1.setPosition(.58);
            h.turret2.setPosition(.58);

            frontResult = frontPredominantColorProcessor.getAnalysis();
            backResult = backPredominantColorProcessor.getAnalysis();

            all = (h.topDistSensor.getState() && h.midDistSensor.getState() && h.frontDistSensor.getState());

            // --- PARALLEL CYCLING LOGIC ---
            if (all && !isCycling) {
                boolean shouldStartCycling = false;

                // Checking if the current block FAILS the tag's color requirements
                if (activeTag == 23 && !(frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE))) {
                    shouldStartCycling = true; // Needs Both Purple
                } else if (activeTag == 22 && !(frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN))) {
                    shouldStartCycling = true; // Needs Front Purple, Back Green
                } else if (activeTag == 21 && !(frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE))) {
                    shouldStartCycling = true; // Needs Front Green, Back Purple
                }

                if (shouldStartCycling) {
                    isCycling = true;
                    cycleThread = new Thread(() -> {
                        Timer intakeStateTimer = new Timer();
                        IntakeState currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_INIT;

                        try {
                            while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {

                                frontResult = frontPredominantColorProcessor.getAnalysis();
                                backResult = backPredominantColorProcessor.getAnalysis();

                                // Break out of thread if correct block arrives
                                if (activeTag == 23 && (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE))) break;
                                if (activeTag == 22 && (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN))) break;
                                if (activeTag == 21 && (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE))) break;

                                switch (currentIntakeState) {
                                    case IDLE:
                                        // The cycle finished and the bad block is gone.
                                        // Return to kill the thread and let the main loop take over again.
                                        return;
                                    case SINGLE_NOTE_CYCLE_INIT:
                                        h.sickle.setPosition(0.7);
                                        h.swingArm.setPosition(0.55);
                                        h.gate.setPosition(0.82);
                                        intakeStateTimer.resetTimer();
                                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_1;
                                        break;
                                    case SINGLE_NOTE_CYCLE_STEP_1:
                                        if (intakeStateTimer.getElapsedTime() > 250) {
                                            h.intake.setPower(-1);
                                            h.indexer.setPower(-1);
                                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_2;
                                        }
                                        break;
                                    case SINGLE_NOTE_CYCLE_STEP_2:
                                        if (intakeStateTimer.getElapsedTime() > 500) { // Cumulative time
                                            h.sickle.setPosition(.75);
                                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_3;
                                        }
                                        break;
                                    case SINGLE_NOTE_CYCLE_STEP_3:
                                        if (intakeStateTimer.getElapsedTime() > 750) { // Cumulative time
                                            h.gate.setPosition(0.85);
                                            h.swingArm.setPosition(0.2);
                                            h.intake.setPower(0);
                                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_4;
                                        }
                                        break;
                                    case SINGLE_NOTE_CYCLE_STEP_4:
                                        if (intakeStateTimer.getElapsedTime() > 1000) { // Cumulative time
                                            h.intake.setPower(0.5);
                                            h.indexer.setPower(-1);
                                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_5;
                                        }
                                        break;
                                    case SINGLE_NOTE_CYCLE_STEP_5:
                                        if (intakeStateTimer.getElapsedTime() > 1200) { // Cumulative time
                                            h.indexer.setPower(1);
                                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_6;
                                        }
                                        break;
                                    case SINGLE_NOTE_CYCLE_STEP_6:
                                        if (intakeStateTimer.getElapsedTime() > 1300) { // Cumulative time
                                            h.gate.setPosition(0.65);
                                            h.indexer.setPower(0);
                                            h.swingArm.setPosition(0.55);
                                            h.intake.setPower(-1);
                                            currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_7;
                                        }
                                        break;
                                    case SINGLE_NOTE_CYCLE_STEP_7:
                                        if (intakeStateTimer.getElapsedTime() > 1400) { // Cumulative time
                                            h.intake.setPower(0);
                                            currentIntakeState = IntakeState.IDLE;
                                        }
                                        break;
                                }

                                // Small sleep prevents thread from locking up the CPU
                                Thread.sleep(10);
                            }
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        } finally {
                            h.intake.setPower(0);
                            h.indexer.setPower(0);
                            isCycling = false;
                        }
                    });
                    cycleThread.start();
                }
            }
            // --- END PARALLEL CYCLING LOGIC ---

            // Turret targeting calculations
            double robotX = pip.getPosX(DistanceUnit.INCH);
            double robotY = pip.getPosY(DistanceUnit.INCH);
            double xl = targetX - robotX;
            double yl = targetY - robotY;
            double angleToGoal = Math.atan2(yl, xl);
            double robotHeading = pip.getHeading(AngleUnit.RADIANS);
            double calculatedTurretRad = angleToGoal - robotHeading;

            while (calculatedTurretRad > Math.PI) {
                calculatedTurretRad -= 2 * Math.PI;
            }
            while (calculatedTurretRad < -Math.PI) {
                calculatedTurretRad += 2 * Math.PI;
            }

            double finalServoDegrees = Math.toDegrees(calculatedTurretRad) + 202;
            finalServoDegrees += visionOffsetDeg + samOffset + aimOffsetDeg;
            finalServoDegrees = Range.clip(finalServoDegrees, 0, 404);

            Pose pose = follower.getPose();
            drawOverlay(pose, finalServoDegrees);

            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", pose.getX());
            panelsTelemetry.debug("Y", pose.getY());
            panelsTelemetry.debug("Active Tag ID", activeTag);
            panelsTelemetry.update(telemetry);
        }

        // --- FINAL CLEANUP AFTER OPMODE STOPS ---
        limelight.stop();
        if (cycleThread != null && cycleThread.isAlive()) {
            cycleThread.interrupt();
        }
        h.flywheel1.setVelocity(0);
        h.flywheel2.setVelocity(0);
        if (myVisionPortal != null) {
            myVisionPortal.close();
        }
    }

    private void drawOverlay(Pose pose, double turretServoDeg) {
        if (pose == null) return;
        try {
            PanelsField.INSTANCE.getField().setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        } catch (Exception ignored) {}

        if (paths != null) {
            drawSolidPathChain(paths.lineupwithfirst, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.getfirst, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.gotofirefirst, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.getsecond, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.backtofiresecond, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.lineupwithflush, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.getflush, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.backtofireflush, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.park, "rgba(200,200,200,0.1)");

            PathChain active = getActivePathChain();
            if (active != null) {
                drawFadingPathChain(active);
            }
        }

        try {
            PanelsField.INSTANCE.getField().setStyle("rgba(0,255,100,0.2)", "rgba(0,255,100,1.0)", 2.0);
            PanelsField.INSTANCE.getField().moveCursor(126.0, 59.0);
            PanelsField.INSTANCE.getField().circle(2.0);
            PanelsField.INSTANCE.getField().moveCursor(134.0, 57.0);
            PanelsField.INSTANCE.getField().circle(2.0);
        } catch (Exception ignored) {}

        double robotHeadingRad = Math.toRadians(pose.getHeading());
        try {
            PanelsField.INSTANCE.getField().setStyle("rgba(0,0,0,0)", "rgba(255,255,255,1.0)", 2.0);
            PanelsField.INSTANCE.getField().moveCursor(pose.getX(), pose.getY());
            PanelsField.INSTANCE.getField().circle(8.0);

            double hX = pose.getX() + Math.cos(robotHeadingRad) * 12.0;
            double hY = pose.getY() + Math.sin(robotHeadingRad) * 12.0;
            PanelsField.INSTANCE.getField().moveCursor(pose.getX(), pose.getY());
            PanelsField.INSTANCE.getField().line(hX, hY);

            PanelsField.INSTANCE.getField().setStyle("rgba(0,0,0,0)", "rgba(255,0,0,1.0)", 2.0);
            double turretRelativeDeg = turretServoDeg - 151.5;
            double turretGlobalRad = Math.toRadians(pose.getHeading() + turretRelativeDeg);

            double tX = pose.getX() + Math.cos(turretGlobalRad) * 24.0;
            double tY = pose.getY() + Math.sin(turretGlobalRad) * 24.0;
            PanelsField.INSTANCE.getField().moveCursor(pose.getX(), pose.getY());
            PanelsField.INSTANCE.getField().line(tX, tY);
        } catch (Exception ignored) {}

        try {
            PanelsField.INSTANCE.getField().update();
        } catch (Exception ignored) {}
    }

    private void drawSolidPathChain(PathChain chain, String color) {
        if (chain == null) return;
        try {
            PanelsField.INSTANCE.getField().setStyle("rgba(0,0,0,0)", color, 1.0);
            List<Path> pathList = getPathsFromChain(chain);
            for (Path p : pathList) {
                double[] lastP = getPathPointXY(p, 0.0);
                for (int i = 1; i <= 20; i++) {
                    double[] nextP = getPathPointXY(p, i / 20.0);
                    PanelsField.INSTANCE.getField().moveCursor(lastP[0], lastP[1]);
                    PanelsField.INSTANCE.getField().line(nextP[0], nextP[1]);
                    lastP = nextP;
                }
            }
        } catch (Exception ignored) {}
    }

    private void drawFadingPathChain(PathChain chain) {
        if (chain == null) return;
        try {
            List<Path> pathList = getPathsFromChain(chain);
            for (Path p : pathList) {
                double[] lastP = getPathPointXY(p, 0.0);
                for (int i = 1; i <= 20; i++) {
                    double t = i / 20.0;
                    double[] nextP = getPathPointXY(p, t);
                    double opacity = 1.0 - (t * 0.9);
                    PanelsField.INSTANCE.getField().setStyle("rgba(0,0,0,0)", "rgba(255, 0, 90, " + opacity + ")", 3.0);
                    PanelsField.INSTANCE.getField().moveCursor(lastP[0], lastP[1]);
                    PanelsField.INSTANCE.getField().line(nextP[0], nextP[1]);
                    lastP = nextP;
                }
            }
        } catch (Exception ignored) {}
    }

    private List<Path> getPathsFromChain(PathChain chain) {
        List<Path> pathsList = new ArrayList<>();
        try {
            int size = (int) chain.getClass().getMethod("size").invoke(chain);
            for (int i = 0; i < size; i++) {
                pathsList.add((Path) chain.getClass().getMethod("getPath", Integer.TYPE).invoke(chain, i));
            }
        } catch (Exception e) {
            try {
                pathsList = (List<Path>) chain.getClass().getMethod("getPaths").invoke(chain);
            } catch (Exception ignored) {}
        }
        return pathsList;
    }

    private double[] getPathPointXY(Path path, double t) {
        try {
            Object point = path.getClass().getMethod("getPoint", Double.TYPE).invoke(path, t);
            double x = ((Number) point.getClass().getMethod("getX").invoke(point)).doubleValue();
            double y = ((Number) point.getClass().getMethod("getY").invoke(point)).doubleValue();
            return new double[]{x, y};
        } catch (Exception e1) {
            try {
                Object point = path.getClass().getMethod("getPointAt", Double.TYPE).invoke(path, t);
                double x = ((Number) point.getClass().getMethod("getX").invoke(point)).doubleValue();
                double y = ((Number) point.getClass().getMethod("getY").invoke(point)).doubleValue();
                return new double[]{x, y};
            } catch (Exception e2) {
                return new double[]{0.0, 0.0};
            }
        }
    }

    private PathChain getActivePathChain() {
        if (paths == null) return null;
        switch (pathState) {
            case 0:
            case 1:
                return paths.lineupwithfirst;
            case 2:
                return paths.getfirst;
            case 3:
                return paths.gotofirefirst;
            case 4:
                return paths.getsecond;
            case 5:
                return paths.backtofiresecond;
            case 6:
                return paths.lineupwithflush;
            case 7:
                return paths.getflush;
            case 8:
                return paths.backtofireflush;
            case 9:
                return paths.park;
            default:
                return null;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // INITIAL HARDWARE PREP
                // Run your deployment/prep sequence immediately, yielding to sorter
                if (actionTimer.getElapsedTime() > 4000 && !isCycling) {
                    h.gate.setPosition(.97);
                    h.indexer.setPower(-1);
                    h.intake.setPower(-1);
                    h.swingArm.setPosition(.95);
                }

                // Wait a couple of seconds before moving
                if (actionTimer.getElapsedTime() > 8000) {
                    follower.followPath(paths.lineupwithfirst, true);
                    setPathState(1);
                    actionTimer.resetTimer();
                }
                break;

            case 1: // Wait then drive
                // Wait for arrival at lineup
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    // Time-gated hardware action (prep intake), yielding to sorter
                    if (actionTimer.getElapsedTime() > 500 && actionTimer.getElapsedTime() < 2500) {
                        if (!isCycling) {
                            h.gate.setPosition(.65);
                            h.indexer.setPower(-1);
                            h.intake.setPower(-1);
                            h.swingArm.setPosition(.55);
                        }
                    }

                    // Advance to get first
                    if (actionTimer.getElapsedTime() > 2500) {
                        follower.followPath(paths.getfirst, true);
                        setPathState(2);
                        actionTimer.resetTimer();
                        if (!isCycling) {
                            h.intake.setPower(0);
                            h.indexer.setPower(0);
                        }
                    }
                }
                break;

            case 2: // DRIVE IMMEDIATELY
                if (!follower.isBusy()) {
                    follower.followPath(paths.gotofirefirst, false);
                    setPathState(3);
                    actionTimer.resetTimer();
                }
                break;

            case 3: // Pause and act
                // Arrive at fire, stay stationary so the parallel cycleThread can shoot
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 500 && actionTimer.getElapsedTime() < 1500) {
                        if (!isCycling) {
                            h.intake.setPower(0);
                            h.indexer.setPower(0);
                        }
                    }

                    if (actionTimer.getElapsedTime() > 2500) {
                        follower.followPath(paths.getsecond, true);
                        setPathState(4);
                        actionTimer.resetTimer();
                    }
                }
                break;

            case 4: // CONTINUOUS OVERRIDE
                // Continuously assert power OUTSIDE the isBusy check, yielding to sorter
                if (!isCycling) {
                    h.gate.setPosition(1);
                    h.indexer.setPower(-1);
                    h.intake.setPower(-1);
                    h.swingArm.setPosition(.9);
                }

                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if(actionTimer.getElapsedTime() > 2000){
                        state5 = true;
                    }

                    if (state5) {
                        follower.resumePathFollowing();
                        follower.followPath(paths.backtofiresecond, true);
                        setPathState(5);
                        actionTimer.resetTimer();
                        state5 = false; // Reset flag for safety
                    }
                }
                break;

            case 5: // Pause and act (Wait stationary at fire pos)
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    // Wait for cycleThread to shoot, then go to flush
                    if (actionTimer.getElapsedTime() > 2500) {
                        follower.followPath(paths.lineupwithflush, true);
                        setPathState(6);
                        actionTimer.resetTimer();
                    }
                }
                break;

            case 6: // DRIVE IMMEDIATELY
                if (!follower.isBusy()) {
                    follower.followPath(paths.getflush, false);
                    setPathState(7);
                    actionTimer.resetTimer();
                }
                break;

            case 7: // CONTINUOUS OVERRIDE
                // Intake while moving to flush block, yielding to sorter
                if (!isCycling) {
                    h.gate.setPosition(1);
                    h.indexer.setPower(-1);
                    h.intake.setPower(-1);
                    h.swingArm.setPosition(.9);
                }

                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if(actionTimer.getElapsedTime() > 2000){
                        state8 = true;
                    }

                    if (state8) {
                        follower.resumePathFollowing();
                        follower.followPath(paths.backtofireflush, true);
                        setPathState(8);
                        actionTimer.resetTimer();
                        state8 = false;
                    }
                }
                break;

            case 8: // Wait then drive to park
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    // Wait for final cycleThread to shoot
                    if (actionTimer.getElapsedTime() > 2500) {
                        follower.followPath(paths.park, true);
                        setPathState(9);
                        actionTimer.resetTimer();
                    }
                }
                break;

            case 9: // End
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) {
            pathTimer.resetTimer();
        }
    }

    public static class Paths {
        public PathChain lineupwithfirst;
        public PathChain getfirst;
        public PathChain gotofirefirst;
        public PathChain getsecond;
        public PathChain backtofiresecond;
        public PathChain lineupwithflush;
        public PathChain getflush;
        public PathChain backtofireflush;
        public PathChain park;

        public Paths(Follower follower) {
            lineupwithfirst = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(100.000, 8.000),
                                    new Pose(111.000, 19.000),
                                    new Pose(133.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                    .build();

            getfirst = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(133.000, 20.000),
                                    new Pose(136.000, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                    .build();

            gotofirefirst = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(136.000, 8.000),
                                    new Pose(100.000, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .build();

            getsecond = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(100.000, 8.000),
                                    new Pose(91.000, 48.000),
                                    new Pose(100.000, 18.000),
                                    new Pose(69.000, 46.000),
                                    new Pose(115.000, 33.000),
                                    new Pose(127.000, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build();

            backtofiresecond = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(127.000, 35.000),
                                    new Pose(100.000, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                    .build();

            lineupwithflush = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(100.000, 8.000),
                                    new Pose(120.000, 10.000),
                                    new Pose(136.000, 13.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(45))
                    .build();

            getflush = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(136.000, 13.000),
                                    new Pose(136.000, 40.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();

            backtofireflush = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(136.000, 40.000),
                                    new Pose(100.000, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-90))
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.000, 8.000),
                                    new Pose(105.000, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .build();
        }
    }
}