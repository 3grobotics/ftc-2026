package org.firstinspires.ftc.teamcode.pedroPathing;

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
import org.firstinspires.ftc.teamcode.Subsystems.intakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.newintakeSub;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("unused")
@Autonomous(name = "close auto gate ll test", group = "Autonomous")
@Configurable
public class closeAutoGatell extends LinearOpMode {

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

    // Limelight Motif Sum, accessible by both main and background thread
    private int motifSum = 0;

    // Background Threading Variables
    private Thread cycleThread = null;
    private volatile boolean isCycling = false;

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

        Pose pedroStart = new Pose(121.000, 123.000, Math.toRadians(-42), PedroCoordinates.INSTANCE);
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

        int llTagId = 0;
        int llTagId2 = 0;
        double whatItDo = 0;

        // This loop runs ONCE before Start is pressed to detect motifSum
        while (!isStarted() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                List<Integer> seenIds = new ArrayList<>();

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    seenIds.add(fiducial.getFiducialId());
                }

                if (seenIds.size() == 2) {
                    motifSum = seenIds.get(0) + seenIds.get(1);
                } else {
                    motifSum = 0; // Reset if not exactly 2 tags
                }

                if (motifSum == 43) {
                    telemetry.addLine("Motif 21-22 (gpp) detected!");
                } else if (motifSum == 45) {
                    telemetry.addLine("Motif 22-23 (pgp) detected!");
                } else if (motifSum == 44) {
                    telemetry.addLine("Motif 23-21 (ppg) detected!");
                } else if (seenIds.size() == 1) {
                    telemetry.addData("Looking at flat face. Only seeing Tag", seenIds.get(0));
                } else {
                    telemetry.addLine("No valid obelisk face found. Centering turret to 151.5...");
                }
                telemetry.update();
            }
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

        // Main OpMode loop
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            pip.update();
            autonomousPathUpdate();

            h.flywheel1.setVelocity((double) (2800 * 28) / 60);
            h.flywheel2.setVelocity((double) (2800 * 28) / 60);

            frontResult = frontPredominantColorProcessor.getAnalysis();
            backResult = backPredominantColorProcessor.getAnalysis();

            all = (h.topDistSensor.getState() && h.midDistSensor.getState() && h.frontDistSensor.getState());

            // --- PARALLEL CYCLING LOGIC ---
            if (all && !isCycling) {
                boolean shouldStartCycling = false;

                if (motifSum == 44 && !frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) {
                    shouldStartCycling = true;
                } else if (motifSum == 45 && !backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) {
                    shouldStartCycling = true;
                } else if (motifSum == 43 && !(frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE))) {
                    shouldStartCycling = true;
                }

                if (shouldStartCycling) {
                    isCycling = true;
                    cycleThread = new Thread(() -> {
                        try {
                            while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {

                                frontResult = frontPredominantColorProcessor.getAnalysis();
                                backResult = backPredominantColorProcessor.getAnalysis();

                                if (motifSum == 44 && frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) break;
                                if (motifSum == 45 && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) break;
                                if (motifSum == 43 && (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE))) break;

                                h.sickle.setPosition(0.85);
                                h.swingArm.setPosition(0.55);
                                h.gate.setPosition(0.82);
                                Thread.sleep(250);
                                h.intake.setPower(-1);
                                h.indexer.setPower(-1);
                                Thread.sleep(250);
                                h.sickle.setPosition(0.9);
                                Thread.sleep(250);
                                h.gate.setPosition(0.85);
                                h.swingArm.setPosition(0.2);
                                h.intake.setPower(0);
                                Thread.sleep(250);
                                h.intake.setPower(0.5);
                                h.indexer.setPower(-1);
                                Thread.sleep(200);
                                h.indexer.setPower(1);
                                Thread.sleep(100);
                                h.gate.setPosition(0.65);
                                h.indexer.setPower(0);
                                h.swingArm.setPosition(0.55);
                                h.intake.setPower(-1);
                                Thread.sleep(100);
                                h.intake.setPower(0);
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
            panelsTelemetry.debug("Tag ID", motifSum);
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
            drawSolidPathChain(paths.Path1, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path2, "rgba(200,200,200,0.1)"); // Added drawing for Path 2
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
                return paths.Path1;
            case 2:
                return paths.Path2;
            default:
                return null;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1, true);
                setPathState(1);
                actionTimer.resetTimer();
                break;

            case 1:
                if (actionTimer.getElapsedTime() > 3000) {
                    h.gate.setPosition(1);
                    h.indexer.setPower(-1);
                    h.intake.setPower(-1);
                    h.swingArm.setPosition(.9);
                }

                if (actionTimer.getElapsedTime() > 5000 && !follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    setPathState(2);
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                }
                break;

            case 2:
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
        public PathChain Path1, Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(123.000, 118.000),
                                    new Pose(86.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-26.69), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.000, 86.000),
                                    new Pose(78.000, 60.000),
                                    new Pose(105.000, 60.000),
                                    new Pose(128.000, 59.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}