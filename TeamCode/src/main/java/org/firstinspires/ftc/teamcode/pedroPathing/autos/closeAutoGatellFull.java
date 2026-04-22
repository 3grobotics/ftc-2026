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
import org.firstinspires.ftc.teamcode.Subsystems.intakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.newintakeSub;
import org.firstinspires.ftc.teamcode.pedroPathing.ColorCamera2testing;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("unused")
@Autonomous(name = "close auto gate ll full", group = "Autonomous")
@Configurable
public class closeAutoGatellFull extends LinearOpMode {

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
    private Timer cycleTimer = new Timer();
    private volatile boolean isCycling = false;


    boolean state5 = false;
    boolean state8 = false;
    boolean state11 = false;

    double hpos;
    double robotX;
    double robotY;
    double xl;
    double yl;
    double hypot;

    double vx;
    double vy;
    double vTarget                   = 0;
    double tx = -72; // Target X
    double ty = 72;  // Target Y
    double t = 1;
     boolean all = false;
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

        paths = new Paths(follower, h);

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

            vy                         = h.pip.getVelY(DistanceUnit.INCH);
            vx                         = h.pip.getVelX(DistanceUnit.INCH);


            tx                         = -72 - (vx * t); // 72 or tx maybe
            ty                         =  72 - (vy * t); // 72 or tx maybe


            h.pip.update(); // leftdate position first!

            robotX                     = h.pip.getPosX(DistanceUnit.INCH);
            robotY                     = h.pip.getPosY(DistanceUnit.INCH);

            xl                         = tx - robotX;
            yl                         = ty - robotY;
            hypot                      = Math.sqrt((xl * xl) + (yl * yl));


            if ( hypot < 89){
                t                      =  0.004 * hypot + 0.468;
            } else if ( hypot > 89 && hypot < 121)  {
                t                      =  0.833;
            } else if ( hypot > 121) {
                t                      = -0.004 * hypot + 1.317;
            }


            // =========================================================================
            //  TURRET APPLICATION (UNWRAPPING BOUNDARY FIX)
            // =========================================================================
            double angleToGoal = Math.atan2(yl, xl);
            double robotHeading = h.pip.getHeading(AngleUnit.RADIANS);

            // Absolute target angle relative to the robot chassis
            // Removed + Math.PI to correct the 180-degree offset
            double targetTurretRad = angleToGoal - robotHeading;

            // unwrap to [-pi, pi]
            while (targetTurretRad > Math.PI) targetTurretRad -= 2 * Math.PI;
            while (targetTurretRad < -Math.PI) targetTurretRad += 2 * Math.PI;

            // Convert to degrees and apply center offset
            double finalServoDegrees = Math.toDegrees(targetTurretRad) - (314.6112145 / 2.0);


            h.turret1.setPosition(Math.abs((finalServoDegrees) / 314.6112145));
            h.turret2.setPosition(Math.abs((finalServoDegrees) / 314.6112145));





            // =========================================================================
            //  FLYWHEEL LINEAR REGRESSION
            // =========================================================================

            if (hypot < 75) {

                vTarget = 9.938 * hypot + 1022.646;
                /*double d1 = 53, d2 = 95;
                double v1 = 1450, v2 = 1960;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = 1200 + (slope * (hypot - d1));*/
            } else if(hypot > 75 && hypot < 96.3){
                vTarget = 12.066 * hypot + 863.05;
                // Zone: Far
                /*double d1 = 132.5, d2 = 158;
                double v1 = 2330, v2 = 2430;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = 2300 + (slope * (hypot - d1));*/
            } else if(hypot > 96.3 && hypot < 129) {
                vTarget = 9.817 * hypot + 1079.623;
            } else if(hypot > 151) {
                vTarget = 11.727 * hypot + 833.217;
            }


            vTarget = Range.clip(vTarget, 0, 6000); // Adjust max based on motor

                    h.flywheel1.setVelocity(((vTarget) * 37.33) / 60);
                    h.flywheel2.setVelocity(((vTarget) * 37.33) / 60);

            // =========================================================================
            //  HOOD LINEAR REGRESSION
            // =========================================================================

            if (hypot < 75) {

                hpos = -0.006 * hypot + 1.053;
                    /*double d1 = 53, d2 = 95;
                    double v1 = .7, v2 = .2;
                    double slope = (v2 - v1) / (d2 - d1);
                    hpos = v1 + (slope * (hypot - d1));*/
            } else if(hypot > 75 && hypot < 96.3){
                hpos = -0.005 * hypot + 0.975;
                // Zone: Far
                    /*double d1 = 132.5, d2 = 158;
                    double v1 = 0, v2 = 0;
                    double slope = (v2 - v1) / (d2 - d1);
                    hpos = v1 + (slope * (hypot - d1));*/
            } else if(hypot > 96.3 && hypot < 129){
                hpos = -0.015 * hypot + 1.944;
            } else if(hypot > 151){
                hpos = 0;
            }


            hpos = Range.clip(hpos, 0, .7);
            h.hood.setPosition(hpos);
            // h.hood.setPosition(hpos - velDiff);


            frontResult = frontPredominantColorProcessor.getAnalysis();
            backResult = backPredominantColorProcessor.getAnalysis();

            all = (h.topDistSensor.getState() && h.midDistSensor.getState() && h.frontDistSensor.getState());

            // --- PARALLEL CYCLING LOGIC ---
            if (!isCycling) {
                boolean shouldStopCycling = false;

                if (motifSum == 44 && frontResult != null && frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) {
                    shouldStopCycling = true;
                } else if (motifSum == 45 && backResult != null && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) {
                    shouldStopCycling = true;
                } else if (motifSum == 43 && frontResult != null && backResult != null && (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE))) {
                    shouldStopCycling = true;
                }

                if (!shouldStopCycling && all) {
                    isCycling = true;
                    cycleThread = new Thread(() -> {
                        try {
                            int cycleStep = 0;
                            cycleTimer.resetTimer();

                            while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
                                frontResult = frontPredominantColorProcessor.getAnalysis();
                                backResult = backPredominantColorProcessor.getAnalysis();

                                // Check to break cycle
                                boolean stopCondition = (motifSum == 44 && frontResult != null && frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) ||
                                                       (motifSum == 45 && backResult != null && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) ||
                                                       (motifSum == 43 && frontResult != null && backResult != null && (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE)));
                                if (stopCondition || !all) break;

                                switch (cycleStep) {
                                    case 0: // Reset positions
                                        h.swingArm.setPosition(1);
                                        h.sickle.setPosition(1);
                                        h.gate.setPosition(0.65);
                                        h.intake.setPower(0);
                                        h.indexer.setPower(0);
                                        if (cycleTimer.getElapsedTime() > 500) {
                                            cycleStep = 1;
                                            cycleTimer.resetTimer();
                                        }
                                        break;
                                    case 1: // Prep intake
                                        h.sickle.setPosition(0.8);
                                        h.swingArm.setPosition(0.6);
                                        h.gate.setPosition(0.82);
                                        if (cycleTimer.getElapsedTime() > 250) {
                                            cycleStep = 2;
                                            cycleTimer.resetTimer();
                                        }
                                        break;
                                    case 2: // Intake on
                                        h.intake.setPower(-1);
                                        h.indexer.setPower(-1);
                                        if (cycleTimer.getElapsedTime() > 250) {
                                            cycleStep = 3;
                                            cycleTimer.resetTimer();
                                        }
                                        break;
                                    case 3: // Sickle adjust
                                        h.sickle.setPosition(0.75);
                                        if (cycleTimer.getElapsedTime() > 250) {
                                            cycleStep = 4;
                                            cycleTimer.resetTimer();
                                        }
                                        break;
                                    case 4: // Gate and Swing adjust
                                        h.gate.setPosition(0.85);
                                        h.swingArm.setPosition(0.2);
                                        h.intake.setPower(0);
                                        if (cycleTimer.getElapsedTime() > 250) {
                                            cycleStep = 5;
                                            cycleTimer.resetTimer();
                                        }
                                        break;
                                    case 5: // Transfer
                                        h.intake.setPower(0.5);
                                        h.indexer.setPower(-1);
                                        if (cycleTimer.getElapsedTime() > 200) {
                                            cycleStep = 6;
                                            cycleTimer.resetTimer();
                                        }
                                        break;
                                    case 6: // Indexer bump
                                        h.indexer.setPower(1);
                                        if (cycleTimer.getElapsedTime() > 100) {
                                            cycleStep = 7;
                                            cycleTimer.resetTimer();
                                        }
                                        break;
                                    case 7: // Clear and reset
                                        h.gate.setPosition(0.65);
                                        h.indexer.setPower(0);
                                        h.swingArm.setPosition(0.6);
                                        h.intake.setPower(-1);
                                        if (cycleTimer.getElapsedTime() > 100) {
                                            cycleStep = 8;
                                            cycleTimer.resetTimer();
                                        }
                                        break;
                                    case 8: // Finish
                                        h.intake.setPower(0);
                                        cycleStep = 0; // Loop back to start of cycle
                                        cycleTimer.resetTimer();
                                        break;
                                }

                                // Use a tiny sleep to prevent pegged CPU usage in the background thread
                                // without blocking the hardware update frequency.
                                Thread.sleep(10);
                            }
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        } finally {
                            isCycling = false;
                        }
                    });
                    cycleThread.setPriority(Thread.NORM_PRIORITY);
                    cycleThread.start();
                }
            }


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
            drawSolidPathChain(paths.Path2, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path3, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path4, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path5, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path6, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path7, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path8, "rgba(200,200,200,0.1)");

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
            case 3:
                return paths.Path3;
            case 4:
                return paths.Path4;
            case 5:
                return paths.Path5;
            case 6:
                return paths.Path6;
            case 7:
                return paths.Path7;
            case 8:
                return paths.Path8;
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

                if (actionTimer.getElapsedTime() > 5000 && !follower.isBusy()) {
                    follower.followPath(paths.RESET, true);
                    setPathState(8);
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                }
                break;

            /*case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    follower.followPath(paths.Path3, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    follower.followPath(paths.Path4, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    follower.followPath(paths.Path6, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    follower.followPath(paths.Path7, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    follower.followPath(paths.Path8, true);
                    setPathState(8);
                }
                break;*/

            case 8:
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
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain RESET;

        public Paths(Follower follower, hardwareSubNewBot h) {
                Path1 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(125.000, 118.000),
                                        new Pose(57.000, 86.000),
                                        new Pose(83.012, 90.814),
                                        new Pose(96.000, 57.000),
                                        new Pose(102.000, 63.000),
                                        new Pose(126.000, 63.000)
                                )
                        )

                    .addTemporalCallback
                        (1000,
                        () -> {
                        h.gate.setPosition(1);
                        h.indexer.setPower(-1);
                        h.intake.setPower(-1);
                        h.swingArm.setPosition(.9);
                        })
                    .setLinearHeadingInterpolation(Math.toRadians(-23), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(126.000, 59.000),
                                    new Pose(116.000, 71.000),
                                    new Pose(130.000, 70.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.000, 70.000),
                                    new Pose(82.000, 69.000),
                                    new Pose(86.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(86.000, 86.000),
                                    new Pose(126.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(126.000, 86.000),
                                    new Pose(86.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.000, 86.000),
                                    new Pose(91.000, 26.000),
                                    new Pose(126.000, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(126.000, 35.000),
                                    new Pose(86.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(307))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(86.000, 86.000),
                                    new Pose(86.000, 103.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(307), Math.toRadians(270))
                    .build();

            RESET = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(126.000, 63.000),
                                    new Pose(57.000, 86.000),
                                    new Pose(83.012, 90.814),
                                    new Pose(96.000, 57.000),
                                    new Pose(102.000, 63.000),
                                    new Pose(125.000, 118.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-23))
                    .build();
        }
    }
}