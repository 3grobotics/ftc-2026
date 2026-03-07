package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSub;
import org.firstinspires.ftc.teamcode.Subsystems.intakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.turretSub;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("unused")
@Autonomous(name = "close auto gate", group = "Autonomous")
@Configurable // Panels
public class closeAutoGate extends LinearOpMode {

    public Follower follower;                   // Pedro follower

    private int pathState = 0;                  // state machine state
    private Paths paths;                        // path definitions

    private Timer pathTimer, actionTimer;
    public static turretSub turretSub;
    public static intakeSub intakeSub;
    public static flywheelSub flywheelSub;
    private hardwareSub hSub;

    public static double farSlope = 1750;
    public static double targetX = 144;         // goal X (inches)
    public static double targetY = 144;          // goal Y (inches)
    public static double aimOffsetDeg = 0.0;    // extra global trim
    public static double samOffset = 5.0;       // teleop-style trim
    public static double visionOffsetDeg = 0.0; // future LL correction if desired

    @Override
    public void runOpMode() {
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        Timer opmodeTimer = new Timer();

        // Map subsystems first (safe for callbacks and hardware availability)
        hSub = new hardwareSub(hardwareMap);
        turretSub = new turretSub(hardwareMap);
        intakeSub = new intakeSub(hardwareMap);
        flywheelSub = new flywheelSub(hardwareMap);

        // Pinpoint first
        GoBildaPinpointDriver pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pip.setOffsets(-42, -90, DistanceUnit.MM);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Pedro follower
        follower = Constants.createFollower(hardwareMap);

        // Pedro start pose (YOUR PEDRO BUILD USES DEGREES)
        Pose pedroStart = new Pose(126, 120, Math.toRadians(36), PedroCoordinates.INSTANCE);

        // Convert Pedro -> FTC frame for Pinpoint
        Pose ftcStart = pedroStart.getAsCoordinateSystem(FTCCoordinates.INSTANCE);

        // If heading looks wrong, try Math.toDegrees(ftcStart.getHeading()) instead.
        double ftcHeadingDeg = ftcStart.getHeading();

        pip.setPosition(new Pose2D(
                DistanceUnit.INCH,
                ftcStart.getX(),
                ftcStart.getY(),
                AngleUnit.DEGREES,
                ftcHeadingDeg
        ));

        follower.setStartingPose(pedroStart);

        // Build paths AFTER subsystems exist (your callbacks use intakeSub static methods)
        paths = new Paths(follower);

        pathTimer = new Timer();
        actionTimer = new Timer();

        pathTimer.resetTimer();
        actionTimer.resetTimer();
        opmodeTimer.resetTimer();

        hSub.flywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        hSub.flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("PedroStartX", pedroStart.getX());
        panelsTelemetry.debug("PedroStartY", pedroStart.getY());
        panelsTelemetry.debug("PedroStartH", pedroStart.getHeading());
        panelsTelemetry.debug("FTCStartX", ftcStart.getX());
        panelsTelemetry.debug("FTCStartY", ftcStart.getY());
        panelsTelemetry.debug("FTCStartH(deg?)", ftcHeadingDeg);
        panelsTelemetry.update(telemetry);

        while (!isStarted() && !isStopRequested()) {
            follower.update();

            // Update subsystems
            turretSub.loop();
            intakeSub.loop();
            flywheelSub.loop();

            // Update Pinpoint in init too (handy for checking pose)
            pip.update();

            Pose pose = follower.getPose();

            // Draw overlay during init (turret centered at 151.5)
            drawOverlay(pose, 151.5);

            panelsTelemetry.debug("Status", "Init Loop");
            panelsTelemetry.debug("X", pose.getX());
            panelsTelemetry.debug("Y", pose.getY());
            panelsTelemetry.debug("Heading", pose.getHeading());
            panelsTelemetry.debug("PinpointX", pip.getPosX(DistanceUnit.INCH));
            panelsTelemetry.debug("PinpointY", pip.getPosY(DistanceUnit.INCH));
            panelsTelemetry.update(telemetry);
        }

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        setPathState(0);
        opmodeTimer.resetTimer();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            turretSub.loop();
            intakeSub.loop();
            flywheelSub.loop();

            // =========================
            // AUTO AIM / HOOD / VELOCITY
            // =========================
            pip.update();

            double robotX = pip.getPosX(DistanceUnit.INCH);
            double robotY = pip.getPosY(DistanceUnit.INCH);

            double xl = targetX - robotX;
            double yl = targetY - robotY;
            double distanceToTarget = Math.sqrt((xl * xl) + (yl * yl));

            // =========================================================================
            //  TURRET APPLICATION
            // =========================================================================
            double angleToGoal = Math.atan2(yl, xl);
            double robotHeading = pip.getHeading(AngleUnit.RADIANS);

            // Base Turret Angle Calculation
            double calculatedTurretRad = angleToGoal - robotHeading;

            // 1. Wrap angle (-180 to 180)
            while (calculatedTurretRad > Math.PI) calculatedTurretRad -= 2 * Math.PI;
            while (calculatedTurretRad < -Math.PI) calculatedTurretRad += 2 * Math.PI;

            // 2. Convert to Degrees and apply center offset
            double finalServoDegrees = Math.toDegrees(calculatedTurretRad) + 151.5;

            // 4. Combine Physics + Vision
            finalServoDegrees += visionOffsetDeg + samOffset + aimOffsetDeg;

            // 5. Clamp and Set
            finalServoDegrees = Range.clip(finalServoDegrees, 0, 303);
            double turretPos = finalServoDegrees / 303.0;
            hSub.turret1.setPosition(turretPos);
            hSub.turret2.setPosition(turretPos);

            // =========================================================================
            //  HOOD LINEAR REGRESSION
            // =========================================================================
            double hoodPosition;
            if (distanceToTarget < 130) {
                // Zone: Close
                double d1 = 57.5, d2 = 97.3;
                double v1 = 0.5, v2 = 0.7;
                double slope = (v2 - v1) / (d2 - d1);
                hoodPosition = v1 + (slope * (distanceToTarget - d1));
            } else {
                // Zone: Far
                double d1 = 136.5, d2 = 158.1;
                double v1 = 0.7, v2 = 1.0;
                double slope = (v2 - v1) / (d2 - d1);
                hoodPosition = v1 + (slope * (distanceToTarget - d1));
            }
            hSub.hood.setPosition(Range.clip(hoodPosition, 0.5, 1.0));

            // =========================================================================
            //  FLYWHEEL LINEAR REGRESSION
            // =========================================================================
            double vTarget = Range.clip(calculateVTarget(distanceToTarget), 0, 2500);

            hSub.flywheel1.setVelocity(vTarget);
            hSub.flywheel2.setVelocity(vTarget);

            autonomousPathUpdate();

            Pose pose = follower.getPose();



            // Draw interactive overlay
            drawOverlay(pose, finalServoDegrees);

            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", pose.getX());
            panelsTelemetry.debug("Y", pose.getY());
            panelsTelemetry.debug("Heading", pose.getHeading());
            panelsTelemetry.debug("Busy", follower.isBusy());
            panelsTelemetry.debug("flywheel1 velocity", hSub.flywheel1.getVelocity());
            panelsTelemetry.debug("actionTimer", actionTimer.getElapsedTime());

            panelsTelemetry.debug("AimDist", distanceToTarget);
            panelsTelemetry.debug("TurretDeg", finalServoDegrees);
            panelsTelemetry.debug("TurretPos", turretPos);
            panelsTelemetry.debug("HoodPos", hSub.hood.getPosition());
            panelsTelemetry.debug("vTarget", vTarget);
            panelsTelemetry.debug("PinpointX", robotX);
            panelsTelemetry.debug("PinpointY", robotY);
            panelsTelemetry.debug("OpModeTime", opmodeTimer.getElapsedTime());
            panelsTelemetry.debug("pose", follower.getPose());

            panelsTelemetry.update(telemetry);
        }
    }

    private void drawOverlay(Pose pose, double turretServoDeg) {
        if (pose == null) return;

        // 1. Choose Coordinates System (Directly targets the correct private/public Kotlin layout)
        try {
            PanelsField.INSTANCE.getField().setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        } catch(Exception ignored) {}

        // 2. Draw Faded Background Paths
        if (paths != null) {
            drawSolidPathChain(paths.Path1, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path2, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path3, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path4, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path5, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path6, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path7, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path8, "rgba(200,200,200,0.1)");

            // 3. Draw Active Path with Progress Fade Effect
            PathChain active = getActivePathChain();
            if (active != null) {
                drawFadingPathChain(active);
            }
        }

        // 4. Intake Event Markers (Kotlin style circles)
        try {
            PanelsField.INSTANCE.getField().setStyle("rgba(0,255,100,0.2)", "rgba(0,255,100,1.0)", 2.0);
            PanelsField.INSTANCE.getField().moveCursor(126.0, 59.0); // Path 1 callback location
            PanelsField.INSTANCE.getField().circle(2.0);
            PanelsField.INSTANCE.getField().moveCursor(134.0, 57.0); // Common intake location
            PanelsField.INSTANCE.getField().circle(2.0);
        } catch (Exception ignored) {}

        // 5. Draw Robot Chassis & Heading Ray
        double robotHeadingRad = Math.toRadians(pose.getHeading());
        try {
            PanelsField.INSTANCE.getField().setStyle("rgba(0,0,0,0)", "rgba(255,255,255,1.0)", 2.0);
            PanelsField.INSTANCE.getField().moveCursor(pose.getX(), pose.getY());
            PanelsField.INSTANCE.getField().circle(8.0); // Approximate robot radius

            // Heading Ray (White)
            double hX = pose.getX() + Math.cos(robotHeadingRad) * 12.0;
            double hY = pose.getY() + Math.sin(robotHeadingRad) * 12.0;
            PanelsField.INSTANCE.getField().moveCursor(pose.getX(), pose.getY());
            PanelsField.INSTANCE.getField().line(hX, hY);

            // 6. Draw Turret Aim Ray (Red)
            PanelsField.INSTANCE.getField().setStyle("rgba(0,0,0,0)", "rgba(255,0,0,1.0)", 2.0);
            double turretRelativeDeg = turretServoDeg - 151.5;
            double turretGlobalRad = Math.toRadians(pose.getHeading() + turretRelativeDeg);

            double tX = pose.getX() + Math.cos(turretGlobalRad) * 24.0;
            double tY = pose.getY() + Math.sin(turretGlobalRad) * 24.0;
            PanelsField.INSTANCE.getField().moveCursor(pose.getX(), pose.getY());
            PanelsField.INSTANCE.getField().line(tX, tY);
        } catch (Exception ignored) {}

        // 7. CRITICAL: Update Drawing
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

                    // Fade active path from opacity 1.0 down to 0.1 towards the end
                    double opacity = 1.0 - (t * 0.9);
                    PanelsField.INSTANCE.getField().setStyle("rgba(0,0,0,0)", "rgba(255, 0, 90, " + opacity + ")", 3.0);

                    PanelsField.INSTANCE.getField().moveCursor(lastP[0], lastP[1]);
                    PanelsField.INSTANCE.getField().line(nextP[0], nextP[1]);
                    lastP = nextP;
                }
            }
        } catch (Exception ignored) {}
    }

    // Safely handles Pedro Path API changes regarding extracting paths from a chain
    private List<Path> getPathsFromChain(PathChain chain) {
        List<Path> pathsList = new ArrayList<>();
        try {
            int size = (int) chain.getClass().getMethod("size").invoke(chain);
            for(int i = 0; i < size; i++) {
                pathsList.add((Path) chain.getClass().getMethod("getPath", Integer.TYPE).invoke(chain, i));
            }
        } catch(Exception e) {
            try {
                pathsList = (List<Path>) chain.getClass().getMethod("getPaths").invoke(chain);
            } catch (Exception ignored) {}
        }
        return pathsList;
    }

    // Safely retrieves X/Y coordinates using reflection to prevent version mismatch crashes
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
            case 1: return paths.Path1;
            case 2: return paths.Path2;
            case 3: return paths.Path3;
            case 4: return paths.Path4;
            case 5: return paths.Path5;
            case 6: return paths.Path6;
            case 7: return paths.Path7;
            case 8:
            case 9: return paths.Path8;
            default: return null;
        }
    }

    private double calculateVTarget(double distance) {
        if (distance < 130) {
            // Zone: Close
            double d1 = 57.5, d2 = 97.3;
            double v1 = 1310, v2 = 1660;
            double slope = (v2 - v1) / (d2 - d1);
            return 1150 + (slope * (distance - d1));
        } else {
            // Zone: Far
            double d1 = 136.5, d2 = 158.1;
            double v1 = 1900, v2 = 1940;
            double slope = (v2 - v1) / (d2 - d1);
            return farSlope + (slope * (distance - d1));
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                    follower.followPath(paths.Path1, true); // (aka RESET)
                    setPathState(1);
                    actionTimer.resetTimer();
                break;
            case 1:
                actionTimer.getElapsedTime();

                if (actionTimer.getElapsedTime() > 4000 && actionTimer.getElapsedTime() < 6000) {
                    hSub.intake.setPower(1);
                    hSub.gecko.setPower(-1);
                }

                if (actionTimer.getElapsedTime() > 6000) {
                    follower.resumePathFollowing();
                    actionTimer.resetTimer();
                    setPathState(2);
                    hSub.intake.setPower(0);
                    hSub.gecko.setPower(0);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                        hSub.intake.setPower(1);
                        hSub.gecko.setPower(1);
                    }

                    if (actionTimer.getElapsedTime() > 2000) {
                        follower.resumePathFollowing();
                        follower.followPath(paths.Path2, true);
                        setPathState(3);
                        actionTimer.resetTimer();
                    }
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                        hSub.intake.setPower(1);
                        hSub.gecko.setPower(-1);
                    }

                    if (actionTimer.getElapsedTime() > 3000) {
                        follower.resumePathFollowing();
                        follower.followPath(paths.Path3, .8,true);
                        setPathState(4);
                        actionTimer.resetTimer();
                        hSub.intake.setPower(0);
                        hSub.gecko.setPower(0);
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                        hSub.intake.setPower(1);
                        hSub.gecko.setPower(1);
                    }

                    if (actionTimer.getElapsedTime() > 2000) {
                        follower.resumePathFollowing();
                        follower.followPath(paths.Path4, true);
                        setPathState(5);
                        actionTimer.resetTimer();
                    }
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                        hSub.intake.setPower(1);
                        hSub.gecko.setPower(-1);
                    }

                    if (actionTimer.getElapsedTime() > 3000) {
                        follower.resumePathFollowing();
                        follower.followPath(paths.Path5, .8,true);
                        setPathState(6);
                        actionTimer.resetTimer();
                        hSub.intake.setPower(0);
                        hSub.gecko.setPower(0);
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                        hSub.intake.setPower(1);
                        hSub.gecko.setPower(1);
                    }

                    if (actionTimer.getElapsedTime() > 2000) {
                        follower.resumePathFollowing();
                        follower.followPath(paths.Path6, true);
                        setPathState(7);
                        actionTimer.resetTimer();
                    }
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                        hSub.intake.setPower(1);
                        hSub.gecko.setPower(-1);
                    }

                    if (actionTimer.getElapsedTime() > 3000) {
                        follower.resumePathFollowing();
                        follower.followPath(paths.Path7, .8,true);
                        setPathState(8);
                        actionTimer.resetTimer();
                        hSub.intake.setPower(0);
                        hSub.gecko.setPower(0);
                    }
                }
                break;

            case 8:
                // Wait for Path 7 to completely finish driving back to base (96, 8)
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    follower.followPath(paths.Path8, true); // (aka RESET)
                    setPathState(9);
                    actionTimer.resetTimer();
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                        hSub.intake.setPower(1);
                        hSub.gecko.setPower(-1);
                    }

                    if (actionTimer.getElapsedTime() > 3000) {
                        follower.resumePathFollowing();
                        follower.followPath(paths.Path9, true);
                        setPathState(10);
                        actionTimer.resetTimer();
                        hSub.intake.setPower(0);
                        hSub.gecko.setPower(0);
                    }
                }
                break;

            case 10:
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
        public PathChain Path9;


        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(126.000, 120.000),
                                    new Pose(57.000, 86.000),
                                    new Pose(81.000, 49.000),
                                    new Pose(96.000, 57.000),
                                    new Pose(102.000, 61.000),
                                    new Pose(126.000, 59.000)
                            )
                    )
                    .addParametricCallback(.5, intakeSub.IntakeInGeckoOut())
                    .addParametricCallback(1, intakeSub.intakeAndGeckoStop())
                    .addPoseCallback(new Pose(86, 86, Math.toRadians(36)), () -> follower.pausePathFollowing(), .3)
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(126.000, 59.000),
                                    new Pose(86.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.000, 86.000),
                                    new Pose(105.000, 56.000),
                                    new Pose(134.000, 74.000),
                                    new Pose(127.000, 68.000),
                                    new Pose(124.000, 67.000),
                                    new Pose(125.000, 66.000),
                                    new Pose(134.000, 57.000)
                            )
                    )
                    // intake at end of Path3
                    .addPoseCallback(new Pose(134.000, 57.000, Math.toRadians(45)), intakeSub.IntakeInGeckoOut(), 0.5)
                    //.addParametricCallback(1, intakeSub.intakeAndGeckoStop())
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();


            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(134.000, 57.000),
                                    new Pose(86.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.000, 86.000),
                                    new Pose(105.000, 56.000),
                                    new Pose(134.000, 74.000),
                                    new Pose(127.000, 68.000),
                                    new Pose(124.000, 67.000),
                                    new Pose(125.000, 66.000),
                                    new Pose(134.000, 57.000)
                            )
                    )
                    // intake at end of Path5
                    .addPoseCallback(new Pose(134.000, 57.000, Math.toRadians(45)), intakeSub.IntakeInGeckoOut(), 0.5)
                    //.addParametricCallback(0.99, intakeSub.intakeAndGeckoStop())
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(134.000, 57.000),
                                    new Pose(86.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.000, 86.000),
                                    new Pose(105.000, 56.000),
                                    new Pose(134.000, 74.000),
                                    new Pose(127.000, 68.000),
                                    new Pose(124.000, 67.000),
                                    new Pose(125.000, 66.000),
                                    new Pose(134.000, 57.000)
                            )
                    )
                    // intake at end of Path7
                    .addPoseCallback(new Pose(134.000, 57.000, Math.toRadians(45)), intakeSub.IntakeInGeckoOut(), 0.5)
                    //.addParametricCallback(0.99, intakeSub.intakeAndGeckoStop())
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(134.000, 57.000),
                                    new Pose(86.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.000, 86.000),
                                    new Pose(95.000, 67.000),
                                    new Pose(121.000, 71.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}