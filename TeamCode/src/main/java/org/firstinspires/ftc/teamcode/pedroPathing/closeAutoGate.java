package org.firstinspires.ftc.teamcode.pedroPathing;

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
    public static double targetY = 144;         // goal Y (inches)
    public static double aimOffsetDeg = 0.0;    // extra global trim
    public static double samOffset = 5.0;       // teleop-style trim
    public static double visionOffsetDeg = 0.0; // future LL correction if desired
    boolean state5 = false;
    boolean state8 = false;
    boolean state11 = false;

    @Override
    public void runOpMode() {
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        Timer opmodeTimer = new Timer();

        // Map subsystems first
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

        // Pedro start pose
        Pose pedroStart = new Pose(126, 120, Math.toRadians(36), PedroCoordinates.INSTANCE);
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

        hSub.flywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        hSub.flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        while (!isStarted() && !isStopRequested()) {
            follower.update();

            turretSub.loop();
            intakeSub.loop();
            flywheelSub.loop();

            pip.update();

            Pose pose = follower.getPose();
            drawOverlay(pose, 151.5);

            panelsTelemetry.debug("Status", "Init Loop");
            panelsTelemetry.debug("X", pose.getX());
            panelsTelemetry.debug("Y", pose.getY());
            panelsTelemetry.debug("Heading", pose.getHeading());
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

            pip.update();

            double robotX = pip.getPosX(DistanceUnit.INCH);
            double robotY = pip.getPosY(DistanceUnit.INCH);

            double xl = targetX - robotX;
            double yl = targetY - robotY;
            double distanceToTarget = Math.sqrt((xl * xl) + (yl * yl));

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

            double turretPos = finalServoDegrees / 404;
            hSub.turret1.setPosition(turretPos);
            hSub.turret2.setPosition(turretPos);

            double hoodPosition;
            if (distanceToTarget < 130) {
                double d1 = 57.5, d2 = 97.3;
                double v1 = 0.5, v2 = 0.7;
                double slope = (v2 - v1) / (d2 - d1);
                hoodPosition = v1 + (slope * (distanceToTarget - d1));
            } else {
                double d1 = 136.5, d2 = 158.1;
                double v1 = 0.7, v2 = 1.0;
                double slope = (v2 - v1) / (d2 - d1);
                hoodPosition = v1 + (slope * (distanceToTarget - d1));
            }
            hSub.hood.setPosition(Range.clip(hoodPosition, 0.5, 1.0));

            double vTarget = Range.clip(calculateVTarget(distanceToTarget), 0, 2500);

            hSub.flywheel1.setVelocity(vTarget);
            hSub.flywheel2.setVelocity(vTarget);

            autonomousPathUpdate();

            Pose pose = follower.getPose();
            drawOverlay(pose, finalServoDegrees);

            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", pose.getX());
            panelsTelemetry.debug("Y", pose.getY());
            panelsTelemetry.update(telemetry);
        }
    }

    private void drawOverlay(Pose pose, double turretServoDeg) {
        if (pose == null) return;

        try {
            PanelsField.INSTANCE.getField().setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        } catch(Exception ignored) {}

        if (paths != null) {
            drawSolidPathChain(paths.Path1, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path2, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path3, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path4, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path5, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path6, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path7, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path8, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path9, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path10, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path11, "rgba(200,200,200,0.1)");
            drawSolidPathChain(paths.Path12, "rgba(200,200,200,0.1)");

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
            case 2: return paths.Path1;
            case 3: return paths.Path2;
            case 4: return paths.Path3;
            case 5: return paths.Path4;
            case 6: return paths.Path4;
            case 7: return paths.Path5;
            case 8: return paths.Path6;
            case 9: return paths.Path6;
            case 10: return paths.Path7;
            case 11: return paths.Path8;
            case 12: return paths.Path8;
            case 13: return paths.Path9;
            case 14: return paths.Path10;
            case 15: return paths.Path11;
            case 16: return paths.Path12;
            default: return null;
        }
    }

    private double calculateVTarget(double distance) {
        if (distance < 130) {
            double d1 = 57.5, d2 = 97.3;
            double v1 = 1310, v2 = 1660;
            double slope = (v2 - v1) / (d2 - d1);
            return 1150 + (slope * (distance - d1));
        } else {
            double d1 = 136.5, d2 = 158.1;
            double v1 = 1900, v2 = 1940;
            double slope = (v2 - v1) / (d2 - d1);
            return farSlope + (slope * (distance - d1));
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1, true);
                setPathState(1);
                actionTimer.resetTimer();
                break;

            case 1: // Wait then drive (Original State 1)
                actionTimer.getElapsedTime();

                if (actionTimer.getElapsedTime() > 3000) {
                    hSub.intake.setPower(1);
                    hSub.gecko.setPower(-1);
                }

                if (actionTimer.getElapsedTime() > 5000) {
                    follower.resumePathFollowing();
                    actionTimer.resetTimer();
                    setPathState(2);
                    hSub.intake.setPower(0);
                    hSub.gecko.setPower(0);
                }
                break;

            case 2: // Wait then drive (Original State 2)
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                        hSub.intake.setPower(1);
                        hSub.gecko.setPower(1);
                    }

                    if (actionTimer.getElapsedTime() > 2000) {
                        follower.followPath(paths.Path2, true);
                        setPathState(3);
                        actionTimer.resetTimer();
                    }
                }
                break;

            case 3: // Wait then drive (Original State 3)
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                        hSub.intake.setPower(1);
                        hSub.gecko.setPower(-1);
                    }

                    if (actionTimer.getElapsedTime() > 2500) {
                        follower.followPath(paths.Path3, .8, false);
                        setPathState(4);
                        actionTimer.resetTimer();
                        hSub.intake.setPower(0);
                        hSub.gecko.setPower(0);
                    }
                }
                break;

            case 4: // DRIVE IMMEDIATELY
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 500) {
                    follower.followPath(paths.Path4, false);
                    setPathState(5);
                    actionTimer.resetTimer();
                }
                break;

            case 5:
                // FIX: Continuously assert power OUTSIDE the isBusy check.
                // This forces it to intake while driving AND while waiting.
                hSub.intake.setPower(1);
                hSub.gecko.setPower(1);

                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if(actionTimer.getElapsedTime() > 2000){
                        state5 = true;
                    }

                    if (state5) {
                        follower.resumePathFollowing();
                        follower.followPath(paths.Path5, true);
                        setPathState(6);
                        actionTimer.resetTimer();
                        state5 = false; // Reset flag for safety
                    }
                }
                break;

            case 6: // DRIVE IMMEDIATELY (Original State 4)
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                        hSub.intake.setPower(1);
                        hSub.gecko.setPower(-1);
                    }

                    if (actionTimer.getElapsedTime() > 2500) {
                        follower.followPath(paths.Path6, .8, false);
                        setPathState(7);
                        actionTimer.resetTimer();
                        hSub.intake.setPower(0);
                        hSub.gecko.setPower(0);
                    }
                }
                break;

            case 7: // DRIVE IMMEDIATELY
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, false);
                    setPathState(8);
                    actionTimer.resetTimer();
                }
                break;

            case 8: // Wait then drive
                // Continuously override while driving/waiting
                hSub.intake.setPower(1);
                hSub.gecko.setPower(1);

                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if(actionTimer.getElapsedTime() > 2000){
                        state8 = true;
                    }

                    if (state8) {
                        follower.resumePathFollowing();
                        follower.followPath(paths.Path8, true);
                        setPathState(9);
                        actionTimer.resetTimer();
                        state8 = false; // Reset flag for safety
                    }
                }
                break;


            case 9: // WAIT 4s (Contents of Original State 6)
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                        hSub.intake.setPower(1);
                        hSub.gecko.setPower(-1);
                    }

                    if (actionTimer.getElapsedTime() > 2500) {
                        follower.followPath(paths.Path9, .8, false);
                        setPathState(10);
                        actionTimer.resetTimer();
                        hSub.intake.setPower(0);
                        hSub.gecko.setPower(0);
                    }
                }
                break;

            case 10: // DRIVE IMMEDIATELY
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, false);
                    setPathState(11);
                    actionTimer.resetTimer();
                }
                break;

            case 11: // Wait then drive
                // Continuously override while driving/waiting
                hSub.intake.setPower(1);
                hSub.gecko.setPower(1);

                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if(actionTimer.getElapsedTime() > 2000){
                        state11 = true;
                    }

                    if (state11) {
                        follower.resumePathFollowing();
                        follower.followPath(paths.Path11, true);
                        setPathState(12);
                        actionTimer.resetTimer();
                        state11 = false; // Reset flag for safety
                    }
                }
                break;

            case 12: // Wait then drive (Original State 7)
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();

                    if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                        hSub.intake.setPower(1);
                        hSub.gecko.setPower(-1);
                    }

                    if (actionTimer.getElapsedTime() > 3000) {
                        follower.followPath(paths.Path12, .8, false);
                        setPathState(13);
                        actionTimer.resetTimer();
                        hSub.intake.setPower(0);
                        hSub.gecko.setPower(0);
                    }
                }
                break;

            case 13: // End
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
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;

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
                    .addPoseCallback(new Pose(86.000, 86.000, Math.toRadians(36)), () -> follower.pausePathFollowing(), .3)
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
                                    new Pose(101.000, 66.000),
                                    new Pose(120.000, 70.000)
                            )
                    )
                    .setBrakingStart(10)
                    .setBrakingStrength(9)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(120.000, 70.000),
                                    new Pose(124.000, 61.000),
                                    new Pose(132.000, 57.000)
                            )
                    )
                    //.addPoseCallback(new Pose(134.000, 57.000, Math.toRadians(45)), intakeSub.IntakeInGeckoOut(), 0.5)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(132.000, 57.000),
                                    new Pose(86.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.000, 86.000),
                                    new Pose(101.000, 66.000),
                                    new Pose(120.000, 70.000)
                            )
                    )
                    .setBrakingStart(10)
                    .setBrakingStrength(9)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(120.000, 70.000),
                                    new Pose(124.000, 61.000),
                                    new Pose(132.000, 57.000)
                            )
                    )
                    //.addPoseCallback(new Pose(120.000, 57.000, Math.toRadians(45)), intakeSub.IntakeInGeckoOut(), 0.5)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(132.000, 57.000),
                                    new Pose(86.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.000, 86.000),
                                    new Pose(101.000, 66.000),
                                    new Pose(120.000, 70.000)
                            )
                    )
                    .setBrakingStart(10)
                    .setBrakingStrength(9)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(120.000, 70.000),
                                    new Pose(124.000, 61.000),
                                    new Pose(132.000, 57.000)
                            )
                    )
                    //.addPoseCallback(new Pose(120.000, 57.000, Math.toRadians(45)), intakeSub.IntakeInGeckoOut(), 0.5)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(132.000, 57.000),
                                    new Pose(86.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(86.000, 86.000),
                                    new Pose(120.000, 72.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}