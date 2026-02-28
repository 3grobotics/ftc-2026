package org.firstinspires.ftc.teamcode.pedroPathing;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range; // Added for auto-aiming clamp

import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSub;
import org.firstinspires.ftc.teamcode.Subsystems.intakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.turretSub;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Autonomous(name = "15 arty test", group = "Autonomous")
@Configurable // Panels
public class PathingTest extends LinearOpMode {

    private TelemetryManager panelsTelemetry;   // Panels telemetry
    public Follower follower;                   // Pedro follower

    private int pathState = 0;                  // state machine state
    private Paths paths;                        // path definitions

    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotorEx intake, flywheel1, flywheel2, gecko;
    private Servo hood, turret1, turret2;
    public static turretSub turretSub;
    public static intakeSub intakeSub;
    public static flywheelSub flywheelSub;
    private hardwareSub hSub;

    private Robot robot;

    // Target coordinates for auto-aiming
    private double targetX = 144;
    private double targetY = 144;
    public static double farSlope = 1750;

    // ===== RR Action runner (minimal) =====
    private Action currentAction = null;
    private final TelemetryPacket actionPacket = new TelemetryPacket();

    private void startAction(Action a) {
        currentAction = a;
    }

    private void updateAction() {
        if (currentAction == null) {
            return;
        }
        // RR convention: true = keep running, false = finished
        boolean keepRunning = currentAction.run(actionPacket);
        if (!keepRunning) {
            currentAction = null;
        }
    }

    public class Robot {
        private class intake implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(1);
                gecko.setPower(1);
                return true; // keep looping
            }
        }
        public Action intake() { return new intake(); }

        private class fire implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intakeSub.intakeInGeckoIn();
                return true; // keep looping
            }
        }
        public Action fire() { return new fire(); }

        private class stopFire implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(0);
                gecko.setPower(0);
                return true; // keep looping
            }
        }
        public Action stopFire() { return new stopFire(); }

        private class flywheelUp implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                hood.setPosition(1);
                turretSub.turretFarFire();
                flywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
                flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
                flywheel1.setVelocity(1800);
                flywheel2.setVelocity(1800);
                return true; // keep looping
            }
        }
        public Action flywheelUp() { return new flywheelUp(); }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(96, 8, Math.toRadians(90)));

        hSub = new hardwareSub(hardwareMap);
        turretSub = new turretSub(hardwareMap);
        intakeSub = new intakeSub(hardwareMap);
        flywheelSub = new flywheelSub(hardwareMap);

        paths = new Paths(follower);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        pathTimer.resetTimer();
        actionTimer.resetTimer();
        opmodeTimer.resetTimer();

        hSub.flywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        hSub.flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        hSub.flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        hSub.flywheel2.setDirection(DcMotorEx.Direction.REVERSE);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        while (!isStarted() && !isStopRequested()) {
            follower.update();

            turretSub.loop();
            intakeSub.loop();
            flywheelSub.loop();

            Pose pose = follower.getPose();
            panelsTelemetry.debug("Status", "Init Loop");
            panelsTelemetry.debug("X", pose.getX());
            panelsTelemetry.debug("Y", pose.getY());
            panelsTelemetry.debug("Heading", pose.getHeading());
            panelsTelemetry.update(telemetry);
        }

        robot = new Robot();

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

            // =========================================================================
            //  DYNAMIC AUTO-AIMING (FROM TELEOP)
            // =========================================================================
            Pose pose = follower.getPose();
            double robotX = pose.getX();
            double robotY = pose.getY();
            double robotHeading = pose.getHeading();

            double xl = targetX - robotX;
            double yl = targetY - robotY;
            double hypot = Math.sqrt((xl * xl) + (yl * yl));

            // TURRET CALCULATION
            double angleToGoal = Math.atan2(yl, xl);
            double calculatedTurretRad = angleToGoal - robotHeading;

            while (calculatedTurretRad > Math.PI) calculatedTurretRad -= 2 * Math.PI;
            while (calculatedTurretRad < -Math.PI) calculatedTurretRad += 2 * Math.PI;

            double finalServoDegrees = Math.toDegrees(calculatedTurretRad) + 151.5;
            finalServoDegrees = Range.clip(finalServoDegrees, 0, 303);

            hSub.turret1.setPosition(finalServoDegrees / 303.0);
            hSub.turret2.setPosition(finalServoDegrees / 303.0);

            // HOOD LINEAR REGRESSION
            double hpos;
            if (hypot < 130) {
                double d1 = 57.5, d2 = 97.3;
                double v1 = 0.5, v2 = 0.7;
                double slope = (v2 - v1) / (d2 - d1);
                hpos = v1 + (slope * (hypot - d1));
            } else {
                double d1 = 136.5, d2 = 158.1;
                double v1 = 0.7, v2 = 1.0;
                double slope = (v2 - v1) / (d2 - d1);
                hpos = v1 + (slope * (hypot - d1));
            }
            hSub.hood.setPosition(Range.clip(hpos, 0.5, 1.0));

            // FLYWHEEL LINEAR REGRESSION
            double vTarget;
            if (hypot < 130) {
                double d1 = 57.5, d2 = 97.3;
                double v1 = 1310, v2 = 1660;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = 1150 + (slope * (hypot - d1));
            } else {
                double d1 = 136.5, d2 = 158.1;
                double v1 = 1900, v2 = 1940;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = farSlope + (slope * (hypot - d1));
            }
            vTarget = Range.clip(vTarget, 0, 2500);

            hSub.flywheel1.setVelocity(vTarget);
            hSub.flywheel2.setVelocity(vTarget);

            // ===== run current RR action each loop =====
            updateAction();

            autonomousPathUpdate();

            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", pose.getX());
            panelsTelemetry.debug("Y", pose.getY());
            panelsTelemetry.debug("Heading", pose.getHeading());
            panelsTelemetry.debug("Distance to Goal", hypot);
            panelsTelemetry.debug("Target Vel", vTarget);
            panelsTelemetry.debug("Busy", follower.isBusy());
            panelsTelemetry.debug("flywheel1 velocity", hSub.flywheel1.getVelocity());
            panelsTelemetry.debug("actiontimer", actionTimer.getElapsedTime());
            panelsTelemetry.update(telemetry);
        }
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                actionTimer.getElapsedTime();

                if (actionTimer.getElapsedTime() > 4000 && actionTimer.getElapsedTime() < 7000) {
                    hSub.intake.setPower(1);
                    hSub.gecko.setPower(-1);
                }

                if (actionTimer.getElapsedTime() > 7000 ) {
                    actionTimer.resetTimer();
                    follower.followPath(paths.Path1, true);
                    setPathState(1);
                    hSub.intake.setPower(0);
                    hSub.gecko.setPower(0);
                }
                break;

            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    follower.followPath(paths.Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    follower.followPath(paths.Path3, true); // (aka RESET)
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    follower.followPath(paths.Path4, true);
                    setPathState(4);
                    actionTimer.resetTimer();
                }
                break;

            case 4:
                follower.pausePathFollowing();

                if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                    hSub.intake.setPower(1);
                    hSub.gecko.setPower(-1);
                }

                if (actionTimer.getElapsedTime() > 3000) {
                    follower.resumePathFollowing();
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
                    actionTimer.resetTimer();
                    hSub.intake.setPower(0);
                    hSub.gecko.setPower(0);
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
                // Wait for Path 7 to completely finish driving back to base (96, 8)
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    actionTimer.resetTimer(); // Start the firing timer NOW
                    setPathState(8);          // Move to the firing state
                }
                break;

            case 8:
                // The robot is naturally stopped here. No path is running. No pausing required.
                if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                    hSub.intake.setPower(1);
                    hSub.gecko.setPower(-1);
                }

                if (actionTimer.getElapsedTime() > 3000) {
                    // Firing is done. NOW we officially start Path 8.
                    follower.followPath(paths.Path8, true);
                    setPathState(9); // Move directly to Case 9 to wait for Path 8 to finish
                    hSub.intake.setPower(0);
                    hSub.gecko.setPower(0);
                }
                break;

            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    follower.followPath(paths.Path9, true);
                    setPathState(10);
                }
                break;

            case 10:
                // Wait for Path 9 to finish, then start Path 10
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    follower.followPath(paths.Path10, true);
                    setPathState(11);
                }
                break;

            case 11:
                // Wait for Path 10 to completely finish driving back to base
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 100) {
                    actionTimer.resetTimer(); // Start the final firing timer NOW
                    setPathState(12);
                }
                break;

            case 12:
                // The robot is naturally stopped at the end of Path 10. Fire the last elements.
                if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                    hSub.intake.setPower(1);
                    hSub.gecko.setPower(-1);
                }

                if (actionTimer.getElapsedTime() > 3000) {
                    // Firing is done. End the autonomous routine.
                    setPathState(-1);
                    hSub.intake.setPower(0);
                    hSub.gecko.setPower(0);
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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(96.000, 8.000), new Pose(98.000, 35.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(98.0, 35.0), new Pose(130.0, 35.0)))
                    .addPoseCallback(new Pose(98.0, 35.0, Math.toRadians(0)), intakeSub.IntakeInGeckoOut(), 3.0)
                    .addParametricCallback(0.99, intakeSub.intakeAndGeckoStop())
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(130.000, 35.000), new Pose(96.000, 8.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(96.000, 8.000), new Pose(100.000, 59.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(100.000, 59.000), new Pose(130.000, 59.000)))
                    .addPoseCallback(new Pose(100, 59, Math.toRadians(0)), intakeSub.IntakeInGeckoOut(), .5)
                    .addParametricCallback(0.99, intakeSub.intakeAndGeckoStop())
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.000, 59.000),
                                    new Pose(124.000, 64.000),
                                    new Pose(129.000, 69.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(129.000, 69.000),
                                    new Pose(92.519, 69.817),
                                    new Pose(96.000, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.000, 8.000),
                                    new Pose(130.000, 27.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-63))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(130.000, 27.000),
                                    new Pose(130.000, 17.000)
                            )
                    )
                    .addPoseCallback(new Pose(130, 27, Math.toRadians(-63)), intakeSub.IntakeInGeckoOut(), 3.0)
                    .addParametricCallback(0.99, intakeSub.intakeAndGeckoStop())
                    .setLinearHeadingInterpolation(Math.toRadians(-63), Math.toRadians(-90))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.000, 17.000),
                                    new Pose(116.800, 28.700),
                                    new Pose(96.000, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))
                    .build();
        }
    }
}