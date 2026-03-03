package org.firstinspires.ftc.teamcode.pedroPathing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSub;
import org.firstinspires.ftc.teamcode.Subsystems.intakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.turretSub;

@Autonomous(name = "close auto gate", group = "Autonomous")
@Configurable // Panels
public class closeAutoGate extends LinearOpMode {

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


    private double flywheel1Vel = 1800;
    private double flywheel2Vel = 1800;
    private Robot robot;

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
        follower.setStartingPose(new Pose(-46.5, 52.5, Math.toRadians(127), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE));

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
            double target  = 0.453;
            double target2 = 0.453;
            hSub.turret1.setPosition(target);
            hSub.turret2.setPosition(target2);

            hSub.flywheel1.setVelocity(flywheel1Vel);
            hSub.flywheel2.setVelocity(flywheel2Vel);

            // ===== run current RR action each loop =====
            updateAction();

            autonomousPathUpdate();

            Pose pose = follower.getPose();
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", pose.getX());
            panelsTelemetry.debug("Y", pose.getY());
            panelsTelemetry.debug("Heading", pose.getHeading());
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

                //if (actionTimer.getElapsedTime() > 4000 && actionTimer.getElapsedTime() < 7000) {
                //    hSub.intake.setPower(1);
                //    hSub.gecko.setPower(-1);
                //}

                //if (actionTimer.getElapsedTime() > 7000) {
                //    actionTimer.resetTimer();
                    follower.followPath(paths.Path1, true);
                    setPathState(1);
                //    hSub.intake.setPower(0);
                //    hSub.gecko.setPower(0);
                //}
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
               //follower.pausePathFollowing();

               //if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
               //    hSub.intake.setPower(1);
               //    hSub.gecko.setPower(-1);
               //}

                //if (actionTimer.getElapsedTime() > 3000) {
                 //   follower.resumePathFollowing();
                 //   follower.followPath(paths.Path5, true);
                    setPathState(5);
                //    actionTimer.resetTimer();
                //    hSub.intake.setPower(0);
                //    hSub.gecko.setPower(0);
                //}
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
                //// The robot is naturally stopped here. No path is running. No pausing required.
                //if (actionTimer.getElapsedTime() > 1000 && actionTimer.getElapsedTime() < 3000) {
                //    hSub.intake.setPower(1);
                //    hSub.gecko.setPower(-1);
                //}

                //if (actionTimer.getElapsedTime() > 3000) {
                //    // Firing is done. NOW we officially start Path 8.
                    follower.followPath(paths.Path8, true);
                    setPathState(9); // Move directly to Case 9 to wait for Path 8 to finish
                 //   hSub.intake.setPower(0);
                 //   hSub.gecko.setPower(0);
                //}
                break;

            case 9:
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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(122.000, 125.000),
                                    new Pose(69.000, 74.000),
                                    new Pose(95.000, 56.000),
                                    new Pose(126.000, 59.000)
                            )
                    )
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
                                    new Pose(134.000, 70.000),
                                    new Pose(134.000, 57.000)
                            )
                    )
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
                                    new Pose(134.000, 70.000),
                                    new Pose(134.000, 57.000)
                            )
                    )
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
                                    new Pose(134.000, 70.000),
                                    new Pose(134.000, 57.000)
                            )
                    )
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
        }
    }
}
