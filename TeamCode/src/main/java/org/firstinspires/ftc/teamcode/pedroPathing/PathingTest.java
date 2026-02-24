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

import org.firstinspires.ftc.teamcode.Subsystems.flywheelSub;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSub;
import org.firstinspires.ftc.teamcode.Subsystems.intakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.turretSub;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

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
        if (currentAction == null) return;
        // RR convention: true = keep running, false = finished
        boolean keepRunning = currentAction.run(actionPacket);
        if (!keepRunning) currentAction = null;
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

        //intake    = hardwareMap.get(DcMotorEx.class, "intake");
        //flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        //flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        //gecko     = hardwareMap.get(DcMotorEx.class, "gecko");
        //hood      = hardwareMap.get(Servo.class, "hood");
        //turret1   = hardwareMap.get(Servo.class, "turret");
        //turret2   = hardwareMap.get(Servo.class, "turret2");



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
        if (isStopRequested()) return;

       // flywheelSub.autoFlywheelFar();
        setPathState(0);
        opmodeTimer.resetTimer();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            turretSub.loop();
            intakeSub.loop();
            flywheelSub.loop();
            double target  = 0.4;
            double target2 = 0.4;
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

                if (actionTimer.getElapsedTime() > 3000) {
                    hSub.intake.setPower(1);
                    hSub.gecko.setPower(-1);
                }

                if (actionTimer.getElapsedTime() > 6000) {
                    actionTimer.resetTimer();
                    follower.followPath(paths.Path1, true);
                    setPathState(1);
                    hSub.intake.setPower(0);
                    hSub.gecko.setPower(0);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true); // (aka RESET)
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    setPathState(4);
                    actionTimer.resetTimer();   // ✅ START THE PAUSE TIMER WHEN ENTERING STATE 4
                }
                break;

            case 4:
                follower.pausePathFollowing();

                if (actionTimer.getElapsedTime() > 1000) {
                    hSub.intake.setPower(1);
                    hSub.gecko.setPower(-1);
                }

                if (actionTimer.getElapsedTime() > 3000) {
                    follower.resumePathFollowing();
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
                    actionTimer.resetTimer();   // optional: reset for next timed thing
                    hSub.intake.setPower(0);
                    hSub.gecko.setPower(0);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path12, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path13, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) pathTimer.resetTimer();
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
        public PathChain Path13;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(96.000, 8.000), new Pose(98.000, 35.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(98.0, 35.0), new Pose(130.0, 35.0)))
                    .addPoseCallback(new Pose(98.0, 35.0, Math.toRadians(0)), intakeSub.IntakeInGeckoOut(), 3.0)
                    .addParametricCallback(99, intakeSub.intakeAndGeckoStop())
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
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(130.000, 59.000), new Pose(101.000, 66.000), new Pose(130.000, 69.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(130.000, 69.000), new Pose(107.500, 72.500), new Pose(96.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(96.000, 8.000), new Pose(100.000, 83.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(100.000, 83.000), new Pose(130.000, 83.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(130.000, 83.000), new Pose(99.491, 91.140), new Pose(96.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(96.000, 8.000), new Pose(133.000, 27.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-63))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(133.000, 27.000), new Pose(135.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-63), Math.toRadians(-90))
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(135.000, 8.000), new Pose(116.800, 28.700), new Pose(96.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))
                    .build();
        }
    }
}