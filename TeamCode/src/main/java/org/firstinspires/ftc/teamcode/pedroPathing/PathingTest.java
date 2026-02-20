package org.firstinspires.ftc.teamcode.pedroPathing;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.turretSub;

@Autonomous(name = "15 arty test", group = "Autonomous")
@Configurable // Panels
public class PathingTest extends OpMode {




    private TelemetryManager panelsTelemetry;   // Panels telemetry
    public Follower follower;                   // Pedro follower

    private int pathState = 0;                  // state machine state
    private Paths paths;                        // path definitions

    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotorEx intake, flywheel1, flywheel2, gecko;
    public turretSub turret;




    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(96, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        intake        = hardwareMap.get(DcMotorEx.class, "intake");
        flywheel1     = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2     = hardwareMap.get(DcMotorEx.class, "flywheel2");
        gecko         = hardwareMap.get(DcMotorEx.class, "gecko");


        // Timers MUST exist before you can reset them
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        pathTimer.resetTimer();
        actionTimer.resetTimer();
        opmodeTimer.resetTimer();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        turret = new turretSub(hardwareMap);

    }

    @Override
    public void start() {
        setPathState(0);
        opmodeTimer.resetTimer();

    }

    @Override
    public void loop() {

        // Update follower FIRST
        follower.update();
        turret.loop();

        // Run your autonomous state machine
        autonomousPathUpdate();

        // Telemetry
        Pose pose = follower.getPose();
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", pose.getX());
        panelsTelemetry.debug("Y", pose.getY());
        panelsTelemetry.debug("Heading", pose.getHeading());
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }


    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1, true);
                turret.turretPointFive();
                setPathState(1);
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
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
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
                    .addPath(
                            new BezierLine(
                                    new Pose(96.000, 8.000),
                                    new Pose(98.000, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(98.000, 35.000),
                                    new Pose(130.000, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(130.000, 35.000),
                                    new Pose(96.000, 8.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.000, 8.000),
                                    new Pose(100.000, 59.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.000, 59.000),
                                    new Pose(130.000, 59.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.000, 59.000),
                                    new Pose(101.000, 66.000),
                                    new Pose(130.000, 69.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.000, 69.000),
                                    new Pose(107.500, 72.500),
                                    new Pose(96.000, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.000, 8.000),
                                    new Pose(100.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.000, 83.000),
                                    new Pose(130.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.000, 83.000),
                                    new Pose(99.491, 91.140),
                                    new Pose(96.000, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.000, 8.000),
                                    new Pose(133.000, 27.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-63))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(133.000, 27.000),
                                    new Pose(135.000, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-63), Math.toRadians(-90))
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(135.000, 8.000),
                                    new Pose(116.800, 28.700),
                                    new Pose(96.000, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))
                    .build();
        }
    }

        }
