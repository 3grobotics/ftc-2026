package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry;   // Panels telemetry
    public Follower follower;                   // Pedro follower

    private int pathState = 0;                  // state machine state
    private Paths paths;                        // path definitions

    private Timer pathTimer, actionTimer, opmodeTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        // Timers MUST exist before you can reset them
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        pathTimer.resetTimer();
        actionTimer.resetTimer();
        opmodeTimer.resetTimer();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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
                    follower.followPath(paths.Path3, true);
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
                    follower.followPath(paths.Path1, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1); // done
                }
                break;

            default:
                // -1 or anything else: do nothing
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) pathTimer.resetTimer();
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(82.832, 8.000),
                            new Pose(80.087, 38.528),
                            new Pose(127.006, 35.329)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(127.006, 35.329),
                            new Pose(123.267, 33.004),
                            new Pose(119.307, 30.541),
                            new Pose(115.347, 28.079),
                            new Pose(111.387, 25.616),
                            new Pose(107.428, 23.154),
                            new Pose(103.468, 20.691),
                            new Pose(99.508, 18.229),
                            new Pose(95.549, 15.767),
                            new Pose(91.589, 13.304),
                            new Pose(83.230, 8.106)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(83.230, 8.106),
                            new Pose(85.795, 59.929),
                            new Pose(126.112, 59.478)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(126.112, 59.478),
                            new Pose(82.957, 6.484)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(82.957, 6.484),
                            new Pose(87.227, 86.075),
                            new Pose(125.484, 84.087)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(125.484, 84.087),
                            new Pose(83.124, 6.615)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();
        }
    }
}