package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.intakeSub;
import org.firstinspires.ftc.teamcode.Subsystems.turretSub;

@Autonomous(name = "path test", group = "Autonomous")
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
        follower.setStartingPose(new Pose(124.000, 120.000, Math.toRadians(-42)));

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
                    follower.followPath(paths.Path1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    setPathState(-1); // done
                }
                break;

            default:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path1, true);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) pathTimer.resetTimer();
    }

    public static class Paths {
        public PathChain Path1;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(124.000, 120.000),
                                    new Pose(73.000, 70.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-42), Math.toRadians(180))
                    .build();
        }
    }
}



















//  i made a repo lol
