package org.firstinspires.ftc.teamcode.pedroPathing;

import android.service.credentials.Action;

import androidx.annotation.NonNull;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.turretSub;

@Autonomous(name = "Daryl Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {




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
                flywheel1.setVelocity(1800);
                flywheel2.setVelocity(1800);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    intake.setPower(1);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.RESET, true);
                    intake.setPower(0);
                    setPathState(3);
                }

            case 3:
                if (!follower.isBusy()) {
                    setPathState(-1); // done
                }
                break;

            default:
                if (!follower.isBusy()) {
                    follower.followPath(paths.RESET, true);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) pathTimer.resetTimer();
    }

    public static class Paths {
        public PathChain RESET, Path1, Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(97.814, 8.000),
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

            RESET = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(130.000, 35.000),
                                    new Pose(96.000, 8.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

        }
    }
}