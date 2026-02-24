package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Example teleop that showcases movement and field-centric driving.
 *
 * Pedro Pathing 2.x version
 */
@TeleOp(name = "Example Robot-Centric Teleop", group = "Examples")
public class ExampleRobotCentricTeleop extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(96, 8, Math.toRadians(90));

    @Override
    public void init() {
        // PP 2.x: use your generated Constants helper (instead of FConstants/LConstants)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // PP 2.x: setTeleOpDrive (not setTeleOpMovementVectors)
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,   // forward/back
                -gamepad1.left_stick_x,   // strafe
                -gamepad1.right_stick_x,  // turn
                true                      // match LocalizationTest (robot-centric flag in your snippet)
        );
        follower.update();

        Pose p = follower.getPose();
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(p.getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() { }
}