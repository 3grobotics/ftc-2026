package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.*;


/**
 * Combined Tuner: Lateral Multiplier, Translational PID, and Heading PID.
 * The robot will actively attempt to stay at (72, 72) with 0 degrees heading.
 * You can push/pull the robot to test the PID resistance and calculate your
 * Lateral Multiplier simultaneously.
 */
@TeleOp(name = "Unified Master Tuner", group = "Tuning")
public class UnifiedMasterTuner extends OpMode {
    private Follower follower;
    private Pose startPose = new Pose(72, 72, 0);

    // Set this to your physical test distance (e.g., 48 inches)
    public static double DISTANCE = 48;

    /**
     * This initializes the Follower and sets the starting pose.
     */
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    /**
     * Runs repeatedly during the initialization phase.
     */
    @Override
    public void init_loop() {
        follower.update();
        telemetry.addLine("Unified Tuner Ready.");
        telemetry.addData("Target Distance", DISTANCE);
        telemetry.addLine("Once started, the robot will hold its position.");
        telemetry.update();
    }

    /**
     * Activates the PIDFs and tells the robot to hold its starting point.
     */
    @Override
    public void start() {
        follower.deactivateAllPIDFs();
        follower.activateTranslational();
        follower.activateHeading();

        // Tells the robot to stay at the startPose (72, 72, 0)
        follower.holdPoint(startPose);
    }

    /**
     * Main loop that calculates the Lateral Multiplier and displays PID error.
     */
    @Override
    public void loop() {
        follower.update();

        // --- Lateral Multiplier Calculation ---
        // We measure how far the robot thinks it has moved in Y (Lateral)
        double currentY = follower.getPose().getY();
        double movedInches = currentY - startPose.getY();

        // Get the current multiplier from your localizer constants
        double currentMultiplier = follower.getPoseTracker().getLocalizer().getLateralMultiplier();

        // Calculate what the multiplier SHOULD be to match the physical DISTANCE
        // Formula: Target / (Measured / OldMultiplier)
        double recommendedMultiplier = 0;
        if (Math.abs(movedInches) > 0.1) {
            recommendedMultiplier = DISTANCE / (movedInches / currentMultiplier);
        }

        // --- Telemetry Output ---
        telemetry.addLine("== LATERAL TUNING ==");
        telemetry.addData("Instructions", "Pull robot right " + DISTANCE + " inches");
        telemetry.addData("Current Y Position", currentY);
        telemetry.addData("Distance Moved", movedInches);
        telemetry.addData("RECOMMENDED MULTIPLIER", recommendedMultiplier);

        telemetry.addLine("\n== PID FEEDBACK (HOLDING) ==");
        telemetry.addData("Translational Error X", follower.errorCalculator.getTranslationalError().getXComponent());
        telemetry.addData("Translational Error Y", follower.errorCalculator.getTranslationalError().getYComponent());
        telemetry.addData("Heading Error (Deg)", Math.toDegrees(follower.errorCalculator.getHeadingError()));

        telemetry.update();

        // Optional: Draw the robot on the dashboard/field if your setup supports it
        // follower.draw();
    }
}