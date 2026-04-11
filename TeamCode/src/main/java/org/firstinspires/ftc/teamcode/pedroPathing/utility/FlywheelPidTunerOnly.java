package org.firstinspires.ftc.teamcode.pedroPathing.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Configurable
@TeleOp(name="Flywheel PID Tuner ONLY", group="tuning")
public class FlywheelPidTunerOnly extends LinearOpMode {

    // -------- Flywheel PIDF (Dashboard) --------
    private static double kP = 0.0;
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kF = 0.0;

    private static double targetVel = 0.0;
    private static double velStep   = 50.0;
    private static double maxVel    = 7000.0;

    private static double ticksPerRev = 28.0;

    // -------- Intake + indexer force run (Dashboard) --------
    // set to 1 to run both at 0.5, set to 0 to turn them off
    private static int RUN_FEED = 0;
    private static double FEED_POWER = 0.5;


    DcMotorEx f1, f2;
    DcMotor intake;
    DcMotorEx indexer;

    boolean prevUp = false;
    boolean prevDown = false;

    @Override
    public void runOpMode() {
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        f1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        f2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        intake = hardwareMap.get(DcMotor.class,   "intake");
        indexer  = hardwareMap.get(DcMotorEx.class, "indexer");
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);

        f1.setDirection(DcMotorSimple.Direction.REVERSE);
        f2.setDirection(DcMotorSimple.Direction.REVERSE);

        f1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        f2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Flywheel PID Tuner ONLY (+ optional feed motors)");
        telemetry.addLine("Dashboard: kP / kI / kD / kF / targetVel");
        telemetry.addLine("Dashboard: RUN_FEED (0/1) + FEED_POWER");
        telemetry.addLine("Dpad Up/Down = change target velocity");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------- Feed motors (only if RUN_FEED == 1) ----------
            if (RUN_FEED == 1) {
                double p = Range.clip(FEED_POWER, -1.0, 1.0);
                intake.setPower(p);
                indexer.setPower(p);
            } else {
                intake.setPower(0.0);
                indexer.setPower(0.0);
            }

            // ---------- Flywheel target adjust ----------
            boolean up   = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            if (up && !prevUp)     targetVel += velStep;
            if (down && !prevDown) targetVel -= velStep;

            prevUp = up;
            prevDown = down;

            targetVel = Range.clip(targetVel, 0.0, maxVel);

            // ---------- Apply PIDF live ----------
            f1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            f2.setVelocityPIDFCoefficients(kP, kI, kD, kF);

            // ---------- Command velocity ----------
            f1.setVelocity(targetVel);
            f2.setVelocity(targetVel);

            // ---------- Telemetry ----------
            double v1 = f1.getVelocity();
            double v2 = f2.getVelocity();

            telemetry.addData("Target (ticks/s)", "%.1f", targetVel);
            telemetry.addData("Flywheel1 (ticks/s)", "%.1f", v1);
            telemetry.addData("Flywheel2 (ticks/s)", "%.1f", v2);
            telemetry.addData("Flywheel1 RPM", "%.1f", (f1.getVelocity() * 60) / 28);
            telemetry.addData("Flywheel2 RPM", "%.1f", (f2.getVelocity() * 60) / 28);

            telemetry.addData("RUN_FEED", RUN_FEED);
            telemetry.addData("FEED_POWER", FEED_POWER);
            telemetry.addData("intake power", intake.getPower());
            telemetry.addData("indexer power", indexer.getPower());

            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.update();

            // Dashboard mirror
            FtcDashboard.getInstance().getTelemetry().addData("targetVel", targetVel);
            FtcDashboard.getInstance().getTelemetry().addData("v1", v1);
            FtcDashboard.getInstance().getTelemetry().addData("v2", v2);
            FtcDashboard.getInstance().getTelemetry().addData("RUN_FEED", RUN_FEED);
            FtcDashboard.getInstance().getTelemetry().addData("FEED_POWER", FEED_POWER);
            FtcDashboard.getInstance().getTelemetry().update();

            panelsTelemetry.addData("targetVel", targetVel);
            panelsTelemetry.addData("v1", v1);
            panelsTelemetry.addData("v2", v2);
            panelsTelemetry.addData("RUN_FEED", RUN_FEED);
            panelsTelemetry.addData("FEED_POWER", FEED_POWER);
            panelsTelemetry.addData("Target (ticks/s)", targetVel);
            panelsTelemetry.addData("Flywheel1 (ticks/s)",  v1);
            panelsTelemetry.addData("Flywheel2 (ticks/s)",  v2);
            panelsTelemetry.addData("Flywheel1 RPM", (f1.getVelocity() * 60) / 28);
            panelsTelemetry.addData("Flywheel2 RPM", (f2.getVelocity() * 60) / 28);
            panelsTelemetry.addData("RUN_FEED", RUN_FEED);
            panelsTelemetry.addData("FEED_POWER", FEED_POWER);
            panelsTelemetry.addData("intake power", intake.getPower());
            panelsTelemetry.update(telemetry);
        }
    }
}