package org.firstinspires.ftc.teamcode.pedroPathing.testing;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;
import org.firstinspires.ftc.teamcode.pedroPathing.ColorCamera2;
import org.firstinspires.ftc.teamcode.pedroPathing.testing.velocityVarSub;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name="velocity compensation test", group="Linear OpMode")
public class velCompensationTest extends LinearOpMode {
    hardwareSubNewBot h;
    varSub v;
    double turretAngle = .5;
    double flywheelSpeed = 1;
    double hoodAngle = 0.7;

    private IntakeState currentIntakeState = IntakeState.IDLE;
    private ElapsedTime intakeStateTimer = new ElapsedTime();
    private int cycleSubStep = 0; // Used for multi-step cycles like single-note and till-color

    enum IntakeState {
        IDLE,
        MANUAL_TRIGGERS_ACTIVE, // User holding triggers for intake/outtake
        GATE_MANUAL_CONTROL,    // User holding dpad for gate position

        SINGLE_NOTE_CYCLE_INIT,     // Triggered by dpad_left
        SINGLE_NOTE_CYCLE_STEP_1,   // Sickle out, swing down, gate open
        SINGLE_NOTE_CYCLE_STEP_2,   // Intake/Indexer on
        SINGLE_NOTE_CYCLE_STEP_3,   // Sickle to 0.9
        SINGLE_NOTE_CYCLE_STEP_4,   // Gate to 0.85, swing to 0.2, intake off
        SINGLE_NOTE_CYCLE_STEP_5,   // Intake 0.5, Indexer -1
        SINGLE_NOTE_CYCLE_STEP_6,   // Indexer 1
        SINGLE_NOTE_CYCLE_STEP_7,   // Gate to 0.65, Indexer off, Swing to 0.55, Intake -1
        SINGLE_NOTE_CYCLE_STEP_8,   // Intake off, cycle complete

        FIRING,                     // Triggered by right_bumper
        FIRING_COMPLETE,            // Short delay after firing

        INTAKE_TILL_FULL_INIT,      // Triggered by gamepad1.ps
        INTAKE_TILL_FULL_RUNNING,   // Actively intaking until sensors are full

        INTAKE_TILL_COLOR_INIT,     // Generic init for color cycles
        INTAKE_TILL_COLOR_SWING_RESET, // Swing arm up, 500ms delay, then start intake cycle steps
        INTAKE_TILL_COLOR_STEP_1,   // Part of the full intake cycle for 'till color'
        INTAKE_TILL_COLOR_STEP_2,
        INTAKE_TILL_COLOR_STEP_3,
        INTAKE_TILL_COLOR_STEP_4,
        INTAKE_TILL_COLOR_STEP_5,
        INTAKE_TILL_COLOR_STEP_6,
        INTAKE_TILL_COLOR_STEP_7,
        INTAKE_TILL_COLOR_STEP_8, // Cycle step 8, then check color and loop if not found

        // Specific flags for which color processor to use during INTAKE_TILL_COLOR_RUNNING
        INTAKE_TILL_PPG, // right_stick_button
        INTAKE_TILL_PGP, // left_stick_button
        INTAKE_TILL_GPP  // left_stick_button && right_stick_button
    }

    // Debounce variables for intake controls (gamepad1)
    private boolean dpadLeftWasPressed = false;
    private boolean psWasPressed = false;
    private boolean rightBumperWasPressed = false;
    private boolean leftStickButtonWasPressed = false;
    private boolean rightStickButtonWasPressed = false; // Changed from original to prevent immediate re-trigger


    @Override
    public void runOpMode() {
        h = new hardwareSubNewBot(hardwareMap);
        v = new varSub();

        h.pip.resetPosAndIMU();

        boolean left, right, prevleft = false, prevright = false;
        boolean aa, bb, prevaa = false, prevbb = false;

        double samOffset = 0;
        waitForStart();

        while (opModeIsActive()) {
            boolean currentDpadLeft = gamepad1.dpad_left;
            boolean currentPs = gamepad1.ps;
            boolean currentRightBumper = gamepad1.right_bumper;
            boolean currentRightStickButton = gamepad1.right_stick_button;
            boolean currentLeftStickButton = gamepad1.left_stick_button;

            // State transition logic from IDLE or by interrupting certain states
            // Prioritize specific multi-button or complex actions over simpler ones.
            // If currently in an active intake state, new button presses might override or be ignored.
            if (currentIntakeState == IntakeState.IDLE || currentIntakeState == IntakeState.MANUAL_TRIGGERS_ACTIVE || currentIntakeState == IntakeState.GATE_MANUAL_CONTROL) {
                if (currentLeftStickButton && currentRightStickButton && !leftStickButtonWasPressed && !rightStickButtonWasPressed) {
                    // Corrected priority: Most specific combination first
                    currentIntakeState = IntakeState.INTAKE_TILL_GPP;
                    cycleSubStep = 0; // Reset sub-step for the new cycle
                    intakeStateTimer.reset();
                } else if (currentRightStickButton && !rightStickButtonWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_PPG;
                    cycleSubStep = 0; // Reset sub-step for the new cycle
                    intakeStateTimer.reset();
                } else if (currentLeftStickButton && !leftStickButtonWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_PGP;
                    cycleSubStep = 0; // Reset sub-step for the new cycle
                    intakeStateTimer.reset();
                } else if (currentPs && !psWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_FULL_INIT;
                    intakeStateTimer.reset();
                } else if (currentDpadLeft && !dpadLeftWasPressed) {
                    currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_INIT;
                    intakeStateTimer.reset();
                } else if (currentRightBumper && !rightBumperWasPressed) { // Only transition to FIRING on initial press
                    currentIntakeState = IntakeState.FIRING;
                    // No timer reset here; FIRING is sustained as long as button is held
                } else {
                    // Manual controls (triggers or gate dpad) if no other sequence is active
                    double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                    if (Math.abs(intakeCmd) > 0.1) {
                        currentIntakeState = IntakeState.MANUAL_TRIGGERS_ACTIVE;
                    } else if (gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_up) {
                        currentIntakeState = IntakeState.GATE_MANUAL_CONTROL;
                    } else if (currentIntakeState != IntakeState.IDLE) { // Ensure idle if no input, AND not already in an active non-manual state
                        currentIntakeState = IntakeState.IDLE;
                    }
                }
            }
            // Update debounce variables
            dpadLeftWasPressed = currentDpadLeft;
            psWasPressed = currentPs;
            rightBumperWasPressed = currentRightBumper;
            rightStickButtonWasPressed = currentRightStickButton;
            leftStickButtonWasPressed = currentLeftStickButton;

            switch (currentIntakeState) {
                case IDLE:
                    // Default positions when nothing is active
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                    h.sickle.setPosition(1.0);
                    // h.swingArm.setPosition(1.0); // Allow swingArm to retain its last position when idle unless another state sets it
                    h.gate.setPosition(0.65); // Default closed/intake position
                    break;

                case MANUAL_TRIGGERS_ACTIVE:
                    double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                    h.intake.setPower(-intakeCmd);
                    h.indexer.setPower(-intakeCmd);
                    h.swingArm.setPosition(.6); // Move swing arm for manual intake
                    h.gate.setPosition(.65);    // Set gate for manual intake

                    if (Math.abs(intakeCmd) < 0.1) { // Triggers released
                        currentIntakeState = IntakeState.IDLE; // Transition to IDLE, swingArm stays at .6
                    }
                    break;

                case GATE_MANUAL_CONTROL:
                    if (gamepad1.dpad_right) {
                        h.gate.setPosition(0.8);
                    } else if (gamepad1.dpad_down) {
                        h.gate.setPosition(0.65);
                    } else if (gamepad1.dpad_up) {
                        h.gate.setPosition(1);
                    } else { // No dpad for gate pressed
                        currentIntakeState = IntakeState.IDLE;
                    }
                    // For manual gate control, the swing arm should probably stay put or return to a default position
                    // Currently, it will retain its position from prior states. If you want it retracted here, add:
                    // h.swingArm.setPosition(1.0);
                    break;

                case SINGLE_NOTE_CYCLE_INIT:
                    h.sickle.setPosition(0.85);
                    h.swingArm.setPosition(0.55);
                    h.gate.setPosition(0.82);
                    intakeStateTimer.reset();
                    currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_1;
                    break;
                case SINGLE_NOTE_CYCLE_STEP_1:
                    if (intakeStateTimer.milliseconds() > 250) {
                        h.intake.setPower(-1);
                        h.indexer.setPower(-1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_2;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_2:
                    if (intakeStateTimer.milliseconds() > 500) { // Cumulative time
                        h.sickle.setPosition(0.9);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_3;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_3:
                    if (intakeStateTimer.milliseconds() > 750) { // Cumulative time
                        h.gate.setPosition(0.85);
                        h.swingArm.setPosition(0.2);
                        h.intake.setPower(0);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_4;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_4:
                    if (intakeStateTimer.milliseconds() > 1000) { // Cumulative time
                        h.intake.setPower(0.5);
                        h.indexer.setPower(-1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_5;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_5:
                    if (intakeStateTimer.milliseconds() > 1200) { // Cumulative time
                        h.indexer.setPower(1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_6;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_6:
                    if (intakeStateTimer.milliseconds() > 1300) { // Cumulative time
                        h.gate.setPosition(0.65);
                        h.indexer.setPower(0);
                        h.swingArm.setPosition(0.55);
                        h.intake.setPower(-1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_7;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_7:
                    if (intakeStateTimer.milliseconds() > 1400) { // Cumulative time
                        h.intake.setPower(0);
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;

                case FIRING:
                    h.gate.setPosition(.97);
                    h.indexer.setPower(-1);
                    h.intake.setPower(-1);
                    h.swingArm.setPosition(.95);
                    // Firing is active as long as the button is held.
                    if (!gamepad1.right_bumper) { // Button released
                        currentIntakeState = IntakeState.FIRING_COMPLETE;
                        intakeStateTimer.reset(); // Start timer for cleanup
                    }
                    break;
                case FIRING_COMPLETE:
                    // Reset to idle after a short delay to ensure components return
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                    h.swingArm.setPosition(1.0); // Retract swing arm after firing
                    h.gate.setPosition(0.65); // Return gate to default closed/intake position
                    if (intakeStateTimer.milliseconds() > 200) { // Short delay for components to return
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;

                case INTAKE_TILL_FULL_INIT:
                    h.gate.setPosition(0.65);
                    h.intake.setPower(-1);
                    h.indexer.setPower(-1);
                    h.sickle.setPosition(1.0); // Ensure sickle is up for general intake
                    h.swingArm.setPosition(0.6); // Lower swing arm for intake
                    currentIntakeState = IntakeState.INTAKE_TILL_FULL_RUNNING;
                    break;
                case INTAKE_TILL_FULL_RUNNING:
                    // Stop if ALL sensors detect something (assuming getState() is true for detection)
                    if (h.topDistSensor.getState() && h.midDistSensor.getState() && h.frontDistSensor.getState()) {
                        h.intake.setPower(0);
                        h.indexer.setPower(0);
                        currentIntakeState = IntakeState.IDLE;
                    } else {
                        // Continue intaking
                        h.intake.setPower(-1);
                        h.indexer.setPower(-1);
                    }
                    break;




            }

            vectorAndTurretUpdate(h.pip.getHeading(AngleUnit.RADIANS));

            while (turretAngle > Math.PI) turretAngle -= 2 * Math.PI;
            while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;

            // 1. Calculate base degrees from physics and apply your proven center logic
            double baseServoDegrees = Math.toDegrees(turretAngle) - (314.6112145 / 2.0);

            left = gamepad2.dpad_left;
            right = gamepad2.dpad_right;
            aa = gamepad2.a;
            bb = gamepad2.b;

            if (left && !prevleft && !right) samOffset = Range.clip(samOffset + 2.5, -40, 40);
            if (right && !prevright && !left) samOffset = Range.clip(samOffset - 2.5, -40, 40);
            if (aa && !prevaa && !bb) samOffset = Range.clip(samOffset + 2.5, -40, 40);
            if (bb && !prevbb && !aa) samOffset = Range.clip(samOffset - 2.5, -40, 40);

            prevleft = left; prevright = right;
            prevaa = aa; prevbb = bb;

            double finalServoDegrees = baseServoDegrees + samOffset;
            double finalPosition = Math.abs((finalServoDegrees) / 314.6112145);

            // 2. Hardware: Turret
            if (gamepad2.ps) {
                h.turret1.setPosition(0.5);
                h.turret2.setPosition(0.5);
            } else {
                h.turret1.setPosition(Range.clip(finalPosition, 0, 1));
                h.turret2.setPosition(Range.clip(finalPosition, 0, 1));
            }

            // 3. Hardware: Flywheel
            if (gamepad1.x) v.var = 1;
            else if (gamepad1.y) v.var = 0;

            if (v.var == 1) {
                h.flywheel1.setVelocity(getFlywheelTicksFromVelocity(flywheelSpeed));
                h.flywheel2.setVelocity(getFlywheelTicksFromVelocity(flywheelSpeed));
            } else {
                h.flywheel1.setVelocity(0);
                h.flywheel2.setVelocity(0);
            }

            // 4. Hardware: Hood (Mapping physics to your specific servo limits)
            // 0.558 rad (32°) = Flattest Shot -> 0.0 Servo
            // 0.959 rad (55°) = Steepest Shot -> 0.7 Servo
            double hoodPercent = (hoodAngle - 0.558505) / (0.959931 - 0.558505);
            double finalHoodPos = hoodPercent * 0.7;
            h.hood.setPosition(Range.clip(finalHoodPos, 0, 0.7));

            // 5. Drive Control
            v.axial = -gamepad1.left_stick_y;
            v.lateral = gamepad1.left_stick_x;
            v.yawCmd = gamepad1.right_stick_x;

            double fl = v.axial + v.lateral + v.yawCmd;
            double fr = v.axial - v.lateral - v.yawCmd;
            double bl = v.axial - v.lateral + v.yawCmd;
            double br = v.axial + v.lateral - v.yawCmd;

            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            h.frontLeft.setPower(fl / max);
            h.frontRight.setPower(fr / max);
            h.backLeft.setPower(bl / max);
            h.backRight.setPower(br / max);

            // Telemetry
            telemetry.addData("Raw Rad", turretAngle);
            telemetry.addData("Turret Servo", finalPosition);
            telemetry.addData("Hood Angle Deg", Math.toDegrees(hoodAngle));
            telemetry.addData("Hood Servo Pos", finalHoodPos);
            telemetry.addData("TPS", getFlywheelTicksFromVelocity(flywheelSpeed));
            telemetry.update();

            h.pip.update();
        }
    }

    private void vectorAndTurretUpdate(double robotHeading){
        double robotX = h.pip.getPosX(DistanceUnit.INCH);
        double robotY = h.pip.getPosY(DistanceUnit.INCH);

        double xl = v.tx - robotX;
        double yl = v.ty - robotY;
        double x = Math.sqrt((xl * xl) + (yl * yl));
        double angleToGoal = Math.atan2(yl, xl);

        double vx = h.pip.getVelX(DistanceUnit.INCH);
        double vy = h.pip.getVelY(DistanceUnit.INCH);

        // 1. Time Estimation & Virtual Target (Velocity Compensation)
        double time = x / 120.0;
        time = Range.clip(time, 0, 1.5);

        double virtualX = xl - (vx * time);
        double virtualY = yl - (vy * time);

        // ndr is the velocity-compensated distance
        double ndr = Math.sqrt(virtualX * virtualX + virtualY * virtualY);
        double compensatedAngleToGoal = Math.atan2(virtualY, virtualX);

        // ==========================================================
        // THE FIX: Define the arc trajectory
        // Instead of a broken formula, we smoothly transition the hood
        // from 55° (Close: 50 in) down to 32° (Far: 130 in).
        // ==========================================================
        double targetAngleDegrees = 55.0 - ((ndr - 50.0) * (55.0 - 32.0) / (130.0 - 50.0));
        targetAngleDegrees = Range.clip(targetAngleDegrees, 32.0, 55.0);

        // Convert to radians for the physics engine
        hoodAngle = Math.toRadians(targetAngleDegrees);

        // ==========================================================
        // PURE PHYSICS: Calculate the exact speed for the chosen angle
        // ==========================================================
        double cosH = Math.cos(hoodAngle);
        double tanH = Math.tan(hoodAngle);

        // The Math.max prevents the 20,000 tick explosion if the math gets too tight
        double verticalGap = Math.max(1.0, (ndr * tanH - velocityVarSub.goalHeight));

        double radicand = (velocityVarSub.g * ndr * ndr) / (2 * Math.pow(cosH, 2) * verticalGap);
        flywheelSpeed = (radicand > 0) ? Math.sqrt(radicand) : 0;

        // Final Turret Angle
        turretAngle = (compensatedAngleToGoal - robotHeading + Math.PI);
    }
    public static double getFlywheelTicksFromVelocity(double physicsVelocityInches){
        // 1. Convert theoretical physics output to FPS
        double theoreticalFPS = physicsVelocityInches / 12.0;

        // 2. Apply the "Reality Check" Ratio
        // This scales the aggressive pure math down to your highly efficient IRL hardware.
        // If it shoots a little too hot, lower this to 0.60. If it falls short, raise to 0.65.
        double aerodynamicFactor = 0.63;
        double actualTargetFPS = theoreticalFPS * aerodynamicFactor;

        // 3. Apply your IRL spreadsheet trendline
        double tps = (77.6 * actualTargetFPS) + 20.9;

        return tps;
    }
}