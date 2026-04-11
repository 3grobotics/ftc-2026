package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSub;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import static org.firstinspires.ftc.teamcode.pedroPathing.stateTeleOpRedNOTMODDED.farSlope;

@TeleOp(name = "ColorCamera 2")
public class ColorCamera2 extends LinearOpMode {

    public hardwareSubNewBot h;
    public varSub v;
    public Timer swingTimer;

    // --- PTO Control Variables (already a good state machine, kept as is) ---
    private boolean ptoButtonWasPressed = false; // For debouncing the gamepad button
    private boolean ptoIsEngaged = false;           // True if PTO is "on", false if "off"
    private ElapsedTime ptoDeploymentTimer = new ElapsedTime();
    private PtoDeploymentState currentPtoDeploymentState = PtoDeploymentState.RETRACTED;

    enum PtoDeploymentState {
        RETRACTED,
        DEPLOYING_R_SERVO,
        WAITING_FOR_DEPLOY_DELAY,
        DEPLOYING_L_SERVO,
        ENGAGED,
        RETRACTING_L_SERVO,
        WAITING_FOR_RETRACT_DELAY,
        RETRACTING_R_SERVO
    }

    private static final double PTO_R_RETRACTED_POSITION = 1.0;
    private static final double PTO_L_RETRACTED_POSITION = 1.0;
    private static final double PTO_R_ENGAGED_POSITION = 0.25;
    private static final double PTO_L_ENGAGED_POSITION = 0.25;
    private static final long PTO_DEPLOYMENT_DELAY_MS = 500;
    // --- END PTO Control Variables ---


    // --- NEW Intake Control Variables (State Machine for Intake and related components) ---
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

    double hpos;
    double samOffset;

    /**
     * This OpMode illustrates how to use a video source (camera) as a color sensor
     */
    @Override
    public void runOpMode() {

        /* these are the subsystems */
        {
            h = new hardwareSubNewBot(hardwareMap);
            v = new varSub();
        }

        /* these are the timers */
        {
            swingTimer = new Timer(); // Note: This timer is not used in the provided code, but kept.
        }

        /* this is the camera stuff */
        PredominantColorProcessor.Builder frontProcessorBuilder;
        PredominantColorProcessor.Builder middleProcessorBuilder;
        PredominantColorProcessor.Builder backProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        PredominantColorProcessor frontPredominantColorProcessor;
        PredominantColorProcessor middlePredominantColorProcessor;
        PredominantColorProcessor backPredominantColorProcessor;
        VisionPortal myVisionPortal;
        PredominantColorProcessor.Result frontResult = null; // Initialize to null
        PredominantColorProcessor.Result middleResult = null; // Initialize to null
        PredominantColorProcessor.Result backResult = null;   // Initialize to null


        // Build a "Color Sensor" vision processor based on the PredominantColorProcessor class.
        frontProcessorBuilder = new PredominantColorProcessor.Builder();
        middleProcessorBuilder = new PredominantColorProcessor.Builder();
        backProcessorBuilder = new PredominantColorProcessor.Builder();
        frontProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(60, 60, 200, 200));
        middleProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(300, 100, 400, 300));
        backProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(700, 100, 800, 300));

        frontProcessorBuilder.setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.BLUE,
                PredominantColorProcessor.Swatch.YELLOW,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.WHITE);
        frontPredominantColorProcessor = frontProcessorBuilder.build();

        middleProcessorBuilder.setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.BLUE,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.WHITE);
        middlePredominantColorProcessor = middleProcessorBuilder.build();

        backProcessorBuilder.setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.BLUE,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.WHITE);
        backPredominantColorProcessor = backProcessorBuilder.build();

        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.addProcessor(frontPredominantColorProcessor);
        myVisionPortalBuilder.addProcessor(middlePredominantColorProcessor);
        myVisionPortalBuilder.addProcessor(backPredominantColorProcessor);
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.setCameraResolution(new Size(800, 448));
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortal = myVisionPortalBuilder.build();

        /* telemetry stuff */
        {
            telemetry.setMsTransmissionInterval(50);
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        }

        // Initialize digital sensor mode (should be done once)
        h.topDistSensor.setMode(DigitalChannel.Mode.INPUT);
        h.midDistSensor.setMode(DigitalChannel.Mode.INPUT); // Assuming these exist
        h.frontDistSensor.setMode(DigitalChannel.Mode.INPUT); // Assuming these exist

        // Set initial positions for intake-related servos and motors
        h.sickle.setPosition(1.0);     // Default: up/retracted
        h.gate.setPosition(0.65);      // Default: closed/intake pos
        h.swingArm.setPosition(1.0);   // Default: up/retracted - This initial setting is fine, as it ensures a known starting state.
        h.intake.setPower(0);
        h.indexer.setPower(0);


        // Gamepad debounce variables for turret trim (gamepad2)
        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;

        boolean aa = gamepad2.a;
        boolean bb = gamepad2.b;
        boolean prevaa = aa;
        boolean prevbb = bb;

        waitForStart();
        while (opModeIsActive()) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            frontResult = frontPredominantColorProcessor.getAnalysis();
            middleResult = middlePredominantColorProcessor.getAnalysis();
            backResult = backPredominantColorProcessor.getAnalysis();

            // =========================================================================
            // NEW INTAKE STATE MACHINE LOGIC
            // =========================================================================

            // Debounce gamepad1 buttons for intake control
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


            // Execute current intake state logic
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

                // --- INTAKE TILL COLOR STATES (PPG, PGP, GPP) ---
                // These states cycle through a full intake sequence, then check for color, and repeat if not found.
                // The 'INIT' state sets up the first cycle step.
                case INTAKE_TILL_PPG:
                case INTAKE_TILL_PGP:
                case INTAKE_TILL_GPP:
                    PredominantColorProcessor.Swatch targetSwatch = PredominantColorProcessor.Swatch.ARTIFACT_GREEN;
                    PredominantColorProcessor.Result checkResult = null;

                    if (currentIntakeState == IntakeState.INTAKE_TILL_PPG) {
                        checkResult = frontResult;
                    } else if (currentIntakeState == IntakeState.INTAKE_TILL_PGP) {
                        checkResult = middleResult;
                    } else { // INTAKE_TILL_GPP
                        checkResult = backResult;
                    }

                    // Always check color at the beginning of each full cycle attempt
                    if (checkResult != null && checkResult.closestSwatch.equals(targetSwatch)) {
                        h.intake.setPower(0);
                        h.indexer.setPower(0);
                        h.swingArm.setPosition(1); // Final reset position
                        cycleSubStep = 0; // Reset for next time
                        currentIntakeState = IntakeState.IDLE;
                        break; // Exit state execution
                    }

                    // If color not found, proceed with the internal cycle steps
                    switch (cycleSubStep) {
                        case 0: // Initial swing arm reset (from previous cycle or start)
                            h.swingArm.setPosition(1);
                            h.sickle.setPosition(1); // Ensure sickle is up during reset
                            h.gate.setPosition(0.65); // Ensure gate is default
                            h.intake.setPower(0);
                            h.indexer.setPower(0);
                            intakeStateTimer.reset(); // Reset timer for this step
                            cycleSubStep = 1;
                            break;
                        case 1: // Wait for swing arm reset delay (500ms from original sleep)
                            if (intakeStateTimer.milliseconds() > 500) {
                                h.sickle.setPosition(0.85);
                                h.swingArm.setPosition(0.6);
                                h.gate.setPosition(0.82);
                                intakeStateTimer.reset();
                                cycleSubStep = 2;
                            }
                            break;
                        case 2: // Wait for 250ms then intake on
                            if (intakeStateTimer.milliseconds() > 250) {
                                h.intake.setPower(-1);
                                h.indexer.setPower(-1);
                                intakeStateTimer.reset();
                                cycleSubStep = 3;
                            }
                            break;
                        case 3: // Wait for 250ms then sickle to 0.9
                            if (intakeStateTimer.milliseconds() > 250) {
                                h.sickle.setPosition(0.9);
                                intakeStateTimer.reset();
                                cycleSubStep = 4;
                            }
                            break;
                        case 4: // Wait for 250ms then gate 0.85, swing 0.2, intake off
                            if (intakeStateTimer.milliseconds() > 250) {
                                h.gate.setPosition(0.85);
                                h.swingArm.setPosition(0.2);
                                h.intake.setPower(0);
                                intakeStateTimer.reset();
                                cycleSubStep = 5;
                            }
                            break;
                        case 5: // Wait for 250ms then intake 0.5, indexer -1
                            if (intakeStateTimer.milliseconds() > 250) {
                                h.intake.setPower(0.5);
                                h.indexer.setPower(-1);
                                intakeStateTimer.reset();
                                cycleSubStep = 6;
                            }
                            break;
                        case 6: // Wait for 200ms then indexer 1
                            if (intakeStateTimer.milliseconds() > 200) {
                                h.indexer.setPower(1);
                                intakeStateTimer.reset();
                                cycleSubStep = 7;
                            }
                            break;
                        case 7: // Wait for 100ms then gate 0.65, indexer 0, swing 0.6, intake -1
                            if (intakeStateTimer.milliseconds() > 100) {
                                h.gate.setPosition(0.65);
                                h.indexer.setPower(0);
                                h.swingArm.setPosition(0.6);
                                h.intake.setPower(-1);
                                intakeStateTimer.reset();
                                cycleSubStep = 8;
                            }
                            break;
                        case 8: // Wait for 100ms then intake off. Cycle complete, loop back to check color.
                            if (intakeStateTimer.milliseconds() > 100) {
                                h.intake.setPower(0);
                                cycleSubStep = 0; // Reset sub-step to re-enter case 0 and do swing arm reset and color check
                            }
                            break;
                    }
                    break;
            }
            // --- END NEW INTAKE STATE MACHINE LOGIC ---

            // --- PTO Control Logic (kept as is, it's already a non-blocking state machine) ---
            if (gamepad1.options && !ptoButtonWasPressed) {
                ptoIsEngaged = !ptoIsEngaged;
                ptoButtonWasPressed = true;

                if (ptoIsEngaged) {
                    currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_R_SERVO;
                    ptoDeploymentTimer.reset();
                } else {
                    currentPtoDeploymentState = PtoDeploymentState.RETRACTING_L_SERVO;
                    ptoDeploymentTimer.reset();
                }
            } else if (!gamepad1.options) {
                ptoButtonWasPressed = false;
            }

            switch (currentPtoDeploymentState) {
                case RETRACTED:
                    h.ptoR.setPosition(PTO_R_RETRACTED_POSITION);
                    h.ptoL.setPosition(PTO_L_RETRACTED_POSITION);
                    break;
                case DEPLOYING_R_SERVO:
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_L_SERVO;
                    break;
                case WAITING_FOR_DEPLOY_DELAY:
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    if (ptoDeploymentTimer.milliseconds() >= PTO_DEPLOYMENT_DELAY_MS) {
                        currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_L_SERVO;
                    }
                    break;
                case DEPLOYING_L_SERVO:
                    h.ptoL.setPosition(PTO_L_ENGAGED_POSITION);
                    currentPtoDeploymentState = PtoDeploymentState.ENGAGED;
                    break;
                case ENGAGED:
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    h.ptoL.setPosition(PTO_L_ENGAGED_POSITION);
                    break;
                case RETRACTING_L_SERVO:
                    h.ptoL.setPosition(PTO_L_RETRACTED_POSITION);
                    currentPtoDeploymentState = PtoDeploymentState.WAITING_FOR_RETRACT_DELAY;
                    break;
                case WAITING_FOR_RETRACT_DELAY:
                    h.ptoL.setPosition(PTO_L_RETRACTED_POSITION);
                    if (ptoDeploymentTimer.milliseconds() >= PTO_DEPLOYMENT_DELAY_MS) {
                        currentPtoDeploymentState = PtoDeploymentState.RETRACTING_R_SERVO;
                    }
                    break;
                case RETRACTING_R_SERVO:
                    h.ptoR.setPosition(PTO_R_RETRACTED_POSITION);
                    currentPtoDeploymentState = PtoDeploymentState.RETRACTED;
                    break;
            }
            // --- END PTO Control Logic ---
            double distanceIn = h.revDist.getDistance(DistanceUnit.INCH);

            v.axial = -gamepad1.left_stick_y;
            v.lateral = gamepad1.left_stick_x;
            v.yawCmd = gamepad1.right_stick_x;

            double fl = v.axial + v.lateral + v.yawCmd;
            double fr = v.axial - v.lateral - v.yawCmd;
            double bl = v.axial - v.lateral + v.yawCmd;
            double br = v.axial + v.lateral - v.yawCmd;

            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            double lift = 0;
            if (gamepad1.touchpad) {
                lift = 1;
            } else {
                lift = 0;
            }

            if (lift == 1) {
                if (distanceIn < 15){
                    h.frontLeft.setPower(1);
                    h. backLeft.setPower(1);
                    h.frontRight.setPower(-1);
                    h.backRight.setPower( -1);
                }
                if (distanceIn < 16 && distanceIn > 15) {
                    h.frontLeft.setPower(.5);
                    h.backLeft.setPower(.5);
                    h.frontRight.setPower(-.5);
                    h.backRight.setPower(-.5);
                }
                if (distanceIn >= 16) {
                    currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_R_SERVO;
                }

            } else {

                h.frontLeft.setPower(fl / max);
                h.frontRight.setPower(fr / max);
                h.backLeft.setPower(bl / max);
                h.backRight.setPower(br / max);
            }


            h.pip.update();

            double robotX = h.pip.getPosX(DistanceUnit.INCH);
            double robotY = h.pip.getPosY(DistanceUnit.INCH);

            double xl = v.tx - robotX;
            double yl = v.ty - robotY;
            double hypot = Math.sqrt((xl * xl) + (yl * yl));

            // =========================================================================
            //  TURRET APPLICATION (UNWRAPPING BOUNDARY FIX)
            // =========================================================================
            double angleToGoal = Math.atan2(yl, xl);
            double robotHeading = h.pip.getHeading(AngleUnit.RADIANS);

            // Absolute target angle relative to the robot chassis
            // ADDED + Math.PI here to invert the facing direction by 180 degrees
            double targetTurretRad = angleToGoal - robotHeading + Math.PI;

            // unwrap to [-pi, pi]
            while (targetTurretRad > Math.PI) targetTurretRad -= 2 * Math.PI;
            while (targetTurretRad < -Math.PI) targetTurretRad += 2 * Math.PI;

            // Convert to degrees and apply center offset
            double baseServoDegrees = Math.toDegrees(targetTurretRad) - (314.6112145 / 2.0);

            // Gamepad trim logic
            left = gamepad2.dpad_left;
            right = gamepad2.dpad_right;

            aa = gamepad2.a;
            bb = gamepad2.b;

            if (left && !prevleft && !right) {
                samOffset = Range.clip(samOffset + 2.5, -40, 40);
            }
            if (right && !prevright && !left) {
                samOffset = Range.clip(samOffset - 2.5, -40, 40);
            }

            prevleft = left;
            prevright = right;

            if (aa && !prevaa && !bb) {
                samOffset = Range.clip(samOffset + 2.5, -40, 40);
            }
            if (bb && !prevbb && !aa) {
                samOffset = Range.clip(samOffset - 2.5, -40, 40);
            }

            prevaa = aa;
            prevbb = bb;

            double finalServoDegrees = baseServoDegrees + v.visionOffsetDeg + samOffset;

            /*if(h.pip.getHeading(AngleUnit.DEGREES) >= 110 || h.pip.getHeading(AngleUnit.DEGREES) <= 156){
                if (finalServoDegrees < 180){
                    h.frontLeft.setPower(1);
                    h.backLeft.setPower( 1);
                    h.frontRight.setPower(-1);
                    h.backRight.setPower(-1);

                } else {
                    h.frontLeft.setPower(-1);
                    h. backLeft.setPower(-1);
                    h.frontRight.setPower(1);
                    h.backRight.setPower(1);

                }
            }*/

            // Turret reset with gamepad1.ps || gamepad2.ps, but only if intake isn't using gamepad1.ps
            // Original code didn't have this condition, but it's good for prioritization.
            if ((gamepad1.ps && !psWasPressed) || gamepad2.ps) { // Added !psWasPressed to prioritize intake state change
                if (currentIntakeState != IntakeState.INTAKE_TILL_FULL_INIT &&
                        currentIntakeState != IntakeState.INTAKE_TILL_FULL_RUNNING) {
                    h.turret1.setPosition(0.5);
                    h.turret2.setPosition(0.5);
                }
            } else {
                h.turret1.setPosition(Math.abs((finalServoDegrees) / 314.6112145));
                h.turret2.setPosition(Math.abs((finalServoDegrees) / 314.6112145));
            }

            if (gamepad1.dpad_up) {
                h.pip.setPosition(new Pose2D(DistanceUnit.INCH, -68.02, 36, AngleUnit.DEGREES, 180));
                samOffset = 0;
            } else if (gamepad1.dpad_down) {
                h.pip.resetPosAndIMU();
                samOffset = 0;
            }


            // =========================================================================
            //  HOOD LINEAR REGRESSION
            // =========================================================================
            {
                if (hypot < 130) {

                double d1 = 53, d2 = 95;
                double v1 = .7, v2 = .2;
                double slope = (v2 - v1) / (d2 - d1);
                hpos = v1 + (slope * (hypot - d1));
                } else {
                    // Zone: Far
                    double d1 = 132.5, d2 = 158;
                    double v1 = 0, v2 = 0;
                    double slope = (v2 - v1) / (d2 - d1);
                    hpos = v1 + (slope * (hypot - d1));
                }

                h.hood.setPosition(Range.clip(hpos, 0, .7)); }

            // =========================================================================
            //  FLYWHEEL LINEAR REGRESSION
            // =========================================================================
            double vTarget;
            if (hypot < 130) {

            double d1 = 53, d2 = 95;
            double v1 = 1450, v2 = 1960;
            double slope = (v2 - v1) / (d2 - d1);
            vTarget = 1200 + (slope * (hypot - d1));
            } else {
                // Zone: Far
                double d1 = 132.5, d2 = 158;
                double v1 = 2330, v2 = 2430;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = 2300 + (slope * (hypot - d1));
            }
            vTarget = Range.clip(vTarget, 0, 5000); // Adjust max based on motor


            /* flywheel turn on/off */
            {
                if (gamepad1.x) {
                    v.var = 1;
                } else if (gamepad1.y) {
                    v.var = 0;
                }

                if (v.var == 1) {
                    h.flywheel1.setVelocity((vTarget * 37.333) / 60);
                    h.flywheel2.setVelocity((vTarget * 37.333) / 60);
                } else {
                    h.flywheel1.setVelocity(0);
                    h.flywheel2.setVelocity(0);
                }
            }









            telemetry.addData("Best Match front", frontResult != null ? frontResult.closestSwatch : "N/A");
            telemetry.addData("Best Match middle", middleResult != null ? middleResult.closestSwatch : "N/A");
            telemetry.addData("Best Match back", backResult != null ? backResult.closestSwatch : "N/A");
            if (frontResult != null) {
                telemetry.addLine("RGB   (" + JavaUtil.formatNumber(frontResult.RGB[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.RGB[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.RGB[2], 3, 0) + ")");
                telemetry.addLine("HSV   (" + JavaUtil.formatNumber(frontResult.HSV[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.HSV[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.HSV[2], 3, 0) + ")");
                telemetry.addLine("YCrCb (" + JavaUtil.formatNumber(frontResult.YCrCb[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.YCrCb[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.YCrCb[2], 3, 0) + ")");
            }

            telemetry.addData("distance in", distanceIn);

            telemetry.addData("x leg", xl);
            telemetry.addData("goalHeight leg", yl);
            telemetry.addData("hypot", hypot);
            telemetry.addData("RPM flywheel 1", "%.3f", ((h.flywheel1.getVelocity() * 60) / 37.333));
            telemetry.addData("RPM flywheel 2", "%.3f", ((h.flywheel2.getVelocity() * 60) / 37.333));
            telemetry.addData("vel ticks flywheel 1", "%.3f", (h.flywheel1.getVelocity()));
            telemetry.addData("vel ticks flywheel 2", "%.3f", (h.flywheel2.getVelocity()));
            telemetry.addData("pip x in", h.pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", h.pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip goalHeight in", h.pip.getPosY(DistanceUnit.INCH));
            telemetry.addData("turret1", h.turret1.getPosition());
            telemetry.addData("turret2", h.turret2.getPosition());
            telemetry.addData("finalServoDegrees", finalServoDegrees);
            telemetry.addData("baseServoDegrees", baseServoDegrees);
            telemetry.addData("PTO State", currentPtoDeploymentState.name());
            telemetry.addData("Intake State", currentIntakeState.name());
            if (currentIntakeState == IntakeState.INTAKE_TILL_PPG || currentIntakeState == IntakeState.INTAKE_TILL_PGP || currentIntakeState == IntakeState.INTAKE_TILL_GPP) {
                telemetry.addData("Cycle Sub-Step", cycleSubStep);
            }


            telemetry.update();
        }
    }
}