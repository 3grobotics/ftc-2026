package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.photon.PhotonCore;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.concurrent.ThreadLocalRandom;

@TeleOp(name = "ColorCamera 2 shoot while move")
public class ColorCamera2testing extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime();
    private Timer endgameTimer;
    private boolean lastResult = false;
    private int falseCount = 0;
    private int successStreak = 0;
    private double probabilityOfStreak = 1.0;
    private boolean lastBumperState = false; // For rising-edge detection

    // 1/6 chance is approximately 0.1667
    private final double SUCCESS_CHANCE = 1.0 / 6.0;
    public hardwareSubNewBot h;
    public varSub v;
    public Timer swingTimer;

    // --- PTO Control Variables (already a good state machine, kept as is) ---
    private boolean ptoButtonWasPressed = false; // For debouncing the gamepad button
    private boolean ptoIsEngaged = false;           // True if PTO is "on", false if "off"
    private final ElapsedTime ptoDeploymentTimer = new ElapsedTime();
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
    private final ElapsedTime intakeStateTimer = new ElapsedTime();
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
    public static double samOffset;

    /**
     * This OpMode illustrates how to use a video source (camera) as a color sensor
     */
    double tx = -72; // Target X
    double ty = 72;  // Target Y
    double t = 1;

    VisionPortal myVisionPortal;
    boolean all = false;

    @Override
    public void runOpMode() {

        /* these are the subsystems */
        {
            h = new hardwareSubNewBot(hardwareMap);
            v = new varSub();
        }

        /* these are the timers */
        {
            endgameTimer = new Timer();
        }


        /* telemetry stuff */
        {
            telemetry.setMsTransmissionInterval(50);
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
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
        //PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        //PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        //PhotonCore.PARALLELIZE_SERVOS = true;
        //PhotonCore.enable();

        float motif                         = 111;
        double robotX;
        double robotY;
        double xl;
        double yl;
        double hypot;

        double vx;
        double vy;
        double vTarget                   = 0;
        double fl                           ;
        double fr                           ;
        double bl                           ;
        double br                           ;
        double max                          ;

        boolean currentDpadLeft             ;
        boolean currentPs                   ;
        boolean currentRightBumper          ;
        boolean currentRightStickButton     ;
        boolean currentLeftStickButton      ;
        double lift                         ;
        double distanceIn                   ;
        double velocityreal                 ;
        double velDiff                      ;

        waitForStart();
        timer.reset();
        while (opModeIsActive()) {







            //telemetry.addData("photon volts aux", PhotonCore.CONTROL_HUB.getAuxiliaryVoltage(VoltageUnit.VOLTS));
            //telemetry.addData("photon amps ", PhotonCore.CONTROL_HUB.getCurrent(CurrentUnit.AMPS));
            //telemetry.addData("expansion photon amps ", PhotonCore.EXPANSION_HUB.getCurrent(CurrentUnit.AMPS));
            //telemetry.addData("total photon amps ", PhotonCore.CONTROL_HUB.getCurrent(CurrentUnit.AMPS) + PhotonCore.EXPANSION_HUB.getCurrent(CurrentUnit.AMPS));
            //telemetry.addData("photon info ", PhotonCore.CONTROL_HUB.getConnectionInfo());
            //PhotonCore.CONTROL_HUB.setConstant(8388736);
            //PhotonCore.EXPANSION_HUB.setConstant(8388736);


            /*try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }*/

            // =========================================================================
            // NEW INTAKE STATE MACHINE LOGIC
            // =========================================================================

            // Debounce gamepad1 buttons for intake control
             currentDpadLeft         = gamepad1.dpad_left;
             currentPs               = gamepad1.ps;
             currentRightBumper      = gamepad1.right_bumper;
             currentRightStickButton = gamepad1.right_stick_button;
             currentLeftStickButton  = gamepad1.left_stick_button;

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
                    h.sickle.setPosition(.85);
                    // h.swingArm.setPosition(1.0); // Allow swingArm to retain its last position when idle unless another state sets it
                    h.gate.setPosition(0.65); // Default closed/intake position
                    break;

                case MANUAL_TRIGGERS_ACTIVE:
                    double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                    h.intake.setPower(-intakeCmd);
                    h.indexer.setPower(-intakeCmd);
                    h.swingArm.setPosition(.65); // Move swing arm for manual intake
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
                    h.sickle.setPosition(0.7);
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
                        h.sickle.setPosition(.75);
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

            if(gamepad1.ps){
                h.nautR.setPosition(.1);
                h.nautL.setPosition(.1);
            } else if (gamepad1.options){
                h.nautR.setPosition(.6);
                h.nautL.setPosition(.6);
            }

            switch (currentPtoDeploymentState) {
                case RETRACTED:
                    h.ptoR.setPosition(PTO_R_RETRACTED_POSITION);
                    h.ptoL.setPosition(PTO_L_RETRACTED_POSITION);
                    h.nautR.setPosition(.1);
                    h.nautL.setPosition(.1);
                    break;
                case DEPLOYING_R_SERVO:
                    h.nautR.setPosition(.6);
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_L_SERVO;
                    break;
                case WAITING_FOR_DEPLOY_DELAY:
                    h.ptoR.setPosition(PTO_R_ENGAGED_POSITION);
                    h.nautR.setPosition(.6);
                    if (ptoDeploymentTimer.milliseconds() >= PTO_DEPLOYMENT_DELAY_MS) {
                        currentPtoDeploymentState = PtoDeploymentState.DEPLOYING_L_SERVO;
                    }
                    break;
                case DEPLOYING_L_SERVO:
                    h.nautL.setPosition(.6);
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


             v.axial = -gamepad1.left_stick_y;
             v.lateral = gamepad1.left_stick_x;
             v.yawCmd = gamepad1.right_stick_x;

             fl = v.axial + v.lateral + v.yawCmd;
             fr = v.axial - v.lateral - v.yawCmd;
             bl = v.axial - v.lateral + v.yawCmd;
             br = v.axial + v.lateral - v.yawCmd;

             max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

             h.frontLeft.setPower(fl / max);
             h.frontRight.setPower(fr / max);
             h.backLeft.setPower(bl / max);
             h.backRight.setPower(br / max);



            vy                         = h.pip.getVelY(DistanceUnit.INCH);
            vx                         = h.pip.getVelX(DistanceUnit.INCH);


            tx                         = -72 - (vx * t); // 72 or tx maybe
            ty                         =  72 - (vy * t); // 72 or tx maybe


            h.pip.update(); // leftdate position first!

            robotX                     = h.pip.getPosX(DistanceUnit.INCH);
            robotY                     = h.pip.getPosY(DistanceUnit.INCH);

            xl                         = tx - robotX;
            yl                         = ty - robotY;
            hypot                      = Math.sqrt((xl * xl) + (yl * yl));


            if ( hypot < 89){
                t                      =  0.004 * hypot + 0.468;
            } else if ( hypot > 89 && hypot < 121)  {
                t                      =  0.833;
            } else if ( hypot > 121) {
                t                      = -0.004 * hypot + 1.317;
            }


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
            //  FLYWHEEL LINEAR REGRESSION
            // =========================================================================

            if (hypot < 75) {

                vTarget = 9.938 * hypot + 1022.646;
                /*double d1 = 53, d2 = 95;
                double v1 = 1450, v2 = 1960;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = 1200 + (slope * (hypot - d1));*/
            } else if(hypot > 75 && hypot < 96.3){
                vTarget = 12.066 * hypot + 863.05;
                // Zone: Far
                /*double d1 = 132.5, d2 = 158;
                double v1 = 2330, v2 = 2430;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = 2300 + (slope * (hypot - d1));*/
            } else if(hypot > 96.3 && hypot < 129) {
                vTarget = 9.817 * hypot + 1079.623;
            } else if(hypot > 151) {
                vTarget = 11.727 * hypot + 833.217;
            }


            vTarget = Range.clip(vTarget, 0, 6000); // Adjust max based on motor

            /* flywheel turn on/off */
            {
                if (gamepad1.x) {
                    v.var = 1;
                } else if (gamepad1.y) {
                    v.var = 0;
                }

                if (v.var == 1) {
                    h.flywheel1.setVelocity(((vTarget) * 37.33) / 60);
                    h.flywheel2.setVelocity(((vTarget) * 37.33) / 60);


                    /*
                    h.flywheel1.setVelocity((vTarget * 37.333) / 60);
                    h.flywheel2.setVelocity((vTarget * 37.333) / 60);*/
                } else {
                    h.flywheel1.setVelocity(0);
                    h.flywheel2.setVelocity(0);
                }
            }
            // =========================================================================
            //  HOOD LINEAR REGRESSION
            // =========================================================================

            if (hypot < 75) {

                hpos = -0.006 * hypot + 1.053;
                    /*double d1 = 53, d2 = 95;
                    double v1 = .7, v2 = .2;
                    double slope = (v2 - v1) / (d2 - d1);
                    hpos = v1 + (slope * (hypot - d1));*/
            } else if(hypot > 75 && hypot < 96.3){
                hpos = -0.005 * hypot + 0.975;
                // Zone: Far
                    /*double d1 = 132.5, d2 = 158;
                    double v1 = 0, v2 = 0;
                    double slope = (v2 - v1) / (d2 - d1);
                    hpos = v1 + (slope * (hypot - d1));*/
            } else if(hypot > 96.3 && hypot < 129){
                hpos = -0.015 * hypot + 1.944;
            } else if(hypot > 151){
                hpos = 0;
            }

             velocityreal = (h.flywheel1.getVelocity() + h.flywheel2.getVelocity() ) / 2;
             velDiff      = vTarget - velocityreal;
             /* make velocityreal degrees then
             *  somehow get vTarget in degrees
             *  then set servo like regular*
             *  based off distance but take off
             *  however much veldiff is
             *
             *  *in degrees but servopos if somehow i can do that
             */
            hpos = Range.clip(hpos, 0, .7);
            h.hood.setPosition(hpos);
           // h.hood.setPosition(hpos - velDiff);



            if (endgameTimer.getElapsedTimeSeconds() > 90 || gamepad2.options){
                
                
                
                PredominantColorProcessor.Builder frontProcessorBuilder;
                PredominantColorProcessor.Builder backProcessorBuilder;
                VisionPortal.Builder myVisionPortalBuilder;
                PredominantColorProcessor frontPredominantColorProcessor;
                PredominantColorProcessor backPredominantColorProcessor;
                VisionPortal myVisionPortal;
                PredominantColorProcessor.Result frontResult;
                PredominantColorProcessor.Result backResult;


                // Build a "Color Sensor" vision processor based on the PredominantColorProcessor class.
                frontProcessorBuilder = new PredominantColorProcessor.Builder();
                backProcessorBuilder = new PredominantColorProcessor.Builder();
                /* - Focus the color sensor by defining a RegionOfInterest (ROI) which you want to inspect.
                     This can be the entire frame, or a sub-region defined using:
                     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
                     Use one form of the ImageRegion class to define the ROI.
                 100x100 pixel square near the upper left corner*/
                frontProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(
                        80,
                        200,
                        250,
                        400));
                backProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(
                        600,
                        100,
                        800,
                        300));

                 /* - Set the list of "acceptable" color swatches (matches).
                      Only colors that you assign here will be returned.
                      If you know the sensor will be pointing to one of a few specific colors, enter them here.
                      Or, if the sensor may be pointed randomly, provide some additional colors that may match the surrounding.
                      Note that in the example shown below, only some of the available colors are included.
                      This will force any other colored region into one of these colors.
                      eg: Green may be reported as YELLOW, as this may be the "closest" match.*/

                frontProcessorBuilder.setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE);
                frontPredominantColorProcessor = frontProcessorBuilder.build();


                backProcessorBuilder.setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE);
                backPredominantColorProcessor = backProcessorBuilder.build();

                // Build a vision portal to run the Color Sensor process.
                myVisionPortalBuilder = new VisionPortal.Builder();
                //  - Add the colorSensor process created above.
                myVisionPortalBuilder.addProcessor(frontPredominantColorProcessor);
                myVisionPortalBuilder.addProcessor(backPredominantColorProcessor);
                //  - Set the desired video resolution.
                //      Since a high resolution will not improve this process, choose a lower resolution that is
                //      supported by your camera. This will improve overall performance and reduce latency.
                // Set the stream format.
                myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
                myVisionPortalBuilder.setCameraResolution(new Size(800, 448));
                //  - Choose your video source. This may be for a webcam or for a Phone Camera.


                myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


                //myVisionPortal = myVisionPortalBuilder.build();


                //frontResult = frontPredominantColorProcessor.getAnalysis();
                //backResult = backPredominantColorProcessor.getAnalysis();


                if (gamepad1.touchpad) {
                    lift = 1;
                } else {
                    lift = 0;
                }
                distanceIn = h.revDist.getDistance(DistanceUnit.INCH);
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

                }

                if(h.topDistSensor.getState() == true && h.midDistSensor.getState() == true && h.frontDistSensor.getState() == true){
                    all = true;
                } else {
                    all = false;
                }


                // cycles till ppg
                if (gamepad2.a == true) {
                    while (opModeIsActive() && !isStopRequested()) {
                        frontResult = frontPredominantColorProcessor.getAnalysis(); // refresh camera result

                        if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) {
                            break; // stop when green
                        }

                        h.sickle.setPosition(0.85);
                        h.swingArm.setPosition(0.5);
                        h.gate.setPosition(0.82);
                        sleep(250);
                        h.intake.setPower(-1);
                        h.indexer.setPower(-1);
                        sleep(250);
                        h.sickle.setPosition(0.9);
                        sleep(250);
                        h.gate.setPosition(0.85);
                        h.swingArm.setPosition(0.2);
                        h.intake.setPower(0);
                        sleep(250);
                        h.intake.setPower(0.5);
                        h.indexer.setPower(-1);
                        sleep(200);
                        h.indexer.setPower(1);
                        sleep(100);
                        h.gate.setPosition(0.65);
                        h.indexer.setPower(0);
                        h.swingArm.setPosition(0.5);
                        h.intake.setPower(-1);
                        sleep(100);
                        h.intake.setPower(0);

                        h.swingArm.setPosition(1);
                        sleep(500);

                    }

                }
                Gamepad.LedEffect motif112Effect = new Gamepad.LedEffect.Builder()
                        .addStep(255, 0, 255, 400) //purple
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 255, 400) //purple
                        .addStep(0, 0, 0, 100)
                        .addStep(0, 255, 0, 400)   //green
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 0, 500)   //red
                        .addStep(0, 0, 0, 400)     //pause
                        .setRepeating(true)
                        .build();

                Gamepad.LedEffect motif121Effect = new Gamepad.LedEffect.Builder()
                        .addStep(255, 0, 255, 400) //purple
                        .addStep(0, 0, 0, 100)
                        .addStep(0, 255, 0, 400)   //green
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 255, 400) //purple
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 0, 500)   //red
                        .addStep(0, 0, 0, 400)     //pause
                        .setRepeating(true)
                        .build();

                Gamepad.LedEffect motif211Effect = new Gamepad.LedEffect.Builder()
                        .addStep(0, 255, 0, 400)   //green
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 255, 400) //purple
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 255, 400) //purple
                        .addStep(0, 0, 0, 100)
                        .addStep(255, 0, 0, 500)   //red
                        .addStep(0, 0, 0, 400)     //pause
                        .setRepeating(true)
                        .build();

                if (gamepad2.left_bumper){

                    if (gamepad2.x){
                        motif = 211;
                        gamepad2.runLedEffect(motif211Effect);
                    } else if (gamepad2.a){
                        motif = 121;
                        gamepad2.runLedEffect(motif121Effect);
                    } else if (gamepad2.b){
                        motif = 112;
                        gamepad2.runLedEffect(motif112Effect);
                    }

                }

                // motif + 0
                if (gamepad2.x && !gamepad2.left_bumper){
                    if (motif == 211 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            frontResult = frontPredominantColorProcessor.getAnalysis();
                            backResult = backPredominantColorProcessor.getAnalysis();

                            if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) || gamepad2.touchpadWasPressed()) {
                                break; // stop when green
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);
                        }
                    }

                    if (motif == 121 && all){
                        // sort to 121
                        while (opModeIsActive() && !isStopRequested()) {
                            backResult = backPredominantColorProcessor.getAnalysis(); // refresh camera

                            if (backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) || gamepad2.touchpadWasPressed()) {
                                break; // stop when green
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);
                        }
                    }

                    if (motif == 112 && all){
                        // sort to 112
                        while (opModeIsActive() && !isStopRequested()) {
                            frontResult = frontPredominantColorProcessor.getAnalysis(); // refresh camera result

                            if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) || gamepad2.touchpadWasPressed()) {
                                break; // stop when green
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);

                        }
                    }


                }

                // motif + 1
                if (gamepad2.a && !gamepad2.left_bumper){
                    if (motif == 211 && all){
                        // sort to 121
                        while (opModeIsActive() && !isStopRequested()) {
                            backResult = backPredominantColorProcessor.getAnalysis(); // refresh camera

                            if (backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) || gamepad2.touchpadWasPressed()) {
                                break; // stop when green
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);
                        }
                    }

                    if (motif == 121 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            frontResult = frontPredominantColorProcessor.getAnalysis(); // refresh camera result

                            if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) || gamepad2.touchpadWasPressed()) {
                                break; // stop when green
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);

                        }
                    }

                    if (motif == 112 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            frontResult = frontPredominantColorProcessor.getAnalysis();
                            backResult = backPredominantColorProcessor.getAnalysis();

                            if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) || gamepad2.touchpadWasPressed()) {
                                break; // stop when green
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);
                        }
                    }
                }

                // motif + 2
                if (gamepad2.b  && !gamepad2.left_bumper){
                    if (motif == 211 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            frontResult = frontPredominantColorProcessor.getAnalysis(); // refresh camera result

                            if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) || gamepad2.touchpadWasPressed()) {
                                break; // stop when green
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);

                        }
                    }

                    if (motif == 121 && all){
                        while (opModeIsActive() && !isStopRequested()) {
                            frontResult = frontPredominantColorProcessor.getAnalysis();
                            backResult = backPredominantColorProcessor.getAnalysis();

                            if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) || gamepad2.touchpadWasPressed()) {
                                break; // stop when green
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);
                        }
                    }

                    if (motif == 112 && all){
                        // sort to 121
                        while (opModeIsActive() && !isStopRequested()) {
                            backResult = backPredominantColorProcessor.getAnalysis(); // refresh camera

                            if (backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) || gamepad2.touchpadWasPressed()) {
                                break; // stop when green
                            }

                            h.sickle.setPosition(0.85);
                            h.swingArm.setPosition(0.5);
                            h.gate.setPosition(0.82);
                            sleep(250);
                            h.intake.setPower(-1);
                            h.indexer.setPower(-1);
                            sleep(250);
                            h.sickle.setPosition(0.9);
                            sleep(250);
                            h.gate.setPosition(0.85);
                            h.swingArm.setPosition(0.2);
                            h.intake.setPower(0);
                            sleep(250);
                            h.intake.setPower(0.5);
                            h.indexer.setPower(-1);
                            sleep(200);
                            h.indexer.setPower(1);
                            sleep(100);
                            h.gate.setPosition(0.65);
                            h.indexer.setPower(0);
                            h.swingArm.setPosition(0.5);
                            h.intake.setPower(-1);
                            sleep(100);
                            h.intake.setPower(0);

                            h.swingArm.setPosition(1);
                            sleep(500);
                        }
                    }
                }
            }








           // telemetry.addData("distance in", distanceIn);

            //telemetry.addData("x leg", xl);
            //telemetry.addData("y leg", yl);
            //telemetry.addData("hypot", hypot);
            //telemetry.addData("t", t);
            //telemetry.addData("vel x", vx);
            //telemetry.addData("vel y", vy);
            //telemetry.addData("tx", tx);
            //telemetry.addData("ty", ty);
            //telemetry.addData("vTarget", vTarget);
            telemetry.addData("hpos", hpos);
            telemetry.addData("RPM flywheel 1", "%.3f", ((h.flywheel1.getVelocity() * 60) / 37.333));
            telemetry.addData("RPM flywheel 2", "%.3f", ((h.flywheel2.getVelocity() * 60) / 37.333));
            //telemetry.addData("vel ticks flywheel 1", "%.3f", (h.flywheel1.getVelocity()));
            //telemetry.addData("vel ticks flywheel 2", "%.3f", (h.flywheel2.getVelocity()));
            telemetry.addData("pip x in", h.pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", h.pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip y in", h.pip.getPosY(DistanceUnit.INCH));
            //telemetry.addData("turret1", h.turret1.getPosition());
            //telemetry.addData("turret2", h.turret2.getPosition());
            telemetry.addData("finalServoDegrees", finalServoDegrees);
            //telemetry.addData("baseServoDegrees", baseServoDegrees);
            telemetry.addData("PTO State", currentPtoDeploymentState.name());
            //telemetry.addData("Intake State", currentIntakeState.name());
            if (currentIntakeState == IntakeState.INTAKE_TILL_PPG || currentIntakeState == IntakeState.INTAKE_TILL_PGP || currentIntakeState == IntakeState.INTAKE_TILL_GPP) {
                telemetry.addData("Cycle Sub-Step", cycleSubStep);
            }

            telemetry.update();
        }
    }
}