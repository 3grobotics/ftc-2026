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
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name="velocity compensation test no ai", group="Linear OpMode")
//@Disabled
public class velCompensationTestNoAI extends LinearOpMode {
    
    hardwareSubNewBot h;
    
    varSub v;
    double turretAngle = .5;
    double flywheelSpeed = 1;
    double hoodAngle = 0.7;
    double radicand1;
    double x;
    double xl;
    double yl;
    double hypot;
    double robotX;
    double robotY;
    private IntakeState currentIntakeState = IntakeState.IDLE;
    private ElapsedTime intakeStateTimer = new ElapsedTime();
    private int cycleSubStep = 0; // Used for multi-step cycles like single-note and till-color

    boolean aa = false;
    boolean bb = false;
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

    double mult = 5;

    boolean prevaa = false;
    boolean prevbb = false;
    @Override
    public void runOpMode() {
        h = new hardwareSubNewBot(hardwareMap);
        v = new varSub();

        h.pip.resetPosAndIMU();

        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;

        boolean aa = false;
        boolean bb = false;
        boolean prevaa = aa;
        boolean prevbb = bb;

        double samOffset = 0;

        waitForStart();

        while (opModeIsActive()) {

            // Debounce gamepad1 buttons for intake control
            boolean currentDpadLeft = gamepad1.dpad_left;
            boolean currentPs = gamepad1.ps;
            boolean currentRightBumper = gamepad1.right_bumper;
            boolean currentRightStickButton = gamepad1.right_stick_button;
            boolean currentLeftStickButton = gamepad1.left_stick_button;

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

            }

            vectorAndTurretUpdate(h.pip.getHeading(AngleUnit.RADIANS));


            while (turretAngle > Math.PI) turretAngle -= 2 * Math.PI;
            while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;

            // Convert to degrees and apply center offset
            double baseServoDegrees = (turretAngle) - (314.6112145 / 2.0);

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

            double finalServoDegrees = turretAngle + samOffset;

            //h.turret1.setPosition(Math.abs((finalServoDegrees) / 314.6112145));
            //h.turret2.setPosition(Math.abs((finalServoDegrees) / 314.6112145));

            telemetry.addData("turretAngle", turretAngle);
            telemetry.addData("x", robotX);
            telemetry.addData("y", robotY);
            telemetry.addData("theta", velocityVarSub.theta);
            telemetry.addData("(finalServoDegrees) / 314.6112145)", (Math.abs((finalServoDegrees) / 314.6112145)));
            telemetry.addData("hood angle", ((0.959931 - 0.558505) / (.7 - 0)) * hoodAngle);
            telemetry.addData("flywheel speed", flywheelSpeed);
            telemetry.addData("radicand1", radicand1);
            telemetry.addData("hypot", hypot);
            telemetry.addData("mult", mult);
            telemetry.update();









            if (gamepad1.x) {
                v.var = 1;
            } else if (gamepad1.y) {
                v.var = 0;
            }

            if (v.var == 1) {
                h.flywheel1.setVelocity(flywheelSpeed);
                h.flywheel2.setVelocity(flywheelSpeed);
            } else {
                h.flywheel1.setVelocity(0);
                h.flywheel2.setVelocity(0);
            }





            h.hood.setPosition(getHoodTicksFromDegrees(Math.toDegrees(hoodAngle)));





            h.pip.update();
        }
    }

    private void vectorAndTurretUpdate(double robotHeading){



        // private Vector????
        robotX = -h.pip.getPosX(DistanceUnit.INCH);
        robotY = h.pip.getPosY(DistanceUnit.INCH);

        xl = v.tx - robotX;
        yl = v.ty - robotY;

        hypot  = Math.sqrt((xl * xl) + (yl * yl));

        double angleToGoal = Math.atan2(yl, xl);


        x = hypot;

        hoodAngle = MathFunctions.clamp(Math.atan(((velocityVarSub.goalHeight * 2) / x ) - (Math.tan(velocityVarSub.theta))) , 0.558505,0.959931);



        radicand1 = Math.sqrt((velocityVarSub.g * x * x) / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - velocityVarSub.goalHeight)));

        if (radicand1 > 0) {
            //flywheelSpeed = Math.sqrt(radicand1);
            flywheelSpeed = getFlywheelTicksFromVelocity(radicand1, hypot);
        } else {
            flywheelSpeed = 0; // Or a reasonable default
        }

        /*double parallelComponent = h.pip.getVelY(DistanceUnit.INCH);
        double perpendicularComponent = h.pip.getVelX(DistanceUnit.INCH);

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double denominatorTime = flywheelSpeed * Math.cos(hoodAngle);
        double time = (denominatorTime != 0) ? x / denominatorTime : 0;
        double ivr = x / time * parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        hoodAngle = MathFunctions.clamp(Math.atan2(vz, nvr), 0.558505, 0.959931);

        double radicand2 = (velocityVarSub.g * ndr * ndr) / ((2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - velocityVarSub.goalHeight)));

        if (radicand2 > 0) {
            flywheelSpeed = Math.sqrt(radicand2);
        } else {
            // If the compensated version fails, keep the old speed or set to 0
            flywheelSpeed = 0;
        }
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        turretAngle = robotHeading - angleToGoal + turretVelCompOffset;*/







    }

    public  double getFlywheelTicksFromVelocity(double velocity, double x){


        aa = gamepad1.a;
        bb = gamepad1.b;

        if (aa && !prevaa && !bb) {
            mult = Range.clip(mult - 0.01, 0, 10);
        }
        if (bb && !prevbb && !aa) {
            mult = Range.clip(mult + 0.01, 0, 10);
        }

        // update prev AFTER using them
        prevaa = aa;
        prevbb = bb;

       
        
        if(x > 101){
            return velocity * (mult);
        } else if (x > 76){
            return velocity * (mult);
        } else if(x > 50){
            return velocity * (mult);
        }
        
        
        
        return velocity * (mult);
        //return velocity * (6.636705202);

    }
    public static double getHoodTicksFromDegrees(double degrees){
        return 0.0304 * degrees - .973913;
    }
}