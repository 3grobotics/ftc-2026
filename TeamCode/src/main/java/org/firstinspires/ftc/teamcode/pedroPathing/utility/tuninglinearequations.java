package org.firstinspires.ftc.teamcode.pedroPathing.utility;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;
import org.firstinspires.ftc.teamcode.pedroPathing.ColorCamera2;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name="the opmode for tuning linear equations", group="linear equations test")
//@Disabled
public class tuninglinearequations extends LinearOpMode {
    public hardwareSubNewBot h;
    public varSub var;
    public Timer swingTimer;
    Servo t, t2, hood;
    DcMotorEx f1, f2;
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
        var = new varSub();
        swingTimer = new Timer();

        hood = hardwareMap.get(Servo.class, "hood");
        t = hardwareMap.get(Servo.class, "turret");
        t2 = hardwareMap.get(Servo.class, "turret2");
        f1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        f2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        hood.setDirection(Servo.Direction.REVERSE);
        f1.setDirection(DcMotorEx.Direction.REVERSE);
        f2.setDirection(DcMotorEx.Direction.REVERSE);

        double sp = 0.5;
        double v = 0;

        boolean prevxx = false;
        boolean prevbb = false;
        boolean prevaa = false;
        boolean prevyy = false;

        double target_y = 72;
        double target_x = -72;
        boolean swingHigh = false;

        GoBildaPinpointDriver.Register[] defaultRegisters = {
                GoBildaPinpointDriver.Register.DEVICE_STATUS,
                GoBildaPinpointDriver.Register.LOOP_TIME,
                GoBildaPinpointDriver.Register.X_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.Y_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.X_POSITION,
                GoBildaPinpointDriver.Register.Y_POSITION,
                GoBildaPinpointDriver.Register.H_ORIENTATION,
                GoBildaPinpointDriver.Register.X_VELOCITY,
                GoBildaPinpointDriver.Register.Y_VELOCITY,
                GoBildaPinpointDriver.Register.H_VELOCITY,
        };

        GoBildaPinpointDriver pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor indexer = hardwareMap.get(DcMotor.class, "indexer");
        Servo gate = hardwareMap.get(Servo.class, "gate");
        Servo swingArm = hardwareMap.get(Servo.class, "swingArm");


        pip.setOffsets(48.591, -129.909, DistanceUnit.MM);
        pip.setBulkReadScope(defaultRegisters);
        pip.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pip.resetPosAndIMU();
        //cc issue? VV
        hood.setDirection(Servo.Direction.REVERSE);
        t.setDirection(Servo.Direction.REVERSE);
        t2.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            t.setPosition(0.5);

            /*hood*/
            {
                boolean xx = gamepad1.x;
                boolean bb = gamepad1.b;

                if (xx && !prevxx && !bb) {
                    sp = Range.clip(sp - 0.1, 0, .7);
                }
                if (bb && !prevbb && !xx) {
                    sp = Range.clip(sp + 0.1, 0, .7);
                }

                // update prev AFTER using them
                prevxx = xx;
                prevbb = bb;

                hood.setPosition(sp);
            }

            /*flywheel*/
            {
                boolean aa = gamepad1.a;
                boolean yy = gamepad1.y;

                if (aa && !prevaa && !yy) {
                    v = Range.clip(v + 50, -7000, 7000.0);
                }
                if (yy && !prevyy && !aa) {
                    v = Range.clip(v - 50, -7000, 7000.0);
                }

                // update prev AFTER using them
                prevaa = aa;
                prevyy = yy;

                // apply velocity after updating v so it changes immediately
                f1.setVelocity(v);
                f2.setVelocity(v);
            }

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
        }


            pip.update();

            double robotX = -pip.getPosX(DistanceUnit.INCH);
            double robotY = pip.getPosY(DistanceUnit.INCH);

            double xl = target_x - pip.getPosX(DistanceUnit.INCH);
            double yl = target_y - pip.getPosY(DistanceUnit.INCH);
            double hypot = Math.sqrt((xl * xl) + (yl * yl));

            telemetry.addData("x leg", xl);
            telemetry.addData("y leg", yl);
            telemetry.addData("hypot", hypot);
            telemetry.addData("servo pos", sp);
            telemetry.addData("vels", v);
            telemetry.addData("RPM flywheel 1", "%.3f", ((f1.getVelocity() * 60) / 37.333));
            telemetry.addData("RPM flywheel 2", "%.3f", ((f2.getVelocity() * 60) / 37.333));
            telemetry.addData("vel ticks flywheel 1", "%.3f", (f1.getVelocity()));
            telemetry.addData("vel ticks flywheel 2", "%.3f", (f2.getVelocity()));
            telemetry.addData("pip x in", pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip y in", pip.getPosY(DistanceUnit.INCH));
            telemetry.update();
        }
    }


}