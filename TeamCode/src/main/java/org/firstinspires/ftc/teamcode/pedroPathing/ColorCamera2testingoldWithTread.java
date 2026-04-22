package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
import org.firstinspires.ftc.teamcode.Subsystems.turretSub;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name = "ColorCamera 2 shoot while move")
public class ColorCamera2testingoldWithTread extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime();
    private turretSub turret;

    public hardwareSubNewBot h;
    public varSub v;
    public Timer swingTimer;

    private Thread upperThread = null;
    private volatile boolean runThread = false;

    // --- PTO Control Variables (already a good state machine, kept as is) ---
    private boolean ptoButtonWasPressed = false;
    private boolean ptoIsEngaged = false;
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

    // --- NEW Intake Control Variables ---
    private IntakeState currentIntakeState = IntakeState.IDLE;
    private final ElapsedTime intakeStateTimer = new ElapsedTime();
    private int cycleSubStep = 0;

    enum IntakeState {
        IDLE,
        MANUAL_TRIGGERS_ACTIVE,
        GATE_MANUAL_CONTROL,

        SINGLE_NOTE_CYCLE_INIT,
        SINGLE_NOTE_CYCLE_STEP_1,
        SINGLE_NOTE_CYCLE_STEP_2,
        SINGLE_NOTE_CYCLE_STEP_3,
        SINGLE_NOTE_CYCLE_STEP_4,
        SINGLE_NOTE_CYCLE_STEP_5,
        SINGLE_NOTE_CYCLE_STEP_6,
        SINGLE_NOTE_CYCLE_STEP_7,
        SINGLE_NOTE_CYCLE_STEP_8,

        FIRING,
        FIRING_COMPLETE,

        INTAKE_TILL_FULL_INIT,
        INTAKE_TILL_FULL_RUNNING,

        INTAKE_TILL_COLOR_INIT,
        INTAKE_TILL_COLOR_SWING_RESET,
        INTAKE_TILL_COLOR_STEP_1,
        INTAKE_TILL_COLOR_STEP_2,
        INTAKE_TILL_COLOR_STEP_3,
        INTAKE_TILL_COLOR_STEP_4,
        INTAKE_TILL_COLOR_STEP_5,
        INTAKE_TILL_COLOR_STEP_6,
        INTAKE_TILL_COLOR_STEP_7,
        INTAKE_TILL_COLOR_STEP_8,

        INTAKE_TILL_PPG,
        INTAKE_TILL_PGP,
        INTAKE_TILL_GPP
    }

    private boolean dpadLeftWasPressed = false;
    private boolean psWasPressed = false;
    private boolean rightBumperWasPressed = false;
    private boolean leftStickButtonWasPressed = false;
    private boolean rightStickButtonWasPressed = false;

    double hpos;
    public static double samOffset;

    double tx = -72;
    double ty = 72;
    double t = 1;
    public static int var = 0;

    VisionPortal myVisionPortal;
    @Override
    public void runOpMode() {

        /* these are the subsystems */
        {
            h = new hardwareSubNewBot(hardwareMap);
            v = new varSub();
            turret = new turretSub(hardwareMap);

            // LINKING THE SUBSYSTEMS TO THE THREAD
            turret.h = h;
            turret.v = v;
        }

        /* these are the timers */
        {
            swingTimer = new Timer();
        }

        /* this is the camera stuff */
        PredominantColorProcessor.Builder frontProcessorBuilder;
        PredominantColorProcessor.Builder middleProcessorBuilder;
        PredominantColorProcessor.Builder backProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        PredominantColorProcessor frontPredominantColorProcessor;
        PredominantColorProcessor middlePredominantColorProcessor;
        PredominantColorProcessor backPredominantColorProcessor;

        PredominantColorProcessor.Result frontResult;
        PredominantColorProcessor.Result middleResult ;
        PredominantColorProcessor.Result backResult   ;

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

        h.topDistSensor.setMode(DigitalChannel.Mode.INPUT);
        h.midDistSensor.setMode(DigitalChannel.Mode.INPUT);
        h.frontDistSensor.setMode(DigitalChannel.Mode.INPUT);

        h.sickle.setPosition(1.0);
        h.gate.setPosition(0.65);
        h.swingArm.setPosition(1.0);
        h.intake.setPower(0);
        h.indexer.setPower(0);


        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;

        boolean aa = gamepad2.a;
        boolean bb = gamepad2.b;
        boolean prevaa = aa;
        boolean prevbb = bb;
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        PhotonCore.PARALLELIZE_SERVOS = true;
        PhotonCore.enable();

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

        if(isStopRequested()) return;

        startTurretControlThread();
        timer.reset();

        while (opModeIsActive()) {

            h.pip.update();
            turret.runTurret();

            telemetry.addData("photon volts aux", PhotonCore.CONTROL_HUB.getAuxiliaryVoltage(VoltageUnit.VOLTS));
            telemetry.addData("photon amps ", PhotonCore.CONTROL_HUB.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("expansion photon amps ", PhotonCore.EXPANSION_HUB.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("total photon amps ", PhotonCore.CONTROL_HUB.getCurrent(CurrentUnit.AMPS) + PhotonCore.EXPANSION_HUB.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("photon info ", PhotonCore.CONTROL_HUB.getConnectionInfo());

            frontResult = frontPredominantColorProcessor.getAnalysis();
            middleResult = middlePredominantColorProcessor.getAnalysis();
            backResult = backPredominantColorProcessor.getAnalysis();

            // =========================================================================
            // NEW INTAKE STATE MACHINE LOGIC
            // =========================================================================

            currentDpadLeft         = gamepad1.dpad_left;
            currentPs               = gamepad1.ps;
            currentRightBumper      = gamepad1.right_bumper;
            currentRightStickButton = gamepad1.right_stick_button;
            currentLeftStickButton  = gamepad1.left_stick_button;

            if (currentIntakeState == IntakeState.IDLE || currentIntakeState == IntakeState.MANUAL_TRIGGERS_ACTIVE || currentIntakeState == IntakeState.GATE_MANUAL_CONTROL) {
                if (currentLeftStickButton && currentRightStickButton && !leftStickButtonWasPressed && !rightStickButtonWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_GPP;
                    cycleSubStep = 0;
                    intakeStateTimer.reset();
                } else if (currentRightStickButton && !rightStickButtonWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_PPG;
                    cycleSubStep = 0;
                    intakeStateTimer.reset();
                } else if (currentLeftStickButton && !leftStickButtonWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_PGP;
                    cycleSubStep = 0;
                    intakeStateTimer.reset();
                } else if (currentPs && !psWasPressed) {
                    currentIntakeState = IntakeState.INTAKE_TILL_FULL_INIT;
                    intakeStateTimer.reset();
                } else if (currentDpadLeft && !dpadLeftWasPressed) {
                    currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_INIT;
                    intakeStateTimer.reset();
                } else if (currentRightBumper && !rightBumperWasPressed) {
                    currentIntakeState = IntakeState.FIRING;
                } else {
                    double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                    if (Math.abs(intakeCmd) > 0.1) {
                        currentIntakeState = IntakeState.MANUAL_TRIGGERS_ACTIVE;
                    } else if (gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_up) {
                        currentIntakeState = IntakeState.GATE_MANUAL_CONTROL;
                    } else if (currentIntakeState != IntakeState.IDLE) {
                        currentIntakeState = IntakeState.IDLE;
                    }
                }
            }
            dpadLeftWasPressed = currentDpadLeft;
            psWasPressed = currentPs;
            rightBumperWasPressed = currentRightBumper;
            rightStickButtonWasPressed = currentRightStickButton;
            leftStickButtonWasPressed = currentLeftStickButton;


            switch (currentIntakeState) {
                case IDLE:
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                    h.sickle.setPosition(.85);
                    h.gate.setPosition(0.65);
                    break;

                case MANUAL_TRIGGERS_ACTIVE:
                    double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                    h.intake.setPower(-intakeCmd);
                    h.indexer.setPower(-intakeCmd);
                    h.swingArm.setPosition(.65);
                    h.gate.setPosition(.65);

                    if (Math.abs(intakeCmd) < 0.1) {
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;

                case GATE_MANUAL_CONTROL:
                    if (gamepad1.dpad_right) {
                        h.gate.setPosition(0.8);
                    } else if (gamepad1.dpad_down) {
                        h.gate.setPosition(0.65);
                    } else if (gamepad1.dpad_up) {
                        h.gate.setPosition(1);
                    } else {
                        currentIntakeState = IntakeState.IDLE;
                    }
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
                    if (intakeStateTimer.milliseconds() > 500) {
                        h.sickle.setPosition(.75);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_3;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_3:
                    if (intakeStateTimer.milliseconds() > 750) {
                        h.gate.setPosition(0.85);
                        h.swingArm.setPosition(0.2);
                        h.intake.setPower(0);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_4;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_4:
                    if (intakeStateTimer.milliseconds() > 1000) {
                        h.intake.setPower(0.5);
                        h.indexer.setPower(-1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_5;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_5:
                    if (intakeStateTimer.milliseconds() > 1200) {
                        h.indexer.setPower(1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_6;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_6:
                    if (intakeStateTimer.milliseconds() > 1300) {
                        h.gate.setPosition(0.65);
                        h.indexer.setPower(0);
                        h.swingArm.setPosition(0.55);
                        h.intake.setPower(-1);
                        currentIntakeState = IntakeState.SINGLE_NOTE_CYCLE_STEP_7;
                    }
                    break;
                case SINGLE_NOTE_CYCLE_STEP_7:
                    if (intakeStateTimer.milliseconds() > 1400) {
                        h.intake.setPower(0);
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;

                case FIRING:
                    h.gate.setPosition(.97);
                    h.indexer.setPower(-1);
                    h.intake.setPower(-1);
                    h.swingArm.setPosition(.95);
                    if (!gamepad1.right_bumper) {
                        currentIntakeState = IntakeState.FIRING_COMPLETE;
                        intakeStateTimer.reset();
                    }
                    break;
                case FIRING_COMPLETE:
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                    h.swingArm.setPosition(1.0);
                    h.gate.setPosition(0.65);
                    if (intakeStateTimer.milliseconds() > 200) {
                        currentIntakeState = IntakeState.IDLE;
                    }
                    break;

                case INTAKE_TILL_FULL_INIT:
                    h.gate.setPosition(0.65);
                    h.intake.setPower(-1);
                    h.indexer.setPower(-1);
                    h.sickle.setPosition(1.0);
                    h.swingArm.setPosition(0.6);
                    currentIntakeState = IntakeState.INTAKE_TILL_FULL_RUNNING;
                    break;
                case INTAKE_TILL_FULL_RUNNING:
                    if (h.topDistSensor.getState() && h.midDistSensor.getState() && h.frontDistSensor.getState()) {
                        h.intake.setPower(0);
                        h.indexer.setPower(0);
                        currentIntakeState = IntakeState.IDLE;
                    } else {
                        h.intake.setPower(-1);
                        h.indexer.setPower(-1);
                    }
                    break;

                case INTAKE_TILL_PPG:
                case INTAKE_TILL_PGP:
                case INTAKE_TILL_GPP:
                    PredominantColorProcessor.Swatch targetSwatch = PredominantColorProcessor.Swatch.ARTIFACT_GREEN;
                    PredominantColorProcessor.Result checkResult;

                    if (currentIntakeState == IntakeState.INTAKE_TILL_PPG) {
                        checkResult = frontResult;
                    } else if (currentIntakeState == IntakeState.INTAKE_TILL_PGP) {
                        checkResult = middleResult;
                    } else {
                        checkResult = backResult;
                    }

                    if (checkResult != null && checkResult.closestSwatch.equals(targetSwatch)) {
                        h.intake.setPower(0);
                        h.indexer.setPower(0);
                        h.swingArm.setPosition(1);
                        cycleSubStep = 0;
                        currentIntakeState = IntakeState.IDLE;
                        break;
                    }

                    switch (cycleSubStep) {
                        case 0:
                            h.swingArm.setPosition(1);
                            h.sickle.setPosition(1);
                            h.gate.setPosition(0.65);
                            h.intake.setPower(0);
                            h.indexer.setPower(0);
                            intakeStateTimer.reset();
                            cycleSubStep = 1;
                            break;
                        case 1:
                            if (intakeStateTimer.milliseconds() > 500) {
                                h.sickle.setPosition(0.8);
                                h.swingArm.setPosition(0.6);
                                h.gate.setPosition(0.82);
                                intakeStateTimer.reset();
                                cycleSubStep = 2;
                            }
                            break;
                        case 2:
                            if (intakeStateTimer.milliseconds() > 250) {
                                h.intake.setPower(-1);
                                h.indexer.setPower(-1);
                                intakeStateTimer.reset();
                                cycleSubStep = 3;
                            }
                            break;
                        case 3:
                            if (intakeStateTimer.milliseconds() > 250) {
                                h.sickle.setPosition(0.75);
                                intakeStateTimer.reset();
                                cycleSubStep = 4;
                            }
                            break;
                        case 4:
                            if (intakeStateTimer.milliseconds() > 250) {
                                h.gate.setPosition(0.85);
                                h.swingArm.setPosition(0.2);
                                h.intake.setPower(0);
                                intakeStateTimer.reset();
                                cycleSubStep = 5;
                            }
                            break;
                        case 5:
                            if (intakeStateTimer.milliseconds() > 250) {
                                h.intake.setPower(0.5);
                                h.indexer.setPower(-1);
                                intakeStateTimer.reset();
                                cycleSubStep = 6;
                            }
                            break;
                        case 6:
                            if (intakeStateTimer.milliseconds() > 200) {
                                h.indexer.setPower(1);
                                intakeStateTimer.reset();
                                cycleSubStep = 7;
                            }
                            break;
                        case 7:
                            if (intakeStateTimer.milliseconds() > 100) {
                                h.gate.setPosition(0.65);
                                h.indexer.setPower(0);
                                h.swingArm.setPosition(0.6);
                                h.intake.setPower(-1);
                                intakeStateTimer.reset();
                                cycleSubStep = 8;
                            }
                            break;
                        case 8:
                            if (intakeStateTimer.milliseconds() > 100) {
                                h.intake.setPower(0);
                                cycleSubStep = 0;
                            }
                            break;
                    }
                    break;
            }

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
            distanceIn = h.revDist.getDistance(DistanceUnit.INCH);

            v.axial = -gamepad1.left_stick_y;
            v.lateral = gamepad1.left_stick_x;
            v.yawCmd = gamepad1.right_stick_x;

            fl = v.axial + v.lateral + v.yawCmd;
            fr = v.axial - v.lateral - v.yawCmd;
            bl = v.axial - v.lateral + v.yawCmd;
            br = v.axial + v.lateral - v.yawCmd;

            max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));


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

            if (gamepad1.dpad_up) {
                h.pip.setPosition(new Pose2D(DistanceUnit.INCH, -68.02, 36, AngleUnit.DEGREES, 180));
                samOffset = 0;
            } else if (gamepad1.dpad_down) {
                h.pip.resetPosAndIMU();
                samOffset = 0;
            }

            if (gamepad1.x) {
                var = 1;
            } else if (gamepad1.y) {
                var = 0;
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

            telemetry.addData("t", t);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("vTarget", vTarget);
            telemetry.addData("hpos", hpos);

            // READING FROM THREAD VARIABLES FOR TELEMETRY
            telemetry.addData("RPM flywheel 1", "%.3f", ((turret.currentFly1Vel * 60) / 37.333));
            telemetry.addData("RPM flywheel 2", "%.3f", ((turret.currentFly2Vel * 60) / 37.333));
            telemetry.addData("vel ticks flywheel 1", "%.3f", (turret.currentFly1Vel));
            telemetry.addData("vel ticks flywheel 2", "%.3f", (turret.currentFly2Vel));
            telemetry.addData("pip x in", h.pip.getPosX(DistanceUnit.INCH));
            telemetry.addData("heading", h.pip.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pip goalHeight in", h.pip.getPosY(DistanceUnit.INCH));
            telemetry.addData("turret1", turret.currentTurretCommand);
            telemetry.addData("turret2", turret.currentTurretCommand);

            telemetry.addData("PTO State", currentPtoDeploymentState.name());
            telemetry.addData("Intake State", currentIntakeState.name());
            if (currentIntakeState == IntakeState.INTAKE_TILL_PPG || currentIntakeState == IntakeState.INTAKE_TILL_PGP || currentIntakeState == IntakeState.INTAKE_TILL_GPP) {
                telemetry.addData("Cycle Sub-Step", cycleSubStep);
            }

            // Removed conflicting clearBulkCache() calls
            telemetry.update();
        }
        runThread = false;

        if (upperThread != null) {
            upperThread.interrupt();
        }
    }

    private void startTurretControlThread() {
        if (runThread) return;
        runThread = true;

        upperThread = new Thread(() -> {
            while (runThread && !isStopRequested()) {
                turret.loop();
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });

        upperThread.start();
    }
}