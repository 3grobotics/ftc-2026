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
    boolean bPressed = false;
    int ptoState = 0;

    /**
     * This OpMode illustrates how to use a video source (camera) as a color sensor
     *
     * A "color sensor" will typically determine the color of the object that it is pointed at.
     *
     * This sample performs the same function, except it uses a video camera to inspect an object or scene.
     * The user may choose to inspect all, or just a Region of Interest (ROI), of the active camera view.
     * The user must also provide a list of "acceptable colors" (Swatches)
     * from which the closest matching color will be selected.
     *
     * To perform this function, a VisionPortal runs a PredominantColorProcessor process.
     * The PredominantColorProcessor process is created first,
     * and then the VisionPortal is built to use this process.
     * The PredominantColorProcessor analyses the ROI and splits the colored pixels into several color-clusters.
     * The largest of these clusters is then considered to be the "Predominant Color"
     * The process then matches the Predominant Color with the closest Swatch and returns that match.
     * The process also returns the actual Predominant Color in three different color spaces: RGB, HSV & YCrCb
     * Each returned color-space value has three components, in the following ranges:
     *    RGB   Red 0-255, Green 0-255, Blue 0-255
     *    HSV   Hue 0-180, Saturation 0-255, Value 0-255
     *    YCrCb Luminance(Y) 0-255, Cr 0-255 (center 128), Cb 0-255 (center 128)
     *
     * To aid the user, a colored rectangle is drawn on the camera preview to show the RegionOfInterest,
     * The Predominant Color is used to paint the rectangle border,
     * so the user can verify that the color is reasonable.
     */
    @Override
    public void runOpMode() {
        
        /* these are the subsystems */ {
        h = new hardwareSubNewBot(hardwareMap);
        v = new varSub(); }
        
        /* these are the timers */ {
            swingTimer = new Timer();
        }

        /* this is the camera stuff */ { /*PredominantColorProcessor.Builder frontProcessorBuilder;
        PredominantColorProcessor.Builder middleProcessorBuilder;
        PredominantColorProcessor.Builder backProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        PredominantColorProcessor frontPredominantColorProcessor;
        PredominantColorProcessor middlePredominantColorProcessor;
        PredominantColorProcessor backPredominantColorProcessor;
        VisionPortal myVisionPortal;
        PredominantColorProcessor.Result frontResult;
        PredominantColorProcessor.Result middleResult;
        PredominantColorProcessor.Result backResult;





        // Build a "Color Sensor" vision processor based on the PredominantColorProcessor class.
        frontProcessorBuilder = new PredominantColorProcessor.Builder();
        middleProcessorBuilder = new PredominantColorProcessor.Builder();
        backProcessorBuilder = new PredominantColorProcessor.Builder();
        // - Focus the color sensor by defining a RegionOfInterest (ROI) which you want to inspect.
        //     This can be the entire frame, or a sub-region defined using:
        //     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
        //     Use one form of the ImageRegion class to define the ROI.
        // 100x100 pixel square near the upper left corner
        frontProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(60, 60, 200, 200));
        middleProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(300, 100, 400, 300));
        backProcessorBuilder.setRoi(ImageRegion.asImageCoordinates(700, 100, 800, 300));

        // - Set the list of "acceptable" color swatches (matches).
        //     Only colors that you assign here will be returned.
        //     If you know the sensor will be pointing to one of a few specific colors, enter them here.
        //     Or, if the sensor may be pointed randomly, provide some additional colors that may match the surrounding.
        //     Note that in the example shown below, only some of the available colors are included.
        //     This will force any other colored region into one of these colors.
        //     eg: Green may be reported as YELLOW, as this may be the "closest" match.
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
                //PredominantColorProcessor.Swatch.YELLOW,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.WHITE);
        middlePredominantColorProcessor = middleProcessorBuilder.build();

        backProcessorBuilder.setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.BLUE,
                //PredominantColorProcessor.Swatch.YELLOW,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.WHITE);
        backPredominantColorProcessor = backProcessorBuilder.build();

        // Build a vision portal to run the Color Sensor process.
        myVisionPortalBuilder = new VisionPortal.Builder();
        //  - Add the colorSensor process created above.
        myVisionPortalBuilder.addProcessor(frontPredominantColorProcessor);
        myVisionPortalBuilder.addProcessor(middlePredominantColorProcessor);
        myVisionPortalBuilder.addProcessor(backPredominantColorProcessor);
        //  - Set the desired video resolution.
        //      Since a high resolution will not improve this process, choose a lower resolution that is
        //      supported by your camera. This will improve overall performance and reduce latency.
        // Set the stream format.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.setCameraResolution(new Size(800, 448));
        //  - Choose your video source. This may be for a webcam or for a Phone Camera.


        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));






        myVisionPortal = myVisionPortalBuilder.build();*/ }

        /* telemetry stuff */ {
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE); }

        /* booleans that have to be here */ {
        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;

        boolean aa = gamepad2.a;
        boolean bb = gamepad2.b;
        boolean prevaa = aa;
        boolean prevbb = bb;}
        waitForStart();
        while (opModeIsActive() || opModeInInit()) {

            /* telemetry init message */
            telemetry.addLine("Preview on/off: 3 dots, Camera Stream");
            telemetry.addLine("");
            // Request the most recent color analysis.
            // This will return the closest matching colorSwatch and the predominant color in RGB, HSV and YCrCb color spaces.
            //frontResult = frontPredominantColorProcessor.getAnalysis();
            //middleResult = middlePredominantColorProcessor.getAnalysis();
            //backResult = backPredominantColorProcessor.getAnalysis();
            h.topDistSensor.setMode(DigitalChannel.Mode.INPUT);
            h.sickle.setPosition(1);
            /* setting pos*/ {
            if (gamepad1.dpad_right) {
                h.gate.setPosition(0.8);
            } else if (gamepad1.dpad_down) {
                h.gate.setPosition(0.65);
            }

            if (gamepad1.a) {
                h.swingArm.setPosition(0.5);
            } else if (gamepad1.b) {
                h.swingArm.setPosition(0.2);
            } else if (gamepad2.x) {
                h.swingArm.setPosition(0.9);
            }

            if (gamepad1.dpad_up) {
                h.gate.setPosition(1);
            } }

            /* intake */ {
                double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
            /*if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakeCmd = 1.0;
                h.sickle.setPosition(0.9);
            }
            else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intakeCmd = -1.0;

            }*/



            h.intake.setPower(-intakeCmd);
            h.indexer.setPower(-intakeCmd);

            if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                h.swingArm.setPosition(.5);
                h.gate.setPosition(.65);
            } }

            /* cycles once */ {
            if (gamepad1.dpad_left) {
                h.sickle.setPosition(0.9);
                h.swingArm.setPosition(0.5);
                h.gate.setPosition(0.9);
                sleep(250);
                h.intake.setPower(-1);
                h.indexer.setPower(-1);
                sleep(500);
                h.gate.setPosition(0.85);
                h.swingArm.setPosition(0.2);
                h.intake.setPower(0);
                h.indexer.setPower(0);
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
            } }

            /* fire */ {
            if (gamepad1.right_bumper) {
                h.gate.setPosition(1);
                h.indexer.setPower(-1);
                h.intake.setPower(-1);

                if (swingTimer.getElapsedTime() >= 50) {
                    v.swingHigh = !v.swingHigh;
                    swingTimer.resetTimer();
                }

                if (v.swingHigh) {
                    h.swingArm.setPosition(0.9);
                } else {
                    h.swingArm.setPosition(0.8);
                }

            } else {
                v.swingHigh = false;
                swingTimer.resetTimer();
            } }

            // intake till full
            // TODO add a timer to this V

            /* intake till full */{
            if (gamepad1.ps) {
                h.gate.setPosition(0.65);
                while ((h.topDistSensor.getState() && h.midDistSensor.getState() && h.frontDistSensor.getState()) == false) {
                    h.intake.setPower(-1);
                    h.indexer.setPower(-1);
                }
                h.indexer.setPower(0);
                h.intake.setPower(0);
            } }
            // todo ^^ add a timer to this ^^

            /*
            // cycles till ppg
            if (gamepad1.right_stick_button) {
                while (opModeIsActive() && !isStopRequested()) {
                    frontResult = frontPredominantColorProcessor.getAnalysis(); // refresh camera result

                    if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) ) {
                        break; // stop when green
                    }

                    h.sickle.setPosition(0.9);
                    h.swingArm.setPosition(0.5);
                    h.gate.setPosition(0.8);
                    sleep(250);

                    h.intake.setPower(-1);
                    h.indexer.setPower(-1);
                    sleep(500);

                    h.swingArm.setPosition(0.2);
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                    sleep(250);

                    h.intake.setPower(0.2);
                    h.indexer.setPower(-1);
                    sleep(100);

                    h.indexer.setPower(1);
                    sleep(100);

                    h.gate.setPosition(0.65);
                    h.indexer.setPower(0);
                    h.swingArm.setPosition(0.5);
                    h.intake.setPower(-1);
                    sleep(100);

                    h.intake.setPower(0);
                    sleep(100);

                    h.swingArm.setPosition(.9);
                    sleep(500);
                }

                // stop everything after loop exits
                h.intake.setPower(intakeCmd);
                h.indexer.setPower(intakeCmd);
                h.indexer.setPower(0);
            }


            // cycles till pgp
            if (gamepad1.left_stick_button) {
                while (opModeIsActive() && !isStopRequested()) {
                    middleResult = middlePredominantColorProcessor.getAnalysis(); // refresh camera

                    if (middleResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) ) {
                        break; // stop when green
                    }

                    h.sickle.setPosition(0.9);
                    h.swingArm.setPosition(0.5);
                    h.gate.setPosition(0.8);
                    sleep(250);

                    h.intake.setPower(-1);
                    h.indexer.setPower(-1);
                    sleep(500);

                    h.swingArm.setPosition(0.2);
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                    sleep(250);

                    h.intake.setPower(0.2);
                    h.indexer.setPower(-1);
                    sleep(100);

                    h.indexer.setPower(1);
                    sleep(100);

                    h.gate.setPosition(0.65);
                    h.indexer.setPower(0);
                    h.swingArm.setPosition(0.5);
                    h.intake.setPower(-1);
                    sleep(100);

                    h.intake.setPower(0);
                    sleep(100);

                    h.swingArm.setPosition(.9);
                    sleep(500);
                }

                // stop everything after loop exits
                h.intake.setPower(0);
                h.indexer.setPower(0);
            }

            // cycles till gpp
            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                while (opModeIsActive() && !isStopRequested()) {
                    backResult = middlePredominantColorProcessor.getAnalysis(); // refresh camera

                    if (backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) ) {
                        break; // stop when green
                    }

                    h.sickle.setPosition(0.9);
                    h.swingArm.setPosition(0.5);
                    h.gate.setPosition(0.8);
                    sleep(250);

                    h.intake.setPower(-1);
                    h.indexer.setPower(-1);
                    sleep(500);

                    h.swingArm.setPosition(0.2);
                    h.intake.setPower(0);
                    h.indexer.setPower(0);
                    sleep(250);

                    h.intake.setPower(0.2);
                    h.indexer.setPower(-1);
                    sleep(100);

                    h.indexer.setPower(1);
                    sleep(100);

                    h.gate.setPosition(0.65);
                    h.indexer.setPower(0);
                    h.swingArm.setPosition(0.5);
                    h.intake.setPower(-1);
                    sleep(100);

                    h.intake.setPower(0);
                    sleep(100);

                    h.swingArm.setPosition(.9);
                    sleep(500);
                }

                // stop everything after loop exits
                h.intake.setPower(0);
                h.indexer.setPower(0);
            }*/

            h.pip.update(); // leftdate position first!

            /*double robotX = h.pip.getPosX(DistanceUnit.INCH);
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
            double targetTurretRad = angleToGoal - robotHeading;

            // 1. The "Unwrap" Boundary (-180 to 180)
            // If the target crosses the back of the robot (>180 deg), it subtracts 360 deg.
            // This forces the turret to turn clockwise to catch it on the other side.
            while (targetTurretRad > Math.PI) targetTurretRad -= 2 * Math.PI;
            while (targetTurretRad < -Math.PI) targetTurretRad += 2 * Math.PI;

            // 2. Convert to Degrees and apply center offset (202 deg for a 404 range)
            double baseServoDegrees = Math.toDegrees(targetTurretRad) + 151.5;

            // 3. Combine Gamepad Logic + Vision + samOffset
            left = gamepad2.dpad_left;
            right = gamepad2.dpad_right;

            aa = gamepad2.a;
            bb = gamepad2.b;

            if (left && !prevleft && !right) {
                v.samOffset = Range.clip(v.samOffset + 2.5, -40, 40);
            }
            if (right && !prevright && !left) {
                v.samOffset = Range.clip(v.samOffset - 2.5, -40, 40);
            }

            // update prev AFTER using them
            prevleft = left;
            prevright = right;

            /*if (aa && !prevaa && !bb) {
                samOffset = Range.clip(samOffset + 2.5, -40, 40);
            }
            if (bb && !prevbb && !aa) {
                samOffset = Range.clip(samOffset - 2.5, -40, 40);
            }

            prevaa = aa;
            prevbb = bb;

            double finalServoDegrees = baseServoDegrees + v.visionOffsetDeg + v.samOffset;

            // 4. Clamp and Set
            finalServoDegrees = Range.clip(finalServoDegrees, 0, 303);

            if (gamepad1.ps || gamepad2.ps) {
                h.turret1.setPosition(0.5);
                h.turret2.setPosition(0.5);
            } else {
                h.turret1.setPosition(finalServoDegrees / 303);
                h.turret2.setPosition(finalServoDegrees / 303);
            }*/


            /*// =========================================================================
            //  HOOD LINEAR REGRESSION
            // =========================================================================
            double hpos;
            if (hypot < 130) {
                // Zone: Close
                double d1 = 57.5, d2 = 97.3;
                double v1 = 0.5, v2 = 0.7;
                double slope = (v2 - v1) / (d2 - d1);
                hpos = v1 + (slope * (hypot - d1));
            } else {
                // Zone: Far
                double d1 = 136.5, d2 = 158.1;
                double v1 = 0.7, v2 = 1.0;
                double slope = (v2 - v1) / (d2 - d1);
                hpos = v1 + (slope * (hypot - d1));
            }
            h.hood.setPosition(Range.clip(hpos, 0.5, 1.0));*/

            // =========================================================================
            //  FLYWHEEL LINEAR REGRESSION
            // =========================================================================
            /*double vTarget;
            if (hypot < 130) {
                // Zone: Close
                double d1 = 57.5, d2 = 97.3;
                double v1 = 1310, v2 = 1660;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = 1150 + (slope * (hypot - d1));
            } else {
                // Zone: Far
                double d1 = 136.5, d2 = 158.1;
                double v1 = 1900, v2 = 1940;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = farSlope + (slope * (hypot - d1));
            }
            vTarget = Range.clip(vTarget, 0, 2500); // Adjust max based on motor*/


            /* flywheel turn on/off */ {
            if (gamepad1.x) {
                v.var = 1;
            }
            else if (gamepad1.y) {
                v.var = 0;
            }

            if (v.var == 1) {
                h.flywheel1.setPower(1);
                h.flywheel2.setPower(1);
               //h.flywheel1.setVelocity(vTarget);
               //h.flywheel2.setVelocity(vTarget);
            } else {
                h.flywheel1.setVelocity(0);
                h.flywheel2.setVelocity(0);
            } }

            /*driving */{
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
            }

            /*if (gamepad1.right_bumper && gamepad1.left_bumper){
                h.ptoL.setPosition(1.0);
                h.ptoR.setPosition(1.0);
            } else {
                h.ptoL.setPosition(0.5);
                h.ptoR.setPosition(0.5);
            }*/

            /* PTO */ {
            if (gamepad2.a || gamepad1.options && !bPressed) {
                ptoState = (ptoState + 1) % 2;
                bPressed = true;
            } else if (!gamepad2.a || !gamepad1.options) {
                bPressed = false;
            }
         /*   if (gamepad1.right_bumper && gamepad1.left_bumper && !dPressed) {
                ptoState = (ptoState + 1) % 2;
                dPressed = true;
            } else if (!gamepad2.dpad_up) {
                dPressed = false;
            }*/
            switch (ptoState){
                case 0:
                    h.ptoR.setPosition(1.0);
                    h.ptoL.setPosition(1.0);
                    break;
                case 1:
                    h.ptoR.setPosition(0.5);
                    h.ptoL.setPosition(0.5);
                    break;
            } }

            //telemetry.addData("Best Match front", frontResult.closestSwatch);
            //telemetry.addData("Best Match middle", middleResult.closestSwatch);
            //telemetry.addData("Best Match back", backResult.closestSwatch);
            //telemetry.addLine("RGB   (" + JavaUtil.formatNumber(frontResult.RGB[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.RGB[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.RGB[2], 3, 0) + ")");
            //telemetry.addLine("HSV   (" + JavaUtil.formatNumber(frontResult.HSV[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.HSV[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.HSV[2], 3, 0) + ")");
            //telemetry.addLine("YCrCb (" + JavaUtil.formatNumber(frontResult.YCrCb[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.YCrCb[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.YCrCb[2], 3, 0) + ")");
            telemetry.update();
        }
    }
}
