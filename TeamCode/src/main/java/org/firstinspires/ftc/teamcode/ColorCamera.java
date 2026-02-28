package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name = "ColorCamera (Blocks to Java)")
public class ColorCamera extends LinearOpMode {

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
        PredominantColorProcessor.Builder frontProcessorBuilder;
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
        DigitalChannel topDistSensor;
        Servo sickle;
        Servo gate;
        Servo swingArm;
        DcMotor frontIntake;
        DcMotor indexer;
        DigitalChannel midDistSensor;
        DigitalChannel frontDistSensor;
        ColorSensor topColorSensor;
        ColorSensor midColorSensor;
        ColorSensor frontColorSensor;


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
        swingArm = hardwareMap.get(Servo.class, "swingArm");
        topDistSensor = hardwareMap.get(DigitalChannel.class, "topDistSensor");
        sickle = hardwareMap.get(Servo.class, "sickle");
        gate = hardwareMap.get(Servo.class, "gate");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        midDistSensor = hardwareMap.get(DigitalChannel.class, "midDistSensor");
        frontDistSensor = hardwareMap.get(DigitalChannel.class, "frontDistSensor");
        topColorSensor = hardwareMap.get(ColorSensor.class, "topColorSensor");
        midColorSensor = hardwareMap.get(ColorSensor.class, "midColorSensor");
        frontColorSensor = hardwareMap.get(ColorSensor.class, "frontColorSensor");



        myVisionPortal = myVisionPortalBuilder.build();
        // Speed up telemetry updates, Just use for debugging.
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit()) {
            telemetry.addLine("Preview on/off: 3 dots, Camera Stream");
            telemetry.addLine("");
            // Request the most recent color analysis.
            // This will return the closest matching colorSwatch and the predominant color in RGB, HSV and YCrCb color spaces.
            frontResult = frontPredominantColorProcessor.getAnalysis();
            middleResult = middlePredominantColorProcessor.getAnalysis();
            backResult = backPredominantColorProcessor.getAnalysis();
            topDistSensor.setMode(DigitalChannel.Mode.INPUT);
                sickle.setPosition(1);
                // Put loop blocks here.
                if (gamepad1.dpad_right) {
                    gate.setPosition(0.8);
                } else if (gamepad1.dpad_down) {
                    gate.setPosition(0.65);
                } else if (gamepad1.a) {
                    swingArm.setPosition(0.5);
                } else if (gamepad1.b) {
                    swingArm.setPosition(0.2);
                } else if (gamepad1.x) {
                    swingArm.setPosition(0.9);
                } else if (gamepad1.dpad_up) {
                    gate.setPosition(1);
                }
                if (gamepad1.right_bumper) {
                    frontIntake.setPower(1);
                } else if (gamepad1.left_bumper) {
                    frontIntake.setPower(-1);
                } else {
                    frontIntake.setPower(0);
                }
                indexer.setPower(gamepad1.right_trigger);
                indexer.setPower(-gamepad1.left_trigger);
                if (gamepad1.dpad_left) {
                    sickle.setPosition(0.9);
                    swingArm.setPosition(0.5);
                    gate.setPosition(0.8);
                    sleep(250);
                    frontIntake.setPower(-1);
                    indexer.setPower(-1);
                    sleep(500);
                    swingArm.setPosition(0.2);
                    frontIntake.setPower(0);
                    indexer.setPower(0);
                    sleep(250);
                    frontIntake.setPower(0.2);
                    indexer.setPower(-1);
                    sleep(100);
                    indexer.setPower(1);
                    sleep(100);
                    gate.setPosition(0.65);
                    indexer.setPower(0);
                    swingArm.setPosition(0.5);
                    frontIntake.setPower(-1);
                    sleep(100);
                    frontIntake.setPower(0);
                }
                if (gamepad1.y) {
                    while (gamepad1.y) {
                        gate.setPosition(1);
                        indexer.setPower(-1);
                        frontIntake.setPower(-1);
                        swingArm.setPosition(0.8);
                        sleep(50);
                        swingArm.setPosition(0.9);
                        sleep(100);
                    }
                    gate.setPosition(0.65);
                    swingArm.setPosition(0.5);
                }
                if (gamepad1.ps) {
                    gate.setPosition(0.65);
                    while ((topDistSensor.getState() && midDistSensor.getState() && frontDistSensor.getState()) == false) {
                        frontIntake.setPower(-1);
                        indexer.setPower(-1);
                    }
                    indexer.setPower(0);
                    frontIntake.setPower(0);
                }
            if (gamepad1.right_stick_button) {
                while (opModeIsActive() && !isStopRequested()) {
                    frontResult = frontPredominantColorProcessor.getAnalysis(); // refresh camera result

                    if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) ) {
                        break; // stop when green
                    }

                    sickle.setPosition(0.9);
                    swingArm.setPosition(0.5);
                    gate.setPosition(0.8);
                    sleep(250);

                    frontIntake.setPower(-1);
                    indexer.setPower(-1);
                    sleep(500);

                    swingArm.setPosition(0.2);
                    frontIntake.setPower(0);
                    indexer.setPower(0);
                    sleep(250);

                    frontIntake.setPower(0.2);
                    indexer.setPower(-1);
                    sleep(100);

                    indexer.setPower(1);
                    sleep(100);

                    gate.setPosition(0.65);
                    indexer.setPower(0);
                    swingArm.setPosition(0.5);
                    frontIntake.setPower(-1);
                    sleep(100);

                    frontIntake.setPower(0);
                    sleep(100);

                    swingArm.setPosition(.9);
                    sleep(500);
                }

                // stop everything after loop exits
                frontIntake.setPower(0);
                indexer.setPower(0);
            }

            if (gamepad1.left_stick_button) {
                while (opModeIsActive() && !isStopRequested()) {
                    middleResult = middlePredominantColorProcessor.getAnalysis(); // refresh camera

                    if (middleResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN) ) {
                        break; // stop when green
                    }

                    sickle.setPosition(0.9);
                    swingArm.setPosition(0.5);
                    gate.setPosition(0.8);
                    sleep(250);

                    frontIntake.setPower(-1);
                    indexer.setPower(-1);
                    sleep(500);

                    swingArm.setPosition(0.2);
                    frontIntake.setPower(0);
                    indexer.setPower(0);
                    sleep(250);

                    frontIntake.setPower(0.2);
                    indexer.setPower(-1);
                    sleep(100);

                    indexer.setPower(1);
                    sleep(100);

                    gate.setPosition(0.65);
                    indexer.setPower(0);
                    swingArm.setPosition(0.5);
                    frontIntake.setPower(-1);
                    sleep(100);

                    frontIntake.setPower(0);
                    sleep(100);

                    swingArm.setPosition(.9);
                    sleep(500);
                }

                // stop everything after loop exits
                frontIntake.setPower(0);
                indexer.setPower(0);
            }
            telemetry.addData("Best Match front", frontResult.closestSwatch);
            telemetry.addData("Best Match middle", middleResult.closestSwatch);
            telemetry.addData("Best Match back", backResult.closestSwatch);
            telemetry.addLine("RGB   (" + JavaUtil.formatNumber(frontResult.RGB[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.RGB[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.RGB[2], 3, 0) + ")");
            telemetry.addLine("HSV   (" + JavaUtil.formatNumber(frontResult.HSV[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.HSV[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.HSV[2], 3, 0) + ")");
            telemetry.addLine("YCrCb (" + JavaUtil.formatNumber(frontResult.YCrCb[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.YCrCb[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.YCrCb[2], 3, 0) + ")");
            telemetry.update();
        }
    }
}