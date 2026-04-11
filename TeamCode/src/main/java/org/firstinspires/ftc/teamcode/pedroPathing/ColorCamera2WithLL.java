package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import java.util.ArrayList;

import java.util.List;

@TeleOp(name = "ColorCamera 2 ll")
public class ColorCamera2WithLL extends LinearOpMode {

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
    private Limelight3A limelight;
    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(4);
        limelight.start();

        /* these are the subsystems */
        {
            h = new hardwareSubNewBot(hardwareMap);
            v = new varSub();
        }

        /* these are the timers */
        {
            swingTimer = new Timer();
        }

        /* this is the camera stuff */
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


        myVisionPortal = myVisionPortalBuilder.build();

        /* telemetry stuff */
        {
            telemetry.setMsTransmissionInterval(50);
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        }

        int motifSum = 0;
        while (!isStarted() && !isStopRequested()) {


            LLResult result = limelight.getLatestResult();
            // Always check if the result is valid first
            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                List<Integer> seenIds = new ArrayList<>();

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    seenIds.add(fiducial.getFiducialId());
                }



                // Only do the math if the camera sees exactly 2 tags
                if (seenIds.size() == 2) {
                    // Grab the first and second IDs in your list and add them together
                    motifSum = seenIds.get(0) + seenIds.get(1);
                }

                // --- EVALUATE THE TRIANGLE MOTIFS ---

                // Example: If Tag 21 and Tag 22 are seen, the sum is 43
                if (motifSum == 43) {
                    telemetry.addLine("Motif 21-22 (gpp) detected!");

                    // Loop to find the specific tag you want to aim at for this motif
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == 21) {
                            double targetTx = fiducial.getTargetXDegrees();
                            // Execute targeting math here for Tag 21
                        }
                    }
                }
                // Example: If Tag 22 and Tag 23 are seen, the sum is 45
                else if (motifSum == 45) {
                    telemetry.addLine("Motif 22-23 (pgp) detected!");

                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == 22) {
                            double targetTx = fiducial.getTargetXDegrees();
                            // Execute targeting math here for Tag 22
                        }
                    }
                }
                // Example: If Tag 23 and Tag 21 are seen, the sum is 44
                else if (motifSum == 44) {
                    telemetry.addLine("Motif 23-21 (ppg) detected!");

                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == 23) {
                            double targetTx = fiducial.getTargetXDegrees();
                            // Execute targeting math here for Tag 23
                        }
                    }
                }

                // --- HANDLING A FLAT FACE (Seeing exactly 1 tag) ---
                else if (seenIds.size() == 1) {
                    int singleTag = seenIds.get(0);
                    telemetry.addData("Looking at flat face. Only seeing Tag", singleTag);

                    // You can pull the targeting data directly from the only item in the list
                    double targetTx = fiducials.get(0).getTargetXDegrees();
                    // Execute targeting math here for the single tag
                }

                // --- NOTHING RECOGNIZED ---
                else {
                    // motifSum is either 0 (saw 3 tags) or an unrecognized background combination
                    telemetry.addLine("No valid obelisk face found. Centering turret to 151.5...");
                    // Add your logic to move the turret back to 151.5 here
                }

                telemetry.update();
            }



        }
        boolean all = false;
        waitForStart();
        while (opModeIsActive()) {
            // Request the most recent color analysis.
            // This will return the closest matching colorSwatch and the predominant color in RGB, HSV and YCrCb color spaces.
            frontResult = frontPredominantColorProcessor.getAnalysis();
        //    middleResult = middlePredominantColorProcessor.getAnalysis();
            backResult = backPredominantColorProcessor.getAnalysis();


            if(h.topDistSensor.getState() == true && h.midDistSensor.getState() == true && h.frontDistSensor.getState() == true){
                all = true;
            } else {
                all = false;
            }


            // cycles till ppg
            if (motifSum == 44 && all == true) {
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


            // cycles till pgp
            if (motifSum == 45 && all == true) {
                while (opModeIsActive() && !isStopRequested()) {
                    backResult = backPredominantColorProcessor.getAnalysis(); // refresh camera

                    if (backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_GREEN)) {
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

                // stop everything after loop exits
                h.intake.setPower(0);
                h.indexer.setPower(0);
            }

            // gpp
            if(motifSum == 43 && all == true) {
                while (opModeIsActive() && !isStopRequested()) {
                    frontResult = frontPredominantColorProcessor.getAnalysis();
                    backResult = backPredominantColorProcessor.getAnalysis();

                    if (frontResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) && backResult.closestSwatch.equals(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE)) {
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
                h.intake.setPower(0);
                h.indexer.setPower(0);
            }


            telemetry.addData("Best Match front", frontResult.closestSwatch);
            //telemetry.addData("Best Match middle", middleResult.closestSwatch);
            telemetry.addData("Best Match back", backResult.closestSwatch);
            telemetry.addLine("RGB   (" + JavaUtil.formatNumber(frontResult.RGB[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.RGB[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.RGB[2], 3, 0) + ")");
            telemetry.addLine("HSV   (" + JavaUtil.formatNumber(frontResult.HSV[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.HSV[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.HSV[2], 3, 0) + ")");
            telemetry.addLine("YCrCb (" + JavaUtil.formatNumber(frontResult.YCrCb[0], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.YCrCb[1], 3, 0) + ", " + JavaUtil.formatNumber(frontResult.YCrCb[2], 3, 0) + ")");




            telemetry.update();
        }
    }

}
