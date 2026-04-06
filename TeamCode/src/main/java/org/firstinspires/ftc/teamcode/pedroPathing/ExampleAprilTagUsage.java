package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.List;

@TeleOp
public class ExampleAprilTagUsage extends LinearOpMode {
    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;
    double x = 0;
    double y = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- INIT ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        pinpoint.resetPosAndIMU();

        waitForStart();

        // --- START ---
        limelight.start();

        // --- LOOP ---
        while (opModeIsActive() && !isStopRequested()) {
            if((getXPoseFromlimelight() * 39.3700787) + (getYPoseFromlimelight() * 39.3700787) - (pinpoint.getPosX(DistanceUnit.INCH) + pinpoint.getPosY(DistanceUnit.INCH)) > 5){
                pinpoint.setPosX(getXPoseFromlimelight() * 39.3700787 ,DistanceUnit.INCH);
                pinpoint.setPosY(getYPoseFromlimelight() * 39.3700787 ,DistanceUnit.INCH);
            }
            pinpoint.update();
            telemetry.addData("Robot X", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Robot Y", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("MT2 Location y:", y * 39.3700787);
            telemetry.addData("MT2 Location x:", x * 39.3700787);
            telemetry.addData("difference", ((getXPoseFromlimelight() * 39.3700787) + (getYPoseFromlimelight() * 39.3700787)) - (pinpoint.getPosX(DistanceUnit.INCH) + pinpoint.getPosY(DistanceUnit.INCH)));

            telemetry.update();

        }
    }

    private Double getXPoseFromlimelight() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double robotYaw = pinpoint.getHeading(AngleUnit.DEGREES);
            limelight.updateRobotOrientation(robotYaw);

            if (result != null && result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();

                if (botpose_mt2 != null) {
                    x = botpose_mt2.getPosition().x;

                }
            }
        }
        return x;
    }

    private Double getYPoseFromlimelight() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double robotYaw = pinpoint.getHeading(AngleUnit.DEGREES);
            limelight.updateRobotOrientation(robotYaw);

            if (result != null && result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();

                if (botpose_mt2 != null) {
                    y = botpose_mt2.getPosition().y;

                }
            }
        }
        return y;
    }
}