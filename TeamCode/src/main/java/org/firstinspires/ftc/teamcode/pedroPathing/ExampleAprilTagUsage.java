package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp
public class ExampleAprilTagUsage extends LinearOpMode {
    private Limelight3A camera;
    private Follower follower;
    private boolean pathStarted = false;

    // The goBILDA RGB Indicator Light
    private ServoImplEx light;

    // Target location (X in inches, Y in inches, Heading in radians)
    private final Pose TARGET_LOCATION = new Pose(100, 100, Math.toRadians(36));

    // Self-Tuning Adaptive Filters
    private final SelfTuningKalmanFilter filterX = new SelfTuningKalmanFilter();
    private final SelfTuningKalmanFilter filterY = new SelfTuningKalmanFilter();
    private final SelfTuningKalmanFilter filterH = new SelfTuningKalmanFilter();

    // Update Thresholds
    private final double DISTANCE_TOLERANCE_INCHES = 2.5;
    private final double HEADING_TOLERANCE_RADIANS = Math.toRadians(5.0);

    @Override
    public void runOpMode() throws InterruptedException {
        camera = hardwareMap.get(Limelight3A.class, "limelight");

        // --- Light Initialization ---
        light = hardwareMap.get(ServoImplEx.class, "light");
        // Expand the PWM range to match the 500-2500us goBILDA specification perfectly
        light.setPwmRange(new PwmControl.PwmRange(500, 2500));
        // Turn it on! Change this value (between 0.0 and 1.0) to change the color/flash pattern
        light.setPosition(0.5);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(270)));

        waitForStart();

        camera.start();
        camera.pipelineSwitch(0);

        while (opModeIsActive()) {
            follower.update();

            // 1. Follow Path (Executes only once)
            if (!pathStarted) {
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
                                .setLinearHeadingInterpolation(
                                        follower.getPose().getHeading(),
                                        TARGET_LOCATION.getHeading()
                                )
                                .build()
                );
                pathStarted = true;
            }

            // 2. Fetch Filtered Camera Pose
            Pose camPose = getFilteredPoseFromCamera();

            // 3. Pose Overwrite Logic based on Tolerance
            if (camPose != null) {
                double distanceError = Math.hypot(
                        camPose.getX() - follower.getPose().getX(),
                        camPose.getY() - follower.getPose().getY()
                );

                double headingError = Math.abs(camPose.getHeading() - follower.getPose().getHeading());
                while (headingError > Math.PI) headingError -= 2 * Math.PI;
                headingError = Math.abs(headingError);

                if (distanceError > DISTANCE_TOLERANCE_INCHES || headingError > HEADING_TOLERANCE_RADIANS) {
                    follower.setPose(camPose);
                }
            }

            // 4. Telemetry
            telemetry.addData("Follower Busy", follower.isBusy());
            telemetry.addData("Odo X", follower.getPose().getX());
            telemetry.addData("Odo Y", follower.getPose().getY());
            telemetry.addData("Odo H (deg)", Math.toDegrees(follower.getPose().getHeading()));

            if (camPose != null) {
                telemetry.addData("Filtered Cam X", camPose.getX());
                telemetry.addData("Filtered Cam Y", camPose.getY());
            }
            telemetry.update();
        }
        camera.stop();
    }

    Pose getFilteredPoseFromCamera() {
        LLResult llResult = camera.getLatestResult();

        if (llResult == null || !llResult.isValid()) {
            return null;
        }

        Pose3D botpose = llResult.getBotpose();
        if (botpose == null || botpose.getPosition() == null) return null;

        Position thePosition = botpose.getPosition();

        double rawXInches = thePosition.x * 39.3700787;
        double rawYInches = thePosition.y * 39.3700787;

        double rawHeadingRad;
        try {
            rawHeadingRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        } catch (Exception e) {
            rawHeadingRad = follower.getPose().getHeading();
        }

        double filteredX = filterX.update(rawXInches);
        double filteredY = filterY.update(rawYInches);
        double filteredH = filterH.updateAngle(rawHeadingRad);

        return new Pose(filteredX, filteredY, filteredH, FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    // --- Self-Tuning Adaptive Kalman Filter ---
    public static class SelfTuningKalmanFilter {
        private final double Q = 0.1;
        private final double BASE_R = 0.5;
        private double x;
        private double p = 1.0;
        private boolean initialized = false;

        public double update(double measurement) {
            if (!initialized) {
                x = measurement;
                p = 1.0;
                initialized = true;
                return x;
            }

            double residual = Math.abs(measurement - x);
            double dynamicR = BASE_R + (residual * residual * 0.5);

            p = p + Q;
            double k = p / (p + dynamicR);
            x = x + k * (measurement - x);
            p = (1 - k) * p;

            return x;
        }

        public double updateAngle(double measurement) {
            if (!initialized) {
                x = measurement;
                p = 1.0;
                initialized = true;
                return x;
            }

            double diff = measurement - x;
            while (diff > Math.PI) diff -= 2 * Math.PI;
            while (diff < -Math.PI) diff += 2 * Math.PI;

            double residual = Math.abs(diff);
            double dynamicR = BASE_R + (residual * residual * 2.0);

            p = p + Q;
            double k = p / (p + dynamicR);

            x = x + k * diff;

            while (x > Math.PI) x -= 2 * Math.PI;
            while (x < -Math.PI) x += 2 * Math.PI;

            p = (1 - k) * p;
            return x;
        }
    }
}