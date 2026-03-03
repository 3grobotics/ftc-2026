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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp
public class ExampleAprilTagUsage extends LinearOpMode {
    private Limelight3A camera; //any camera here
    private Follower follower;
    private boolean following = false;
    private final Pose TARGET_LOCATION = new Pose(72,72,45); //Put the target location here
    double xx = 0;
    double yy = 0;
    double hh = 0;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor frontLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        // init()
        camera = hardwareMap.get(Limelight3A.class, "limelight");
         frontRight = hardwareMap.get(DcMotor.class, "frontRight");
         backRight = hardwareMap.get(DcMotor.class, "backRight ");
         backLeft = hardwareMap.get(DcMotor.class, "backLeft ");
         frontLeft = hardwareMap.get(DcMotor.class, "frontLeft ");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(126,120,36));

        waitForStart();

        // start()
        camera.start();
        camera.pipelineSwitch(0);

        // loop()
        while (opModeIsActive()) {
            follower.update();
            //if you're not using limelight you can follow the same steps: build an offset pose, put your heading offset, and generate a path etc
            if (!following) {
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
                                .setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta())
                                .build()
                );
            }
            //This uses the aprilTag to relocalize your robot
            //You can also create a custom AprilTag fusion Localizer for the follower if you want to use this by default for all your autos
            follower.setPose(getRobotPoseFromCamera());
            if (following && !follower.isBusy()) following = false;
        }
    }

     Pose getRobotPoseFromCamera() {
        LLResult llResult = camera.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose();
            Position thePosition = botpose.getPosition();
            xx = thePosition.x * 39.3700787;
            yy = thePosition.y * 39.3700787;
            hh = thePosition.z;
        } else if(!llResult.isValid()){
            backLeft.setPower(-1);
            backRight.setPower(-1);
            frontLeft.setPower(-1);
            frontRight.setPower(-1);
        }

        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates

        return new Pose(xx, yy, hh, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
