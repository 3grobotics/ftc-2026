package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class hardwareSubNewBot {
    public final DcMotorEx flywheel1, flywheel2, intake, frontRight, backRight, backLeft, frontLeft, indexer;
    public final Servo turret1, turret2, hood, swingArm, gate, sickle, ptoR, ptoL;
    public final DigitalChannel topDistSensor, midDistSensor, frontDistSensor;

    public final GoBildaPinpointDriver pip;


    public hardwareSubNewBot(HardwareMap hardwareMap) {
        pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        // gecko = hardwareMap.get(DcMotorEx.class, "gecko");
        turret1 = hardwareMap.get(Servo.class, "turret");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        ptoR = hardwareMap.get(Servo.class, "ptoR");
        ptoL = hardwareMap.get(Servo.class, "ptoL");
        hood = hardwareMap.get(Servo.class, "hood");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        swingArm = hardwareMap.get(Servo.class, "swingArm");
        topDistSensor = hardwareMap.get(DigitalChannel.class, "topDistSensor");
        sickle = hardwareMap.get(Servo.class, "sickle");
        gate = hardwareMap.get(Servo.class, "gate");
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        midDistSensor = hardwareMap.get(DigitalChannel.class, "midDistSensor");
        frontDistSensor = hardwareMap.get(DigitalChannel.class, "frontDistSensor");
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        //hood.setDirection(Servo.Direction.REVERSE);
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        turret1.setDirection(Servo.Direction.REVERSE);
        turret2.setDirection(Servo.Direction.REVERSE);
        indexer.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));

        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

}

