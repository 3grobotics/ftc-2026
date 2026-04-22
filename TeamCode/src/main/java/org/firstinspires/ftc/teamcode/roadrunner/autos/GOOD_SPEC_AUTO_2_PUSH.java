package org.firstinspires.ftc.teamcode.roadrunner.autos;
// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;
import org.firstinspires.ftc.teamcode.pedroPathing.ColorCamera2testing;
import org.firstinspires.ftc.teamcode.pedroPathing.ColorCamera2testingoldWithTread;

@Config
@Autonomous(name = "Daini no Bakudan RED(RSR far A)", group = "competition")
public class GOOD_SPEC_AUTO_2_PUSH extends LinearOpMode {

    public hardwareSubNewBot h;
    public varSub v;
    double robotX;
    double robotY;
    double xl;
    double yl;
    double hypot;

    double tx;
    double ty;
    double vx;
    double vy;
    double vTarget                   = 0;
    double t;
    double hoodTarget;
    double flywheelTarget;

    double hpos;
    double velocityreal;
    double velDiff;

    double angleToGoal;
    double robotHeading;

    double targetTurretRad;

    double baseServoDegrees;
    double target;
    double finalServoDegrees;
    public volatile double currentFly1Vel = 0;
    public volatile double currentFly2Vel = 0;
    public volatile double currentTurretCommand = 0;

    // Inner class for extending viper slide for samples
    public class turret {
        private ServoImplEx turret1;
        private ServoImplEx turret2;

        public turret(HardwareMap hardwareMap) {
            turret1 = hardwareMap.get(ServoImplEx.class, "turret");
            turret2 = hardwareMap.get(ServoImplEx.class, "turret2");
            turret1.setDirection(Servo.Direction.FORWARD);
            turret2.setDirection(Servo.Direction.FORWARD);
            turret1.setPwmRange(new PwmControl.PwmRange(550, 2450));
            turret2.setPwmRange(new PwmControl.PwmRange(550, 2450));
            h = new hardwareSubNewBot(hardwareMap);
            v = new varSub();
            
        }

        public class sigmaSkibidiClip implements Action {
            private final int targetPositionx;
            private final int targetPositiony;

            private boolean initialized = false;

            public sigmaSkibidiClip(int targetPositionx, int targetPositiony) {
                //  add math to make target pos in inches
                this.targetPositionx = targetPositionx;
                this.targetPositiony = targetPositiony;

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    vy                         = h.pip.getVelY(DistanceUnit.INCH);
                    vx                         = h.pip.getVelX(DistanceUnit.INCH);

                    tx                         = targetPositionx/*-72*/ - (vx * t);
                    ty                         = targetPositiony/* 72*/ - (vy * t);

                    robotX                     = h.pip.getPosX(DistanceUnit.INCH);
                    robotY                     = h.pip.getPosY(DistanceUnit.INCH);

                    xl                         = tx - robotX;
                    yl                         = ty - robotY;
                    hypot                      = Math.sqrt((xl * xl) + (yl * yl));

                    if ( hypot < 89){
                        t                      =  0.004 * hypot + 0.468;
                    } else if ( hypot > 89 && hypot < 121)  {
                        t                      =  0.833;
                    } else if ( hypot > 121) {
                        t                      = -0.004 * hypot + 1.317;
                    }

                    angleToGoal       = Math.atan2(yl, xl);
                    robotHeading      = h.pip.getHeading(AngleUnit.RADIANS);

                    targetTurretRad   = angleToGoal - robotHeading + Math.PI;

                    while (targetTurretRad > Math.PI) targetTurretRad -= 2 * Math.PI;
                    while (targetTurretRad < -Math.PI) targetTurretRad += 2 * Math.PI;

                    baseServoDegrees  = Math.toDegrees(targetTurretRad) - (314.6112145 / 2.0);
                    

                    // Calculate and save to volatile variable for main thread telemetry
                    finalServoDegrees = Math.abs((baseServoDegrees) / 314.6112145);
                    h.turret1.setPosition(finalServoDegrees);
                    h.turret2.setPosition(finalServoDegrees);

                    if (hypot < 75) {
                        vTarget = 9.938 * hypot + 1022.646;
                    } else if(hypot > 75 && hypot < 96.3){
                        vTarget = 12.066 * hypot + 863.05;
                    } else if(hypot > 96.3 && hypot < 129) {
                        vTarget = 9.817 * hypot + 1079.623;
                    } else if(hypot > 151) {
                        vTarget = 11.727 * hypot + 833.217;
                    }

                    vTarget = Range.clip(vTarget, 0, 6000);
                    
                    h.flywheel1.setVelocity(((vTarget) * 37.33) / 60);
                    h.flywheel2.setVelocity(((vTarget) * 37.33) / 60);
                   

                    //  HOOD LINEAR REGRESSION
                    if (hypot < 75) {
                        hpos = -0.006 * hypot + 1.053;
                    } else if(hypot > 75 && hypot < 96.3){
                        hpos = -0.005 * hypot + 0.975;
                    } else if(hypot > 96.3 && hypot < 129){
                        hpos = -0.015 * hypot + 1.944;
                    } else if(hypot > 151){
                        hpos = 0;
                    }
                    
                    hpos = Range.clip(hpos, 0, .7);
                    h.hood.setPosition(hpos);

                    initialized = true;
                }
                // Check MotorEx states

                return true; // Action still in progress
            }

        }

        public Action sigmaSkibidiClip(int targetPositionx, int targetPositiony) {
            return new sigmaSkibidiClip(targetPositionx, targetPositiony);
        }
    }
    
       
    // Inner class for intake wheel control
    public class intake {
        public Servo swingarm;
        public DcMotorEx intake;
        public DcMotorEx indexer;
        public Servo gate;


        public intake(HardwareMap hardwareMap) {
            swingarm = hardwareMap.get(Servo.class, "swing arm");
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            indexer = hardwareMap.get(DcMotorEx.class, "indexer");
            gate = hardwareMap.get(Servo.class, "gate");
        }

        // Action for pulling in
        public class intakeIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(1); // Positive power for intake
                indexer.setPower(-1);
                swingarm.setPosition(.65);
                gate.setPosition(.65);
                return false; // Action completes instantly
            }
        }

        public Action intakeIn() {
            return new intakeIn(); // Correctly return a new instance
        }

        // Action for pushing out
        public class intakeOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(-1); // Positive power for intake
                indexer.setPower(1);
                swingarm.setPosition(.65);
                gate.setPosition(.65);
                return false; // Action completes instantly
            }
        }

        public Action intakeOut() {
            return new intakeOut(); // Correctly return a new instance
        }

        public class fire implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gate.setPosition(.97);
                indexer.setPower(-1);
                intake.setPower(-1);
                swingarm.setPosition(.95);
                return false; // Action completes instantly
            }
        }

        public Action fire() {
            return new fire(); // Correctly return a new instance
        }

        // Action for stopping the intake wheel
        public class intakeOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower( 0); // Positive power for intake
                indexer.setPower(0);
                swingarm.setPosition(.65);
                gate.setPosition(.65);
                return false; // Action completes instantly
            }
        }

        public Action intakeOff() {
            return new intakeOff(); // Correctly return a new instance
        }
    }






    public static class TimerAction implements Action {
        private final long durationMs;
        private long startTime;

        public TimerAction(long durationMs) {
            this.durationMs = durationMs;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }
            return System.currentTimeMillis() - startTime < durationMs;
        }
    }
    public static class parmMove {
        public boolean Parmesan;


        public class ParmesanTrue implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Parmesan = true;
                return true;
            }
        }

        public Action ParmesanTrue() {
            return new ParmesanTrue();
        }

        public class ParmesanFalse implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Parmesan = false;
                return false;
            }
        }

        public Action ParmesanFalse() {return new ParmesanFalse();}
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d preloadScored = new Pose2d(-11,29,Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        turret turret = new turret(hardwareMap);
        Action preloadScore = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(48,0))
                .strafeTo(new Vector2d(0,0))
                .strafeTo(new Vector2d(48,0))
                .strafeTo(new Vector2d(0,0))
                .strafeTo(new Vector2d(48,0))
                .strafeTo(new Vector2d(0,0))
                .strafeTo(new Vector2d(48,0))
                .strafeTo(new Vector2d(0,0))
                .strafeTo(new Vector2d(48,0))
                .strafeTo(new Vector2d(0,0))
                .build();

        Action turretRun = new turret(hardwareMap).sigmaSkibidiClip(-72,72);
        Action shlorp = new intake(hardwareMap).intakeIn();
        Action bleh = new intake(hardwareMap).intakeOut();
        Action pew = new intake(hardwareMap).fire();

        waitForStart();

        if (isStopRequested()){
            return;
        }
        Actions.runBlocking(

                new SequentialAction(
                       new ParallelAction(
                              turretRun,
                               new SequentialAction(
                                       // this is where everything goes
                                       new ParallelAction(
                                       preloadScore,
                                       pew
                                       )
                               )
                       )

                )
        );

    }
}

