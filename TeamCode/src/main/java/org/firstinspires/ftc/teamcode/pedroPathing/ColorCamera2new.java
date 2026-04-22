package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.photon.PhotonCore;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.turretSub;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;

@TeleOp(name = "ColorCamera 2 new")
public class ColorCamera2new extends LinearOpMode {

    boolean currentDpadLeft          = false;
    boolean currentPs                = false;
    boolean currentRightBumper       = false;
    boolean currentRightStickButton  = false;
    boolean currentLeftStickButton   = false;



    @Override
    public void runOpMode() {






        waitForStart();
        while (opModeIsActive()) {

            // Debounce gamepad1 buttons for intake control
             currentDpadLeft         = gamepad1.dpad_left;
             currentPs               = gamepad1.ps;
             currentRightBumper      = gamepad1.right_bumper;
             currentRightStickButton = gamepad1.right_stick_button;
             currentLeftStickButton  = gamepad1.left_stick_button;








        }
    }
}