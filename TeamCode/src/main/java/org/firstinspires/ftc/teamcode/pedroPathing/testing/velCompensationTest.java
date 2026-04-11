
package org.firstinspires.ftc.teamcode.pedroPathing.testing;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.hardwareSubNewBot;
import org.firstinspires.ftc.teamcode.Subsystems.varSub;

@TeleOp(name="velocity compensation test", group="Linear OpMode")
//@Disabled
public class velCompensationTest extends LinearOpMode {
        hardwareSubNewBot h;
        varSub v;
    double turretAngle = .5;
    double flywheelSpeed = 1;
    double hoodAngle = 0.7;



    @Override
    public void runOpMode() {
        h = new hardwareSubNewBot(hardwareMap);
        v = new varSub();

        h.pip.resetPosAndIMU();

        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;

        boolean aa = gamepad2.a;
        boolean bb = gamepad2.b;
        boolean prevaa = aa;
        boolean prevbb = bb;

        double samOffset = 0;
        waitForStart();

        while (opModeIsActive()) {

            vectorAndTurretUpdate(h.pip.getHeading(AngleUnit.RADIANS));


            while (turretAngle > Math.PI) turretAngle -= 2 * Math.PI;
            while (turretAngle < -Math.PI) turretAngle += 2 * Math.PI;

            // Convert to degrees and apply center offset
            double baseServoDegrees = (turretAngle) - (314.6112145 / 2.0);

            // Gamepad trim logic
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

            double finalServoDegrees = turretAngle + samOffset;

            //h.turret1.setPosition(Math.abs((finalServoDegrees) / 314.6112145));
            //h.turret2.setPosition(Math.abs((finalServoDegrees) / 314.6112145));

            telemetry.addData("turretAngle", turretAngle);
            telemetry.addData("(finalServoDegrees) / 314.6112145)", (Math.abs((finalServoDegrees) / 314.6112145)));
            telemetry.addData("hood angle", ((0.959931 - 0.558505) / (.7 - 0)) * hoodAngle);
            telemetry.addData("getFlywheelTicksFromVelocity(flywheelSpeed)", getFlywheelTicksFromVelocity(flywheelSpeed));
            telemetry.update();









            if (gamepad1.x) {
                v.var = 1;
            } else if (gamepad1.y) {
                v.var = 0;
            }

            if (v.var == 1) {
                h.flywheel1.setVelocity(getFlywheelTicksFromVelocity(flywheelSpeed));
                h.flywheel2.setVelocity(getFlywheelTicksFromVelocity(flywheelSpeed));
            } else {
                h.flywheel1.setVelocity(0);
                h.flywheel2.setVelocity(0);
            }





            //h.hood.setPosition(((.95 - .55) / (.7 - 0)) * hoodAngle);





            h.pip.update();
        }
    }

    private void vectorAndTurretUpdate(double robotHeading){
        
        double x;
        double xl;
        double yl;
        double hypot;
        double robotX;
        double robotY;

        // private Vector????
        robotX = h.pip.getPosX(DistanceUnit.INCH);
        robotY = h.pip.getPosY(DistanceUnit.INCH);

        xl = v.tx - robotX;
        yl = v.ty - robotY;

        hypot  = Math.sqrt((xl * xl) + (yl * yl));

        double angleToGoal = Math.atan2(yl, xl);


        x = hypot;

        hoodAngle = MathFunctions.clamp(Math.atan(((velocityVarSub.goalHeight * 2) / x ) - (Math.atan(velocityVarSub.theta))) , 0.558505,0.959931);



        double radicand1 = (velocityVarSub.g * x * x) / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - velocityVarSub.goalHeight));

        if (radicand1 > 0) {
            flywheelSpeed = Math.sqrt(radicand1);
        } else {
            flywheelSpeed = 0; // Or a reasonable default
        }

        double parallelComponent = h.pip.getVelY(DistanceUnit.INCH);
        double perpendicularComponent = h.pip.getVelX(DistanceUnit.INCH);

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double denominatorTime = flywheelSpeed * Math.cos(hoodAngle);
        double time = (denominatorTime != 0) ? x / denominatorTime : 0;
        double ivr = x / time * parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        hoodAngle = MathFunctions.clamp(Math.atan2(vz, nvr), 0.558505, 0.959931);

        double radicand2 = (velocityVarSub.g * ndr * ndr) / ((2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - velocityVarSub.goalHeight)));

        if (radicand2 > 0) {
            flywheelSpeed = Math.sqrt(radicand2);
        } else {
            // If the compensated version fails, keep the old speed or set to 0
            flywheelSpeed = 0;
        }
       double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
       turretAngle = robotHeading - angleToGoal + turretVelCompOffset;







    }

    public static double getFlywheelTicksFromVelocity(double velocity){
        return velocity * (79.64046242 * 12);
    }
    public static double getHoodTicksFromDegrees(double degrees){
        return 0.0304 * degrees - .7;
    }
}