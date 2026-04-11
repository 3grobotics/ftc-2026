/*package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "testing")
public class testing extends LinearOpMode {
    double origin = 0;     //It is launched from the origin (this(0,0)) – defined as our robot turret exit
    double shootPoint = 0;  //It passes through a point (this(x,goalHeight)) – which is the position of the goal
    //At that point, it is moving at a specific angle θ (relative to the horizontal)

    double airResistance = 0;


    double alphaDeg = ?;   // launch angle in degrees
    double v0 = ?;         // initial velocity
    double g = 9.81;          // gravity
    double t;           // time

    double alphaRad = Math.toRadians(alphaDeg);

    // velocity components
    double vx = v0 * Math.cos(alphaRad);
    double vy = v0 * Math.sin(alphaRad) - g * t;

    // Position components (integrate the velocity equations)
    double x = v0 * Math.cos(alphaRad) * t;
    double yt = v0 * Math.sin(alphaRad) * t - (0.5 * (g * t * g * t));

    double tanOfAlphaRad = Math.tan(alphaRad);
    double tanOfAlphaRad2 = Math.tan(alphaRad);


    @Override
    public void runOpMode() {
        // V trajectory angle V
        //tanOfAlphaRad = vy/vx;

        // V Plugging the velocity components in the above formula V
        tanOfAlphaRad = (v0 * Math.sin(alphaRad) - g * t) / v0 * Math.cos(alphaRad);


        t = x / v0 * Math.cos(alphaRad);

        tanOfAlphaRad = (v0 * Math.sin(alphaRad) - g * t) / v0 * Math.cos(alphaRad);

        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
*/