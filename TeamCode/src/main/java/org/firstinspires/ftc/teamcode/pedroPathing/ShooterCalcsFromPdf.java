package org.firstinspires.ftc.teamcode.pedroPathing;

public final class ShooterCalcsFromPdf {

    private ShooterCalcsFromPdf() { }

    // PDF uses 386.1 in/s^2 for gravity.
    public static final double GRAVITY_IN_PER_SEC2 = 386.1;

    public static final class InitialShotSolution {
        public final double launchAngleRad;
        public final double launchSpeed;

        public InitialShotSolution(double launchAngleRad, double launchSpeed) {
            this.launchAngleRad = launchAngleRad;
            this.launchSpeed = launchSpeed;
        }
    }

    public static final class CompensatedShotSolution {
        public final double baseLaunchAngleRad;
        public final double baseLaunchSpeed;
        public final double timeToGoalSec;

        public final double thetaDiffRad;
        public final double robotRadialVelocity;
        public final double robotTangentialVelocity;

        public final double vxCompensated;
        public final double vxNew;
        public final double vy;

        public final double compensatedLaunchAngleRad;
        public final double compensatedLaunchSpeed;

        public final double turretOffsetRad;

        public CompensatedShotSolution(
                double baseLaunchAngleRad,
                double baseLaunchSpeed,
                double timeToGoalSec,
                double thetaDiffRad,
                double robotRadialVelocity,
                double robotTangentialVelocity,
                double vxCompensated,
                double vxNew,
                double vy,
                double compensatedLaunchAngleRad,
                double compensatedLaunchSpeed,
                double turretOffsetRad
        ) {
            this.baseLaunchAngleRad = baseLaunchAngleRad;
            this.baseLaunchSpeed = baseLaunchSpeed;
            this.timeToGoalSec = timeToGoalSec;
            this.thetaDiffRad = thetaDiffRad;
            this.robotRadialVelocity = robotRadialVelocity;
            this.robotTangentialVelocity = robotTangentialVelocity;
            this.vxCompensated = vxCompensated;
            this.vxNew = vxNew;
            this.vy = vy;
            this.compensatedLaunchAngleRad = compensatedLaunchAngleRad;
            this.compensatedLaunchSpeed = compensatedLaunchSpeed;
            this.turretOffsetRad = turretOffsetRad;
        }
    }

    public static double solveLaunchAngleRad(double x, double y, double entryAngleRad) {
        return Math.atan((2.0 * y / x) - Math.tan(entryAngleRad));
    }

    public static double solveLaunchSpeed(double x, double y, double launchAngleRad) {
        return solveLaunchSpeed(x, y, launchAngleRad, GRAVITY_IN_PER_SEC2);
    }

    public static double solveLaunchSpeed(double x, double y, double launchAngleRad, double gravity) {
        double cosA = Math.cos(launchAngleRad);
        double tanA = Math.tan(launchAngleRad);

        double denominator = 2.0 * cosA * cosA * (x * tanA - y);
        return Math.sqrt((gravity * x * x) / denominator);
    }

    public static InitialShotSolution solveInitialShot(double x, double y, double entryAngleRad) {
        double alpha = solveLaunchAngleRad(x, y, entryAngleRad);
        double v0 = solveLaunchSpeed(x, y, alpha);
        return new InitialShotSolution(alpha, v0);
    }

    public static double solveTimeToGoal(double x, double launchSpeed, double launchAngleRad) {
        return x / (launchSpeed * Math.cos(launchAngleRad));
    }

    public static double solveRobotVelocityThetaDiff(double robotVelocityAngleRad, double lineToGoalAngleRad) {
        return robotVelocityAngleRad - lineToGoalAngleRad;
    }

    public static double solveRobotRadialVelocity(double robotVelocityMagnitude, double thetaDiffRad) {
        return -Math.cos(thetaDiffRad) * robotVelocityMagnitude;
    }

    public static double solveRobotTangentialVelocity(double robotVelocityMagnitude, double thetaDiffRad) {
        return Math.sin(thetaDiffRad) * robotVelocityMagnitude;
    }

    public static double solveVxCompensated(double distanceToGoal, double timeToGoalSec, double robotRadialVelocity) {
        return (distanceToGoal / timeToGoalSec) + robotRadialVelocity;
    }

    public static double solveVxNew(double vxCompensated, double robotTangentialVelocity) {
        return Math.sqrt(vxCompensated * vxCompensated + robotTangentialVelocity * robotTangentialVelocity);
    }

    public static double solveVy(double launchSpeed, double launchAngleRad) {
        return launchSpeed * Math.sin(launchAngleRad);
    }

    public static double solveCompensatedLaunchAngleRad(double vy, double vxNew) {
        // TODO: The PDF labels this as "Vlaunch,new = arctan(Vy / Vx,new)",
        // but that expression is an angle, not a velocity.
        // Using it as the new launch angle, which matches the surrounding text.
        return Math.atan(vy / vxNew);
    }

    public static double solveCompensatedLaunchSpeed(
            double originalTimeToGoalSec,
            double vxNew,
            double y,
            double compensatedLaunchAngleRad
    ) {
        double newX = vxNew * originalTimeToGoalSec;
        return solveLaunchSpeed(newX, y, compensatedLaunchAngleRad);
    }

    public static double solveTurretOffsetRad(double robotTangentialVelocity, double vxCompensated) {
        // TODO: The PDF prints "Turret Offset = tan(Vrt / Vx,compensated)".
        // Since the text says this is an angle, atan(...) is the version that makes sense here.
        return Math.atan(robotTangentialVelocity / vxCompensated);
    }

    public static double hoodAngleToServoPosition(
            double launchAngle,
            double a1,
            double s1,
            double a2,
            double s2
    ) {
        return ((s1 - s2) / (a1 - a2)) * (launchAngle - a1) + s1;
    }

    public static double launchSpeedToFlywheelSpeed(double launchSpeed, double fitSlope, double fitIntercept) {
        // TODO: The PDF says this comes from an Excel line fit, but it does not include the actual equation.
        // Put your measured line-fit coefficients here.
        return fitSlope * launchSpeed + fitIntercept;
    }

    public static CompensatedShotSolution solveCompensatedShot(
            double x,
            double y,
            double entryAngleRad,
            double robotVelocityMagnitude,
            double robotVelocityAngleRad,
            double lineToGoalAngleRad
    ) {
        InitialShotSolution initial = solveInitialShot(x, y, entryAngleRad);

        double time = solveTimeToGoal(x, initial.launchSpeed, initial.launchAngleRad);

        double thetaDiff = solveRobotVelocityThetaDiff(robotVelocityAngleRad, lineToGoalAngleRad);
        double vrr = solveRobotRadialVelocity(robotVelocityMagnitude, thetaDiff);
        double vrt = solveRobotTangentialVelocity(robotVelocityMagnitude, thetaDiff);

        double vxCompensated = solveVxCompensated(x, time, vrr);
        double vxNew = solveVxNew(vxCompensated, vrt);
        double vy = solveVy(initial.launchSpeed, initial.launchAngleRad);

        double compensatedAngle = solveCompensatedLaunchAngleRad(vy, vxNew);
        double compensatedSpeed = solveCompensatedLaunchSpeed(time, vxNew, y, compensatedAngle);
        double turretOffset = solveTurretOffsetRad(vrt, vxCompensated);

        return new CompensatedShotSolution(
                initial.launchAngleRad,
                initial.launchSpeed,
                time,
                thetaDiff,
                vrr,
                vrt,
                vxCompensated,
                vxNew,
                vy,
                compensatedAngle,
                compensatedSpeed,
                turretOffset
        );
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}