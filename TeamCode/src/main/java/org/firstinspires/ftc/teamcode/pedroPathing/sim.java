package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Decode Simulator", group = "A-Test")
public class sim extends OpMode {

    private Renderer field;
    private Renderer held;
    private ElapsedTime loopTimer;

    int fieldX = 73;
    int fieldY = 74;
    int transmissionInterval = 80;

    // Tuning constants
    double moveSpeed  = 0.035;
    double turnSpeed  = 0.003;
    double accel      = 0.001;
    double decel      = 0.002;
    double turnAccel  = 0.0005;
    double turnDecel  = 0.001;

    Robot redRobot  = new Robot(20,  20,  90,  "#ef5350");
    Robot blueRobot = new Robot(120, 20, 90, "#448aff");

    Artifact[] artifacts = {
            // Left side
            new Artifact(24 + 6, 24 + 12, 'g'),
            new Artifact(24,     24 + 12, 'p'),
            new Artifact(24 - 6, 24 + 12, 'p'),

            new Artifact(24 + 6, 48 + 12, 'p'),
            new Artifact(24,     48 + 12, 'g'),
            new Artifact(24 - 6, 48 + 12, 'p'),

            new Artifact(24 + 6, 72 + 12, 'p'),
            new Artifact(24,     72 + 12, 'p'),
            new Artifact(24 - 6, 72 + 12, 'g'),

            // Left corner
            new Artifact(4, 3,      'p'),
            new Artifact(4, 3 + 6,  'p'),
            new Artifact(4, 3 + 12, 'g'),

            // Right side
            new Artifact(144 - (24 + 6), 24 + 12, 'g'),
            new Artifact(144 - 24,       24 + 12, 'p'),
            new Artifact(144 - (24 - 6), 24 + 12, 'p'),

            new Artifact(144 - (24 + 6), 48 + 12, 'p'),
            new Artifact(144 - 24,       48 + 12, 'g'),
            new Artifact(144 - (24 - 6), 48 + 12, 'p'),

            new Artifact(144 - (24 + 6), 72 + 12, 'p'),
            new Artifact(144 - 24,       72 + 12, 'p'),
            new Artifact(144 - (24 - 6), 72 + 12, 'g'),

            // Right corner
            new Artifact(140, 3,      'p'),
            new Artifact(140, 3 + 6,  'p'),
            new Artifact(140, 3 + 12, 'g'),
    };

    int leftCount  = 0;
    int rightCount = 0;
    Artifact[] leftSlots  = new Artifact[9];
    Artifact[] rightSlots = new Artifact[9];

    double CLASSIFIER_RELEASE_RANGE = 8;

    private static String small(String text) {
        return "<small>" + text + "</small>";
    }

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(transmissionInterval);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);


        field = new Renderer(fieldX, fieldY);
        field.drawFieldLayout();
        field.snapshot();

        held = new Renderer(fieldX, 6);
        held.clear();

        loopTimer = new ElapsedTime();
        telemetry.addLine("Initializing...");
        telemetry.update();
    }

    @Override
    public void loop() {
        double loopTimeMs = loopTimer.milliseconds();
        loopTimer.reset();

        field.restore();

        // --- ROBOT UPDATES ---
        redRobot.update(loopTimeMs, moveSpeed, turnSpeed, accel, decel, turnAccel, turnDecel,
                gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        blueRobot.update(loopTimeMs, moveSpeed, turnSpeed, accel, decel, turnAccel, turnDecel,
                gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x);

        field.drawRobot(redRobot.x,  redRobot.y,  redRobot.heading,  "#ef5350");
        field.drawRobot(blueRobot.x, blueRobot.y, blueRobot.heading, "#448aff");

        // --- PICKUP ---
        redRobot.tryPickup(artifacts);
        blueRobot.tryPickup(artifacts);

        // --- DRAW UNCOLLECTED ARTIFACTS ---
        for (Artifact a : artifacts) {
            if (!a.collected) {
                field.drawArtifact(a.x, a.y, a.color());
            }
        }

        // --- ARTIFACT PHYSICS ---
        for (Artifact a : artifacts) {
            int result = a.update(loopTimeMs, leftSlots, rightSlots);
            if (result == 0) { leftCount++; blueRobot.score++; }
            else if (result == 1) { rightCount++; redRobot.score++; }
        }

        // --- SHOOT ---
        if (gamepad1.right_trigger > 0.5) redRobot.triggerShoot();
        if (gamepad2.right_trigger > 0.5) blueRobot.triggerShoot();
        redRobot.updateShooting(loopTimeMs);
        blueRobot.updateShooting(loopTimeMs);

        // --- CLASSIFIER RELEASE ---
        if (Math.hypot(redRobot.x - 2, redRobot.y - 72) < CLASSIFIER_RELEASE_RANGE ||
                Math.hypot(blueRobot.x - 2, blueRobot.y - 72) < CLASSIFIER_RELEASE_RANGE) {
            for (int i = 0; i < leftSlots.length; i++) {
                if (leftSlots[i] != null && !leftSlots[i].isRolling) {
                    leftSlots[i].rollTo(0);
                    leftSlots[i].classifierSlot = -1;
                    leftSlots[i] = null;
                }
            }
            leftCount = 0;
        }

        if (Math.hypot(redRobot.x - 142, redRobot.y - 72) < CLASSIFIER_RELEASE_RANGE ||
                Math.hypot(blueRobot.x - 142, blueRobot.y - 72) < CLASSIFIER_RELEASE_RANGE) {
            for (int i = 0; i < rightSlots.length; i++) {
                if (rightSlots[i] != null && !rightSlots[i].isRolling) {
                    rightSlots[i].rollTo(0);
                    rightSlots[i].classifierSlot = -1;
                    rightSlots[i] = null;
                }
            }
            rightCount = 0;
        }

        // --- HELD DISPLAY ---
        held.clear();
        int rx = 6;
        for (Artifact h : redRobot.held) {
            if (h != null) { held.drawArtifactPx(rx, 3, h.color()); rx += 8; }
        }
        int bx = fieldX - 6;
        for (int i = blueRobot.held.length - 1; i >= 0; i--) {
            if (blueRobot.held[i] != null) { held.drawArtifactPx(bx, 3, blueRobot.held[i].color()); bx -= 8; }
        }

        // --- TELEMETRY ---
        telemetry.addLine(
                "Field\n" +
                        small(field.renderHtml()) +
                        small("<font color='#ef5350'>RED(" + redRobot.score + ")</font>\t\t\t\t\t\t\t\t<font color='#448aff'>BLUE(" + blueRobot.score + ")</font>\n") +
                        small(held.renderHtml())
        );
        telemetry.update();
    }

    private void resizeField() {
        field.setSize(fieldX, fieldY);
        field.drawFieldLayout();
        field.snapshot();
    }

    // -------------------------------------------------------------------------

    static class Robot {
        double x, y, heading;
        double velX = 0, velY = 0, velTurn = 0;
        String color;
        Artifact[] held = new Artifact[3];
        boolean isShooting = false;
        int nextShootIndex = 0;
        double shootTimer = 0;
        int score = 0;

        static final double SHOOT_INTERVAL = 750.0 / 3.0;
        static final double PICKUP_RANGE   = 7;

        Robot(double x, double y, double heading, String color) {
            this.x = x; this.y = y; this.heading = heading; this.color = color;
        }

        void update(double dt, double moveSpeed, double turnSpeed,
                    double accel, double decel, double turnAccel, double turnDecel,
                    float inputX, float inputY, float inputTurn) {
            double ix = Math.abs(inputX)    < 0.05 ? 0 : -inputX;
            double iy = Math.abs(inputY)    < 0.05 ? 0 : -inputY;
            double it = Math.abs(inputTurn) < 0.05 ? 0 : -inputTurn;

            velX    = smooth(velX,    ix * moveSpeed, accel,     decel,     dt);
            velY    = smooth(velY,    iy * moveSpeed, accel,     decel,     dt);
            velTurn = smooth(velTurn, it * turnSpeed, turnAccel, turnDecel, dt);

            heading += velTurn * dt;
            double cosH = Math.cos(heading), sinH = Math.sin(heading);
            x += (velY * cosH - velX * sinH) * dt;
            y += (velY * sinH + velX * cosH) * dt;
            x = Math.max(0, Math.min(143, x));
            y = Math.max(0, Math.min(143, y));
        }

        void tryPickup(Artifact[] artifacts) {
            for (Artifact a : artifacts) {
                if (a.collected || a.inFlight || a.isClassified()) continue;
                if (Math.hypot(a.x - x, a.y - y) < PICKUP_RANGE) {
                    for (int i = 0; i < held.length; i++) {
                        if (held[i] == null) { held[i] = a; a.collected = true; break; }
                    }
                }
            }
        }

        void triggerShoot() {
            if (isShooting) return;
            for (Artifact h : held) {
                if (h != null) { isShooting = true; nextShootIndex = 0; shootTimer = 0; break; }
            }
        }

        void updateShooting(double dt) {
            if (!isShooting) return;
            shootTimer += dt;
            int expected = (int)(shootTimer / SHOOT_INTERVAL) + 1;
            while (nextShootIndex < held.length && nextShootIndex < expected) {
                Artifact h = held[nextShootIndex];
                if (h != null) {
                    h.collected  = false;
                    h.inFlight   = true;
                    h.velocityX  = Math.cos(heading) * 0.2;
                    h.velocityY  = Math.sin(heading) * 0.2;
                    h.x = x; h.y = y;
                    held[nextShootIndex] = null;
                }
                nextShootIndex++;
            }
            if (nextShootIndex >= held.length) isShooting = false;
        }

        private double smooth(double cur, double target, double accel, double decel, double dt) {
            double diff = target - cur;
            double rate = (Math.abs(target) >= Math.abs(cur) || Math.signum(target) != Math.signum(cur))
                    ? accel : decel;
            double step = rate * dt;
            if (Math.abs(diff) <= step) return target;
            return cur + Math.signum(diff) * step;
        }
    }

    // -------------------------------------------------------------------------

    static class Artifact {
        double x, y;
        char type;
        boolean collected, inFlight, isRolling;
        double velocityX = 0, velocityY = 0;
        int classifierSlot = -1;
        double targetY = 0;
        static final double ROLL_SPEED = 0.15;

        Artifact(double x, double y, char type) { this.x = x; this.y = y; this.type = type; }

        String color() {
            switch (type) {
                case 'g': return "#00ff88";
                case 'p': return "#cc44ff";
                default:  return "#ffffff";
            }
        }

        boolean isClassified() { return classifierSlot >= 0; }

        void rollTo(double destY) { targetY = destY; isRolling = true; }

        int update(double dt, Artifact[] leftSlots, Artifact[] rightSlots) {
            if (isRolling) {
                double step = ROLL_SPEED * dt;
                if (Math.abs(y - targetY) <= step) { y = targetY; isRolling = false; }
                else y += Math.signum(targetY - y) * step;
                return -1;
            }

            if (isClassified() || !inFlight) return -1;

            x += velocityX * dt;
            y += velocityY * dt;

            boolean inLeftGoal  = x >= 6   && x <= 24  && y >= 120;
            boolean inRightGoal = x >= 120  && x <= 138 && y >= 120;

            if (inLeftGoal || inRightGoal) {
                Artifact[] slots = inLeftGoal ? leftSlots : rightSlots;
                velocityX = 0; velocityY = 0; inFlight = false;
                x = inLeftGoal ? 1 : 143;
                y = 120;

                int count = 0;
                for (Artifact s : slots) if (s != null) count++;

                if (count >= slots.length) {
                    classifierSlot = -2;
                    rollTo(0);
                } else {
                    classifierSlot = count;
                    slots[count] = this;
                    rollTo(72 + (count * 6));
                }
                return inLeftGoal ? 0 : 1;
            }

            if (x < 0)    { x = 0;   velocityX = 0; velocityY = 0; inFlight = false; }
            if (x >= 144) { x = 143; velocityX = 0; velocityY = 0; inFlight = false; }
            if (y < 0)    { y = 0;   velocityX = 0; velocityY = 0; inFlight = false; }
            if (y >= 144) { y = 143; velocityX = 0; velocityY = 0; inFlight = false; }

            return -1;
        }
    }

    public static class Renderer {

        private int width;
        private int height;
        private boolean[][] pixels;
        private String[][] cellColors;
        private boolean[][] snapshotPixels;
        private String[][] snapshotColors;
        private double scaleX = width / 144.0;
        private double scaleY = height / 144.0;

        public Renderer(int width, int height) {
            this.width = width;
            this.height = height;
            this.scaleX = width / 144.0;
            this.scaleY = height / 144.0;
            this.pixels = new boolean[height][width];
            this.cellColors = new String[(height + 3) / 4][(width + 1) / 2];
        }

        public void setSize(int width, int height) {
            this.width = width;
            this.height = height;
            scaleX = width / 144.0;
            scaleY = height / 144.0;
            this.pixels = new boolean[height][width];
            this.cellColors = new String[(height + 3) / 4][(width + 1) / 2];
        }

        public void drawFieldLayout() {
            int w = width - 1;
            int h = height - 1;

            int midX = snapX(w / 2);
            int midY = h / 2;

            String grid  = "#666666";
            String field = "#bbbbbb";
            String red   = "#ff4444";
            String blue  = "#448aff";

            clear();

            drawRect(0, 0, w, h);

            int tiles = 6;
            for (int i = 1; i < tiles; i++) {
                int x = snapX((int)((i / (double)tiles) * w));
                int y = (int)((i / (double)tiles) * h);
                drawLine(x, 0, x, h, grid);
                drawLine(0, y, w, y, grid);
            }

            drawLine(0, h, midX, midY, field);
            drawLine(w, h, midX, midY, field);

            int zoneX = snapX(w/3);
            int zoneXR = snapX(w - w/3);
            drawLine(zoneX,  0, midX, h/6, field);
            drawLine(midX, h/6, zoneXR, 0, field);

            int goalInner  = snapX(w/24);
            int goalOuter  = snapX(w/6);
            int goalInnerR = w - goalInner;
            int goalOuterR = w - goalOuter;
            drawLine(goalInner,  h*5/6, goalOuter,  h, blue);
            drawLine(goalInner,  h,     goalOuter,  h, blue);
            drawLine(goalInnerR, h*5/6, goalOuterR, h, red);
            drawLine(goalInnerR, h,     goalOuterR, h, red);

            drawLine(goalInner,  h, goalInner,  midY, field);
            drawLine(goalInnerR, h, goalInnerR, midY, field);

            drawLine(goalInner,  midY, goalInner,  h/6, red);
            drawLine(goalInnerR, midY, goalInnerR, h/6, blue);

            int spikeX = snapX((int)((1.0/6.0) * w));
            int spikeXR = snapX((int)((5.0/6.0) * w));
            int spikeW = snapX((int)((15.0/24.0) * (w/6)));
            int spikeHalf = spikeW / 2;
            drawHorizontal(spikeX  - spikeHalf, spikeX  + spikeHalf, h/4, field);
            drawHorizontal(spikeX  - spikeHalf, spikeX  + spikeHalf, h*5/12, field);
            drawHorizontal(spikeX  - spikeHalf, spikeX  + spikeHalf, h*7/12, field);
            drawHorizontal(spikeXR - spikeHalf, spikeXR + spikeHalf, h/4, field);
            drawHorizontal(spikeXR - spikeHalf, spikeXR + spikeHalf, h*5/12, field);
            drawHorizontal(spikeXR - spikeHalf, spikeXR + spikeHalf, h*7/12, field);

            int hpW  = snapX(w/6);
            int hpWR = snapX((int)((5.0/6.0) * w));
            int hpH  = h/6;
            drawRect(0,     0, hpW,         hpH, field);
            drawRect(hpWR,  0, w - hpWR,    hpH, field);

            int grid2 = snapX((int)((2.0/6.0) * w));
            int grid4 = snapX((int)((4.0/6.0) * w));
            int ebW   = snapX((int)((3.0/24.0) * w));
            int ebH   = h/8;
            drawRect(grid2 - ebW,  h/6, ebW, ebH, red);
            drawRect(grid4,        h/6, ebW, ebH, blue);
        }

        public void drawRobot(double centerXInches, double centerYInches, double headingRadians, String color) {
            int px = toPxX(centerXInches);
            int py = toPxY(centerYInches);
            int r  = toPxX(18/2.0);
            drawCircle(px, py, r, color);
            int endX = (int) Math.round(px + Math.cos(headingRadians) * r);
            int endY = (int) Math.round(py + Math.sin(headingRadians) * r);
            drawLine(px, py, endX, endY, color);
        }

        public void drawArtifact(double xInches, double yInches, String color) {
            int px = toPxX(xInches);
            int py = toPxY(yInches);
            int r  = Math.max(1, toPxX(5/2.0));
            drawCircle(px, py, r, color);
        }

        public void drawArtifactPx(int px, int py, String color) {
            int r = Math.max(1, (int)(width * 2.5 / 144.0));
            drawCircle(px, py, r, color);
        }

        private void drawCircle(int cx, int cy, int r, String color) {
            int x = r;
            int y = 0;
            int err = 0;
            while (x >= y) {
                setPixel(cx + x, cy + y, color);
                setPixel(cx + y, cy + x, color);
                setPixel(cx - y, cy + x, color);
                setPixel(cx - x, cy + y, color);
                setPixel(cx - x, cy - y, color);
                setPixel(cx - y, cy - x, color);
                setPixel(cx + y, cy - x, color);
                setPixel(cx + x, cy - y, color);
                if (err <= 0) { y++; err += 2 * y + 1; }
                if (err > 0)  { x--; err -= 2 * x + 1; }
            }
        }

        public void clear() {
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    pixels[y][x] = false;
            for (int y = 0; y < cellColors.length; y++)
                for (int x = 0; x < cellColors[0].length; x++)
                    cellColors[y][x] = null;
        }

        private int toPxX(double inches) { return (int) Math.round(inches * scaleX); }
        private int toPxY(double inches) { return (int) Math.round(inches * scaleY); }

        public void snapshot() {
            snapshotPixels = new boolean[height][width];
            snapshotColors = new String[cellColors.length][cellColors[0].length];
            for (int y = 0; y < height; y++)
                System.arraycopy(pixels[y], 0, snapshotPixels[y], 0, width);
            for (int y = 0; y < cellColors.length; y++)
                System.arraycopy(cellColors[y], 0, snapshotColors[y], 0, cellColors[0].length);
        }

        public void restore() {
            for (int y = 0; y < height; y++)
                System.arraycopy(snapshotPixels[y], 0, pixels[y], 0, width);
            for (int y = 0; y < cellColors.length; y++)
                System.arraycopy(snapshotColors[y], 0, cellColors[y], 0, cellColors[0].length);
        }

        public void setPixel(int x, int y, String color) {
            int py = (height - 1) - y;
            if (x < 0 || py < 0 || x >= width || py >= height) return;
            pixels[py][x] = true;
            cellColors[py / 4][x / 2] = color;
        }

        public void setPixel(int x, int y) {
            int py = (height - 1) - y;
            if (x < 0 || py < 0 || x >= width || py >= height) return;
            pixels[py][x] = true;
        }

        public void drawHorizontal(int x1, int x2, int y) {
            for (int x = x1; x <= x2; x++) setPixel(x, y);
        }

        public void drawHorizontal(int x1, int x2, int y, String color) {
            for (int x = x1; x <= x2; x++) setPixel(x, y, color);
        }

        public void drawVertical(int y1, int y2, int x) {
            for (int y = y1; y <= y2; y++) setPixel(x, y);
        }

        public void drawVertical(int y1, int y2, int x, String color) {
            for (int y = y1; y <= y2; y++) setPixel(x, y, color);
        }

        public void drawRect(int x, int y, int w, int h) {
            drawHorizontal(x, x + w, y);
            drawHorizontal(x, x + w, y + h);
            drawVertical(y, y + h, x);
            drawVertical(y, y + h, x + w);
        }

        public void drawRect(int x, int y, int w, int h, String color) {
            drawHorizontal(x, x + w, y, color);
            drawHorizontal(x, x + w, y + h, color);
            drawVertical(y, y + h, x, color);
            drawVertical(y, y + h, x + w, color);
        }

        public void drawLine(int x0, int y0, int x1, int y1) {
            int dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
            int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
            int err = dx - dy;
            while (true) {
                setPixel(x0, y0);
                if (x0 == x1 && y0 == y1) break;
                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; x0 += sx; }
                if (e2 < dx)  { err += dx; y0 += sy; }
            }
        }

        public void drawLine(int x0, int y0, int x1, int y1, String color) {
            int dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
            int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
            int err = dx - dy;
            while (true) {
                setPixel(x0, y0, color);
                if (x0 == x1 && y0 == y1) break;
                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; x0 += sx; }
                if (e2 < dx)  { err += dx; y0 += sy; }
            }
        }

        private boolean isTrue(int x, int y) {
            if (x < 0 || y < 0 || x >= width || y >= height) return false;
            return pixels[y][x];
        }

        private char braille(int x, int y) {
            int code = 0;
            if (isTrue(x,     y))     code |= 1;
            if (isTrue(x,     y + 1)) code |= 2;
            if (isTrue(x,     y + 2)) code |= 4;
            if (isTrue(x + 1, y))     code |= 8;
            if (isTrue(x + 1, y + 1)) code |= 16;
            if (isTrue(x + 1, y + 2)) code |= 32;
            if (isTrue(x,     y + 3)) code |= 64;
            if (isTrue(x + 1, y + 3)) code |= 128;
            return (char) (0x2800 + code);
        }

        private int snapX(int x) { return (x / 2) * 2; }

        public String renderHtml() {
            int cellRows = (height + 3) / 4;
            int cellCols = (width + 1) / 2;
            StringBuilder out = new StringBuilder(cellRows * cellCols * 4);
            out.append("<pre style='line-height:1; letter-spacing:0;'>");
            for (int y = 0; y < height; y += 4) {
                int cy = y / 4;
                String runColor = null;
                StringBuilder run = new StringBuilder();
                for (int x = 0; x < width; x += 2) {
                    char b = braille(x, y);
                    String color = cellColors[cy][x / 2];
                    if (!strEq(color, runColor)) {
                        flushRun(out, run, runColor);
                        runColor = color;
                    }
                    run.append(b);
                }
                flushRun(out, run, runColor);
                out.append('\n');
            }
            out.append("</pre>");
            return out.toString();
        }

        private static void flushRun(StringBuilder out, StringBuilder run, String color) {
            if (run.length() == 0) return;
            if (color != null) {
                out.append("<font color='").append(color).append("'>").append(run).append("</font>");
            } else {
                out.append(run);
            }
            run.setLength(0);
        }

        private static boolean strEq(String a, String b) {
            if (a == b) return true;
            if (a == null || b == null) return false;
            return a.equals(b);
        }
    }
}