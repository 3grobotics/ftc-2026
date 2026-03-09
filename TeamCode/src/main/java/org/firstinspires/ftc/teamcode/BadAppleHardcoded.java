package org.firstinspires.ftc.teamcode;

import android.util.Base64;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Bad Apple Hardcoded", group = "Meme")
public class BadAppleHardcoded extends OpMode {

    private Renderer display;
    private ElapsedTime frameTimer;

    final int VIDEO_WIDTH = 64;
    final int VIDEO_HEIGHT = 64;
    final double FRAME_TIME_MS = 100.0; // 10 FPS

    private int currentFrame = 0;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(80);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        display = new Renderer(VIDEO_WIDTH, VIDEO_HEIGHT);
        frameTimer = new ElapsedTime();

        telemetry.addLine("Loaded " + BadAppleData.FRAMES.length + " frames into memory.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (currentFrame >= BadAppleData.FRAMES.length) {
            currentFrame = 0; // Loop the video
        }

        if (frameTimer.milliseconds() >= FRAME_TIME_MS) {
            frameTimer.reset();
            display.clear();

            // Decode the current frame from Base64 back into raw bytes
            byte[] frameData = Base64.decode(BadAppleData.FRAMES[currentFrame], Base64.DEFAULT);
            int bitIndex = 0;

            for (int y = 0; y < VIDEO_HEIGHT; y++) {
                for (int x = 0; x < VIDEO_WIDTH; x++) {
                    int bytePos = bitIndex / 8;
                    int bitPos = 7 - (bitIndex % 8);
                    boolean isWhite = ((frameData[bytePos] >> bitPos) & 1) == 1;

                    if (isWhite) {
                        display.setPixel(x, (VIDEO_HEIGHT - 1 - y));
                    }
                    bitIndex++;
                }
            }

            telemetry.addLine(display.renderHtml());
            telemetry.update();

            currentFrame++;
        }
    }

    // --- STRIPPED DOWN RENDERER ---
    public static class Renderer {
        private int width;
        private int height;
        private boolean[][] pixels;

        public Renderer(int width, int height) {
            this.width = width;
            this.height = height;
            this.pixels = new boolean[height][width];
        }

        public void clear() {
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    pixels[y][x] = false;
        }

        public void setPixel(int x, int y) {
            int py = (height - 1) - y;
            if (x < 0 || py < 0 || x >= width || py >= height) return;
            pixels[py][x] = true;
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

        public String renderHtml() {
            int cellRows = (height + 3) / 4;
            int cellCols = (width + 1) / 2;
            StringBuilder out = new StringBuilder(cellRows * cellCols * 2);
            out.append("<pre style='line-height:1; letter-spacing:0;'>");
            for (int y = 0; y < height; y += 4) {
                for (int x = 0; x < width; x += 2) {
                    out.append(braille(x, y));
                }
                out.append('\n');
            }
            out.append("</pre>");
            return out.toString();
        }
    }
}