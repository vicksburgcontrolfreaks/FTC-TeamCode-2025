package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LEDManager {
    private final RevBlinkinLedDriver leds;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean isRedAlliance;
    private boolean matchStarted = false;
    private boolean sequenceActive = false;
    private boolean magazineFull = false;
    private int flashCount = 0;
    private double flashTimer = 0;
    private FlashColor flashColor = FlashColor.ALLIANCE;

    private enum FlashColor {
        ALLIANCE,
        GREEN
    }

    public LEDManager(RevBlinkinLedDriver leds, boolean redAlliance) {
        this.leds = leds;
        this.isRedAlliance = redAlliance;
        setIdle();
    }

    public void startMatch() {
        matchStarted = true;
        setSolidAlliance();
    }

    public void startSequence() {
        sequenceActive = true;
        setRacing();
    }

    public void endSequence() {
        sequenceActive = false;
        flashGreen(3);
    }

    public void setMagazineFull(boolean full) {
        if (full && !magazineFull) flashAlliance(3);
        magazineFull = full;
    }

    public void update() {
        // Flash takes priority over everything
        if (flashCount > 0) {
            updateFlash();
            return;
        }

        if (!matchStarted) {
            pulseAlliance();
            return;
        }

        if (sequenceActive) {
            updateRacing();
            return;
        }

        if (timer.seconds() > 110) {  // 1:50
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            return;
        }

        setSolidAlliance();
    }

    public void setIdle() {
        pulseAlliance();
    }

    private void pulseAlliance() {
        double pulse = (Math.sin(timer.milliseconds() / 300) + 1) / 2;
        leds.setPattern(isRedAlliance ?
                (pulse > 0.5 ? RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.BLACK) :
                (pulse > 0.5 ? RevBlinkinLedDriver.BlinkinPattern.BLUE : RevBlinkinLedDriver.BlinkinPattern.BLACK));
    }

    private void setSolidAlliance() {
        leds.setPattern(isRedAlliance ? RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    private void setRacing() {
        double t = timer.milliseconds() / 100;
        int pos = (int) t % 8;
        leds.setPattern(pos < 4 ? RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    private void updateRacing() {
        setRacing();
    }

    public void flashGreen(int count) {
        flashCount = count * 2;  // on/off pairs
        flashTimer = timer.milliseconds();
        flashColor = FlashColor.GREEN;
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    private void flashAlliance(int count) {
        flashCount = count * 2;
        flashTimer = timer.milliseconds();
        flashColor = FlashColor.ALLIANCE;
        leds.setPattern(isRedAlliance ? RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    private void updateFlash() {
        if (timer.milliseconds() - flashTimer > 300) {
            flashCount--;
            flashTimer = timer.milliseconds();

            if (flashCount == 0) {
                setSolidAlliance();
            } else {
                // Toggle between color and black
                if (flashCount % 2 == 0) {
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                } else {
                    if (flashColor == FlashColor.GREEN) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    } else {
                        leds.setPattern(isRedAlliance ?
                                RevBlinkinLedDriver.BlinkinPattern.RED :
                                RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    }
                }
            }
        }
    }
}