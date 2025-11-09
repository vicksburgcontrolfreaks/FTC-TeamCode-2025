package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Re-usable 3-shot burst.
 * <p>
 * Usage (auto or teleop):
 * <pre>
 * ThreeShots shots = new ThreeShots(hardware);
 * shots.setTelemetryEnabled(false);   // optional – silence telemetry
 * shots.start(1600);                 // fire three shots at 1600 tps
 * while (shots.isBusy()) {
 *     shots.update(tagId);           // call every loop
 * }
 * </pre>
 */
public class ThreeShots {

    // --------------------------------------------------------------------- //
    // --------------------------  TUNED VALUES  --------------------------- //
    // --------------------------------------------------------------------- //
    private static final int   SET_VELOCITY   = 1600;   // ticks/sec
    private static final int   INDEX_TICKS    = 800;    // collector travel
    private static final double INDEX_POWER    = 0.30;
    private static final int   FLIP_TIME_MS   = 250;
    private static final int   VELOCITY_TOLERANCE = 25; // fire when > SET-VEL-TOL

    // --------------------------------------------------------------------- //
    // --------------------------  STATE  --------------------------------- //
    // --------------------------------------------------------------------- //
    private final RobotHardware hardware;
    private final AlignAprilTag aligner;

    private static final int TAG_ID = 20;  // or 24 for red

    private boolean bPressed   = false;   // true while sequence is running
    public int     shot       = 0;       // 0,1,2
    private int     currentShot = 0;

    // store every shot so you can read it later
    private final double[] preVelocities  = new double[3];
    private final double[] postVelocities = new double[3];

    private boolean enableTelemetry = true;   // turn off with setTelemetryEnabled(false)

    // --------------------------------------------------------------------- //
    // --------------------------  CONSTRUCTOR  ---------------------------- //
    // --------------------------------------------------------------------- //
    public ThreeShots(RobotHardware hardware) {
        this.hardware = hardware;
        this.aligner  = new AlignAprilTag(hardware, null, hardware.telemetry);
        hardware.flipper.setPosition(0.0);
    }

    // --------------------------------------------------------------------- //
    // --------------------------  PUBLIC API  ----------------------------- //
    // --------------------------------------------------------------------- //

    /** Start a new 3-shot burst at the given velocity. */
    public void start(int velocity) {
        if (bPressed) return;
        bPressed = true;
        shot = 0;
        hardware.shooter.setVelocity(velocity);
        if (enableTelemetry) {
            hardware.telemetry.addData("3-SHOT", "START @ %d tps", velocity);
        }
    }

    /** Call every loop (auto or teleop). */
    public void update(int tagId) {
        // STOP AFTER SHOT 3
        if (shot >= 3) return;

        if (bPressed || shot > 0) {
            aligner.alignRotationOnly(tagId);
        }

        double vel = hardware.shooter.getVelocity();

        if (shot == 0 && vel > SET_VELOCITY - VELOCITY_TOLERANCE) {
            flipAndNext();
        } else if (shot > 0 && !hardware.collector.isBusy()) {
            flipAndNext();
        }


        // ---- live telemetry (optional) ----------------------------------
        if (enableTelemetry) {
            hardware.telemetry.addData("Shot", shot);
            hardware.telemetry.addData("Shooter", "%.0f", vel);
            hardware.telemetry.addData("Collector", hardware.collector.getCurrentPosition());
        }
    }

    /** True while the burst is still running. */
    public boolean isBusy() {
        return bPressed || shot > 0;

    }

    /** Turn telemetry on/off (default = on). */
    public void setTelemetryEnabled(boolean enabled) {
        this.enableTelemetry = enabled;
    }

    // --------------------------------------------------------------------- //
    // --------------------------  VELOCITY GETTERS  ----------------------- //
    // --------------------------------------------------------------------- //

    /** Pre-velocity of a specific shot (1-based). */
    public double getPreVelocity(int shotNum) {
        return (shotNum > 0 && shotNum <= 3) ? preVelocities[shotNum - 1] : 0;
    }

    /** Post-velocity of a specific shot (1-based). */
    public double getPostVelocity(int shotNum) {
        return (shotNum > 0 && shotNum <= 3) ? postVelocities[shotNum - 1] : 0;
    }

    /** Current shot number (0 = waiting, 1-3 = fired). */
    public int getCurrentShot() {
        return currentShot;
    }

    // --------------------------------------------------------------------- //
    // --------------------------  INTERNAL LOGIC  ------------------------- //
    // --------------------------------------------------------------------- //

    private void flipAndNext() {
        double pre = hardware.shooter.getVelocity();
        preVelocities[shot] = pre;

        if (enableTelemetry) {
            hardware.telemetry.addData("SHOT %d", shot + 1);
            hardware.telemetry.addData("  Pre", "%.0f", pre);
        }

        // AUTO-ALIGN
        aligner.alignRotationOnly(TAG_ID);

        // FLIP
        hardware.flipper.setPosition(0.5);
        sleep(FLIP_TIME_MS);
        hardware.flipper.setPosition(0.0);

        double post = hardware.shooter.getVelocity();
        postVelocities[shot] = post;

        if (enableTelemetry) {
            hardware.telemetry.addData("  Post", "%.0f", post);
        }

        // ——— ONLY INCREMENT IF NOT LAST SHOT ———
        if (shot < 2) {  // shot = 0 or 1 → becomes 1 or 2
            shot++;
            currentShot = shot;
            indexAndWait();  // Only for shots 1 and 2
        } else {
            // LAST SHOT (shot == 2)
            shot = 3;  // Mark complete
            currentShot = 3;
            hardware.shooter.setPower(0);
            hardware.flipper.setPosition(0.0);
            // RESET COLLECTOR MODE
            hardware.collector.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hardware.collector.setPower(0.0);
            if (enableTelemetry) {
                hardware.telemetry.addData("3-SHOT", "COMPLETE");
                for (int i = 1; i <= 3; i++) {
                    hardware.telemetry.addData("Shot %d", i);
                    hardware.telemetry.addData("  Pre",  "%.0f", getPreVelocity(i));
                    hardware.telemetry.addData("  Post", "%.0f", getPostVelocity(i));
                }
            }
        }
    }

    private void indexAndWait() {
        int target = hardware.collector.getCurrentPosition() + INDEX_TICKS;
        hardware.collector.setTargetPosition(target);
        hardware.collector.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hardware.collector.setPower(INDEX_POWER);

        if (enableTelemetry) {
            hardware.telemetry.addData("INDEX", "+%d ticks", INDEX_TICKS);
        }
        bPressed = false;               // wait for isBusy() in next update()
    }

    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (Exception ignored) {}
    }

    public void interrupt() {
        if (shot >= 3) return;  // Already done

        // STOP EVERYTHING
        hardware.shooter.setPower(0);
        hardware.flipper.setPosition(0.0);
        hardware.collector.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.collector.setPower(0.0);

        // RESET STATE
        shot = 0;
        bPressed = false;

        if (enableTelemetry) {
            hardware.telemetry.addData("3-SHOT", "INTERRUPTED");
        }
    }
}