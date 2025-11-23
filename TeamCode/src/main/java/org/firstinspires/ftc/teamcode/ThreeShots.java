package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    private static final int   SET_VELOCITY   = 1500;   // ticks/sec
    private static final int   INDEX_TICKS    = 1200;    // collector travel
    private static final double INDEX_POWER    = 1.0;
    private static final int   FLIP_TIME_MS   = 300;    // Reduced from 350ms
    private static final int   VELOCITY_TOLERANCE = 30; // Tighter tolerance for consistency
    private static final int   IDLE_VELOCITY  = 500;    // Keep spinning between bursts
    private static final int   RECOVERY_WAIT_MS = 100;  // Max time to wait for velocity recovery

    // --------------------------------------------------------------------- //
    // --------------------------  STATE  --------------------------------- //
    // --------------------------------------------------------------------- //
    private final RobotHardware hardware;
    private final AlignAprilTag aligner;

    private static final int TAG_ID = 20;  // or 24 for red

    // State machine for flipper
    private enum FlipState { IDLE, WAITING_VELOCITY, FLIPPING_UP, FLIPPING_DOWN, INDEXING, RECOVERING }
    private FlipState flipState = FlipState.IDLE;
    private long stateStartTime = 0;

    public int     shot       = 0;       // 0,1,2,3
    private int     currentShot = 0;
    private int     targetVelocity = 1500;  // Store the target velocity for this sequence

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
        // Flipper position will be set in OpMode start() method
    }

    // --------------------------------------------------------------------- //
    // --------------------------  PUBLIC API  ----------------------------- //
    // --------------------------------------------------------------------- //

    /** Start a new 3-shot burst at the given velocity. */
    public void start(int velocity) {
        // Only prevent start if currently running
        if (flipState != FlipState.IDLE) return;  // Already running

        // Reset state for new sequence
        shot = 0;
        currentShot = 0;
        targetVelocity = velocity;  // Store target velocity for this sequence
        flipState = FlipState.WAITING_VELOCITY;
        stateStartTime = System.currentTimeMillis();

        hardware.shooter.setVelocity(velocity);
        if (enableTelemetry) {
            hardware.telemetry.addData("3-SHOT", "START @ %d tps", velocity);
        }
    }

    /** Call every loop (auto or teleop). */
    public void update(int tagId) {
        // STOP AFTER SHOT 3
        if (shot >= 3) return;

        // Keep aligning during entire sequence
        if (flipState != FlipState.IDLE) {
            aligner.alignRotationOnly(tagId);
        }

        double vel = hardware.shooter.getVelocity();
        long elapsed = System.currentTimeMillis() - stateStartTime;

        // State machine for non-blocking operation
        switch (flipState) {
            case IDLE:
                // Nothing to do
                break;

            case WAITING_VELOCITY:
                // Wait for shooter to reach target velocity
                if (vel > targetVelocity - VELOCITY_TOLERANCE) {
                    // Ready to fire
                    preVelocities[shot] = vel;
                    flipState = FlipState.FLIPPING_UP;
                    stateStartTime = System.currentTimeMillis();
                    hardware.flipper.setPosition(1.0);

                    if (enableTelemetry) {
                        hardware.telemetry.addData("SHOT %d", shot + 1);
                        hardware.telemetry.addData("  Pre", "%.0f", vel);
                    }
                }
                break;

            case FLIPPING_UP:
                // Wait for flipper to go up
                if (elapsed >= FLIP_TIME_MS) {
                    flipState = FlipState.FLIPPING_DOWN;
                    stateStartTime = System.currentTimeMillis();
                    hardware.flipper.setPosition(0.0);
                }
                break;

            case FLIPPING_DOWN:
                // Wait for flipper to go down
                if (elapsed >= FLIP_TIME_MS) {
                    postVelocities[shot] = hardware.shooter.getVelocity();

                    if (enableTelemetry) {
                        hardware.telemetry.addData("  Post", "%.0f", postVelocities[shot]);
                    }

                    // For 3rd shot: turn on collector before transitioning
                    if (shot == 2) {
                        hardware.collector.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        hardware.collector.setPower(1.0);
                        shot = 3;  // Mark complete
                        currentShot = 3;
                        flipState = FlipState.IDLE;

                        // Keep shooter at idle speed instead of stopping
                        hardware.shooter.setVelocity(IDLE_VELOCITY);
                        hardware.flipper.setPosition(0.0);

                        if (enableTelemetry) {
                            hardware.telemetry.addData("3-SHOT", "COMPLETE (shooter idling @ %d)", IDLE_VELOCITY);
                            for (int i = 1; i <= 3; i++) {
                                hardware.telemetry.addData("Shot %d", i);
                                hardware.telemetry.addData("  Pre",  "%.0f", getPreVelocity(i));
                                hardware.telemetry.addData("  Post", "%.0f", getPostVelocity(i));
                            }
                        }
                    } else {
                        // For shots 1 and 2: index next sample
                        shot++;
                        currentShot = shot;
                        int target = hardware.collector.getCurrentPosition() + INDEX_TICKS;
                        hardware.collector.setTargetPosition(target);
                        hardware.collector.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        hardware.collector.setPower(INDEX_POWER);

                        flipState = FlipState.INDEXING;
                        stateStartTime = System.currentTimeMillis();

                        if (enableTelemetry) {
                            hardware.telemetry.addData("INDEX", "Start → %d (current: %d)",
                                    target, hardware.collector.getCurrentPosition());
                        }
                    }
                }
                break;

            case INDEXING:
                // Wait for collector to finish indexing
                if (!hardware.collector.isBusy()) {
                    flipState = FlipState.RECOVERING;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case RECOVERING:
                // Wait for shooter velocity to recover before next shot
                if (vel > targetVelocity - VELOCITY_TOLERANCE || elapsed > RECOVERY_WAIT_MS) {
                    flipState = FlipState.WAITING_VELOCITY;
                    stateStartTime = System.currentTimeMillis();
                }
                break;
        }

        // ---- live telemetry (optional) ----------------------------------
        if (enableTelemetry) {
            hardware.telemetry.addData("Shot", shot);
            hardware.telemetry.addData("State", flipState);
            hardware.telemetry.addData("Shooter", "%.0f / %d", vel, targetVelocity);
            hardware.telemetry.addData("Collector Pos", hardware.collector.getCurrentPosition());
            hardware.telemetry.addData("Collector Busy", hardware.collector.isBusy());
        }
    }

    /** True while the burst is still running. */
    public boolean isBusy() {
        return flipState != FlipState.IDLE;
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

    public void interrupt() {
        if (flipState == FlipState.IDLE) return;  // Already done

        // STOP EVERYTHING
        hardware.shooter.setVelocity(IDLE_VELOCITY);  // Idle instead of full stop
        hardware.flipper.setPosition(0.0);

        // CRITICAL: Reset collector mode before stopping
        hardware.collector.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.collector.setPower(0.0);

        // RESET STATE
        shot = 3;  // Mark as complete to prevent restart
        flipState = FlipState.IDLE;
        currentShot = 0;

        if (enableTelemetry) {
            hardware.telemetry.addData("3-SHOT", "INTERRUPTED (shooter idling)");
        }
    }
}