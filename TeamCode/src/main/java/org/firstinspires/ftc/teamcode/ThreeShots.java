package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Re-usable 3-shot burst.
 * Shot 1: Uses flipper
 * Shots 2 & 3: Index until velocity drop detected (ball launches)
 */
public class ThreeShots {

    // --------------------------------------------------------------------- //
    // --------------------------  TUNED VALUES  --------------------------- //
    // --------------------------------------------------------------------- //
    private static final int   INDEX_TICKS = 550;
    private static final int   BACKOFF_TICKS = 50;
    private static final double INDEX_POWER = 0.4;
    private static final int   FLIP_TIME_MS = 350;
    private static final int   VELOCITY_TOLERANCE = 15;
    private static final int   FIRST_SHOT_MIN_WAIT_MS = 500;  // Minimum time to stabilize before first shot

    // --------------------------------------------------------------------- //
    // --------------------------  STATE  --------------------------------- //
    // --------------------------------------------------------------------- //
    private final RobotHardware hardware;
    private final AlignAprilTag aligner;

    private static final int TAG_ID = 20;  // or 24 for red

    // State machine
    private enum FlipState { IDLE, WAITING_VELOCITY, FLIPPING_UP, FLIPPING_DOWN, INDEXING, BACKING_OFF }
    private FlipState flipState = FlipState.IDLE;
    private long stateStartTime = 0;

    public int     shot       = 0;       // 0, 1, 2, 3 (3 = complete)
    private int     targetVelocity = 1500;
    private boolean needsIndexing = false;  // Track if we need to index before next shot

    private boolean enableTelemetry = true;

    // --------------------------------------------------------------------- //
    // --------------------------  CONSTRUCTOR  ---------------------------- //
    // --------------------------------------------------------------------- //
    public ThreeShots(RobotHardware hardware) {
        this.hardware = hardware;
        this.aligner  = new AlignAprilTag(hardware, null, hardware.telemetry);
    }

    // --------------------------------------------------------------------- //
    // --------------------------  PUBLIC API  ----------------------------- //
    // --------------------------------------------------------------------- //

    /** Start a new 3-shot burst at the given velocity. */
    public void start(int velocity) {
        if (flipState != FlipState.IDLE) return;  // Already running

        shot = 0;
        targetVelocity = velocity;
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
                boolean velocityReached = vel > targetVelocity - VELOCITY_TOLERANCE;
                boolean minTimeElapsed = (shot == 0) ? (elapsed >= FIRST_SHOT_MIN_WAIT_MS) : true;

                if (velocityReached && minTimeElapsed) {
                    // Check if we need to index first
                    if (needsIndexing) {
                        // Index before next shot - start indexing with servos
                        needsIndexing = false;

                        int target = hardware.collector.getCurrentPosition() + INDEX_TICKS;
                        hardware.collector.setTargetPosition(target);
                        hardware.collector.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        hardware.collector.setPower(INDEX_POWER);

                        // Start ball servos during indexing
                        hardware.startBallServos();

                        flipState = FlipState.INDEXING;
                        stateStartTime = System.currentTimeMillis();

                        if (enableTelemetry) {
                            hardware.telemetry.addData("INDEXING", "to shot %d", shot + 1);
                        }
                    } else {
                        // Ready to shoot: Start ball servos and flip
                        hardware.startBallServos();
                        hardware.flipper.setPosition(1.0);

                        flipState = FlipState.FLIPPING_UP;
                        stateStartTime = System.currentTimeMillis();

                        if (enableTelemetry) {
                            hardware.telemetry.addData("SHOT %d", shot + 1);
                        }
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
                    // Stop ball servos
                    hardware.stopBallServos();

                    shot++;  // Increment shot

                    if (shot >= 3) {
                        // Sequence complete after shot 3
                        flipState = FlipState.IDLE;

                        if (enableTelemetry) {
                            hardware.telemetry.addData("3-SHOT", "COMPLETE");
                        }
                    } else {
                        // For shots 1 and 2: need to index before next shot
                        needsIndexing = true;
                        flipState = FlipState.WAITING_VELOCITY;
                        stateStartTime = System.currentTimeMillis();

                        if (enableTelemetry) {
                            hardware.telemetry.addData("WAITING", "velocity for indexing");
                        }
                    }
                }
                break;

            case INDEXING:
                // Wait for indexing forward to complete
                if (!hardware.collector.isBusy()) {
                    // Start backing off
                    int target = hardware.collector.getCurrentPosition() - BACKOFF_TICKS;
                    hardware.collector.setTargetPosition(target);
                    hardware.collector.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    hardware.collector.setPower(INDEX_POWER);

                    flipState = FlipState.BACKING_OFF;
                    stateStartTime = System.currentTimeMillis();

                    if (enableTelemetry) {
                        hardware.telemetry.addData("BACKOFF", "%d ticks", BACKOFF_TICKS);
                    }
                }
                break;

            case BACKING_OFF:
                // Wait for backoff to complete
                if (!hardware.collector.isBusy()) {
                    hardware.collector.setPower(0.0);
                    hardware.stopBallServos();

                    // Go back to wait for velocity for next shot
                    flipState = FlipState.WAITING_VELOCITY;
                    stateStartTime = System.currentTimeMillis();

                    if (enableTelemetry) {
                        hardware.telemetry.addData("READY", "for shot %d", shot + 1);
                    }
                }
                break;
        }

        // ---- live telemetry (optional) ----------------------------------
        if (enableTelemetry) {
            hardware.telemetry.addData("Shot", shot);
            hardware.telemetry.addData("State", flipState);
            hardware.telemetry.addData("Shooter", "%.0f / %d", vel, targetVelocity);
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
    // --------------------------  INTERNAL LOGIC  ------------------------- //
    // --------------------------------------------------------------------- //

    public void interrupt() {
        if (flipState == FlipState.IDLE) return;

        // STOP EVERYTHING
        hardware.shooter.setVelocity(0);
        hardware.flipper.setPosition(0.0);
        hardware.stopBallServos();
        hardware.collector.setPower(0.0);

        // RESET STATE
        shot = 3;  // Mark as complete
        flipState = FlipState.IDLE;

        if (enableTelemetry) {
            hardware.telemetry.addData("3-SHOT", "INTERRUPTED");
        }
    }
}
