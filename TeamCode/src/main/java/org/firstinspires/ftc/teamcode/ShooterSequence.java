package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

/**
 * FINAL FIXED VERSION – 3 perfect shots every time
 * 1. Shooter spins FAST + RIGHT direction
 * 2. First shot = NO index
 * 3. Last two shots = index +750 ticks
 * 4. Full reset – no twitching
 */
public class ShooterSequence {

    private final RobotHardware hardware;
    private final ShootAprilTag aligner;
    private final Timer stateTimer = new Timer();
    private final Timer flashTimer = new Timer();

    public enum State { IDLE, START_SHOOTER, WAIT_FOR_SPEED, INDEX_BALL, WAIT_FOR_INDEX,
        FLIP_OUT, FLIP_IN, DONE, POST_SHOT_DELAY }
    private State state = State.IDLE;

    private double targetShooterVelocity = 0.0;
    private int targetCollectorPos = 0;
    private boolean indexingStarted = false;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    private int externalBurstRemaining = 0;

    // NEW: Track which shot we’re on
    private int shotNumber = 0;

    // TUNING
    private static final double SPEED_TOLERANCE = 80;
    private static final double INDEX_TOLERANCE = 30;
    private static final double FLIP_TIME = 0.25;
    private static final double MAX_COLLECTOR_CURRENT = 5.0;
    private static final double SPEED_TIMEOUT = 3.0;
    private static final double POST_SHOT_DELAY = 0.15;
    private static final double FLASH_DURATION = 0.08;

    // PID – P=5.0 is strong & fast
    private static final double P = 30.0, I = 0.0, D = 0.01, F = 0.0;

    public ShooterSequence(RobotHardware hardware, ShootAprilTag aligner) {
        this.hardware = hardware;
        this.aligner = aligner;

        // PID + FLOAT = spins every time
        PIDFCoefficients pid = new PIDFCoefficients(P, I, D, F);
        hardware.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        hardware.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setBurstRemaining(int count) {
        this.externalBurstRemaining = count;
    }

    /** Press B → 3-shot burst */
    public void start(double tps) {
        if (state != State.IDLE) return;

        state = State.START_SHOOTER;
        stateTimer.resetTimer();
        indexingStarted = false;
        shotNumber = 0;                     // ← RESET COUNTER
        targetShooterVelocity = 1600.0;     // ← 1600 ticks/sec
        hardware.shooter.setVelocity(targetShooterVelocity);
    }

    public void update(int tagId) {
        boolean tagVisible = isTagVisible(tagId);
        if (tagVisible) aligner.alignRotationOnly(tagId);
        else if (state == State.FLIP_OUT || state == State.FLIP_IN) {
            abortShot();
            return;
        }

        switch (state) {
            case START_SHOOTER:
                state = State.WAIT_FOR_SPEED;
                break;

            case WAIT_FOR_SPEED:
                hardware.telemetry.addData("Shooter Vel", "%.0f / 1600", hardware.shooter.getVelocity());

                if (stateTimer.getElapsedTimeSeconds() > SPEED_TIMEOUT) {
                    hardware.telemetry.addData("SHOOT", "TIMEOUT → FORCE");
                    goToNextState();
                    break;
                }
                if (Math.abs(hardware.shooter.getVelocity() - targetShooterVelocity) < SPEED_TOLERANCE) {
                    goToNextState();
                }
                break;

            case INDEX_BALL:
                if (!indexingStarted) {
                    targetCollectorPos = hardware.collector.getCurrentPosition() - 750; // give the collector a new target that is 750 ticks from the current position
                    hardware.collector.setTargetPosition(targetCollectorPos); // set the target position
                    hardware.collector.setMode(DcMotor.RunMode.RUN_TO_POSITION); // set to run to position
                    hardware.collector.setPower(0.9); // start moving
                    indexingStarted = true; // mark that indexing has started
                }
                state = State.WAIT_FOR_INDEX; // move to next state
                break;

            case WAIT_FOR_INDEX:
                double current = hardware.collector.getCurrent(CurrentUnit.AMPS); // monitor current draw
                if (current > MAX_COLLECTOR_CURRENT) { // if current exceeds max, abort
                    hardware.collector.setPower(0); // stop collector
                    abortShot(); // abort the shot sequence
                    hardware.telemetry.addData("INDEX", "ABORTED - HIGH CURRENT"); // log abort
                    break;
                }
                if (Math.abs(hardware.collector.getCurrentPosition() - targetCollectorPos) < INDEX_TOLERANCE) { // check if target position is reached
                    hardware.collector.setPower(0); // stop collector
                    state = State.FLIP_OUT; // move to flip out state
                    stateTimer.resetTimer(); // reset timer for flip out
                }
                break;

            case FLIP_OUT:
                hardware.flipper.setPosition(0.5);
                if (stateTimer.getElapsedTimeSeconds() > FLIP_TIME) {
                    state = State.FLIP_IN;
                    stateTimer.resetTimer();
                }
                break;

            case FLIP_IN:
                hardware.flipper.setPosition(0.0);
                if (stateTimer.getElapsedTimeSeconds() > FLIP_TIME) {
                    state = State.POST_SHOT_DELAY;
                    stateTimer.resetTimer();
                }
                break;

            case POST_SHOT_DELAY:
                if (stateTimer.getElapsedTimeSeconds() > POST_SHOT_DELAY) {
                    state = State.DONE;
                }
                break;

            case DONE:
                hardware.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                indexingStarted = false;
                shotNumber++;                                      // ← NEXT SHOT
                state = State.IDLE;

                if (externalBurstRemaining > 0) {
                } else {
                    hardware.shooter.setPower(0);
                    shotNumber = 0;                                // ← FULL RESET
                }
                break;
        }
    }

    /** Skip index on first shot */
    private void goToNextState() {
        if (shotNumber == 0) {
            state = State.FLIP_OUT;        // first ball already loaded
            stateTimer.resetTimer();
        } else {
            state = State.INDEX_BALL;
        }
    }

    private void abortShot() {
        hardware.shooter.setPower(0);
        hardware.flipper.setPosition(0.0);
        if (indexingStarted) {
            hardware.collector.setPower(0);
            hardware.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        state = State.DONE;
    }

    private boolean isTagVisible(int tagId) {
        List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();
        if (detections == null) return false;
        for (AprilTagDetection d : detections) if (d.id == tagId) return true;
        return false;
    }

    public boolean isBusy() { return state != State.IDLE; }
    public State getState() { return state; }
    public RevBlinkinLedDriver.BlinkinPattern getLEDPattern() { return currentPattern; }
}