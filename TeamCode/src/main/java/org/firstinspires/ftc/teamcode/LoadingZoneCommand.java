package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.AutonConstants;

/**
 * Re-usable command to navigate to hand load position and collect samples.
 * Traverses from current pose to the HandLoad pose, then drives forward to collect.
 * <p>
 * Usage in TeleOp:
 * <pre>
 * LoadingZoneCommand loadCmd = new LoadingZoneCommand(hardware, follower, telemetry, "BLUE");
 * loadCmd.start();  // Start the sequence
 * while (loadCmd.isBusy()) {
 *     loadCmd.update();  // Call every loop
 * }
 * </pre>
 */
public class LoadingZoneCommand {

    // --------------------------------------------------------------------- //
    // --------------------------  TUNED VALUES  --------------------------- //
    // --------------------------------------------------------------------- //
    private static final double COLLECTION_TIME_SEC = 4.0;
    private static final double COLLECTION_DRIVE_SPEED = 0.3;

    // --------------------------------------------------------------------- //
    // --------------------------  STATE  --------------------------------- //
    // --------------------------------------------------------------------- //
    private final RobotHardware hardware;
    private final Follower follower;
    private final Telemetry telemetry;
    private final String alliance;  // "BLUE" or "RED"

    private PathChain pathToLoadingZone;
    private Timer collectionTimer;
    private boolean isCollecting = false;
    private boolean isRunning = false;
    private int state = 0;  // 0 = not started, 1 = following path, 2 = collecting, 3 = done

    private Pose targetLoadingZone;
    private double collectionTargetX;

    // --------------------------------------------------------------------- //
    // --------------------------  CONSTRUCTOR  ---------------------------- //
    // --------------------------------------------------------------------- //
    public LoadingZoneCommand(RobotHardware hardware, Follower follower, Telemetry telemetry, String alliance) {
        this.hardware = hardware;
        this.follower = follower;
        this.telemetry = telemetry;
        this.alliance = alliance.toUpperCase();
        this.collectionTimer = new Timer();

        // Set alliance-specific targets for hand load position
        if (this.alliance.equals("BLUE")) {
            targetLoadingZone = AutonConstants.blueHandLoad;
            collectionTargetX = AutonConstants.blueHandLoadPost.getX();
        } else {
            targetLoadingZone = AutonConstants.redHandLoad;
            collectionTargetX = AutonConstants.redHandLoadPost.getX();
        }
    }

    // --------------------------------------------------------------------- //
    // --------------------------  PUBLIC API  ----------------------------- //
    // --------------------------------------------------------------------- //

    /** Start the loading zone sequence from current position */
    public void start() {
        if (isRunning) return;  // Already running

        isRunning = true;
        state = 1;

        // Build path from current position to loading zone
        Pose currentPose = follower.getPose();
        pathToLoadingZone = follower.pathBuilder()
                .addPath(new com.pedropathing.paths.Path(
                        new BezierLine(currentPose, targetLoadingZone)))
                .build();

        follower.followPath(pathToLoadingZone, false);

        telemetry.addData("Hand Load", "Started - Alliance: %s", alliance);
        telemetry.addData("Target", "X=%.1f Y=%.1f",
                targetLoadingZone.getX(), targetLoadingZone.getY());
        telemetry.update();
    }

    /** Call every loop in TeleOp */
    public void update() {
        if (!isRunning) return;

        switch (state) {
            case 1:
                // State 1: Following path to hand load position
                if (!follower.isBusy()) {
                    // Reached hand load position, start collection drive
                    telemetry.addData("Hand Load", "Reached - Starting collection");
                    startCollectionDrive();
                    state = 2;
                }
                break;

            case 2:
                // State 2: Slow collection drive
                updateCollection();
                if (!isCollecting) {
                    // Collection complete
                    telemetry.addData("Hand Load", "Complete");
                    state = 3;
                    isRunning = false;
                }
                break;

            case 3:
                // Done
                break;
        }

        // Telemetry
        telemetry.addData("HandLoad State", state);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Collecting", isCollecting);
        if (isCollecting) {
            telemetry.addData("Collection Time", "%.1fs / %.1fs",
                    collectionTimer.getElapsedTimeSeconds(), COLLECTION_TIME_SEC);
            telemetry.addData("Current X", "%.1f", follower.getPose().getX());
            telemetry.addData("Target X", "%.1f", collectionTargetX);
        }
    }

    /** True while the sequence is running */
    public boolean isBusy() {
        return isRunning;
    }

    /** Stop the sequence immediately */
    public void cancel() {
        if (isCollecting) {
            stopCollection();
        }
        isRunning = false;
        state = 0;
        telemetry.addData("Hand Load", "CANCELLED");
    }

    // --------------------------------------------------------------------- //
    // --------------------------  INTERNAL LOGIC  ------------------------- //
    // --------------------------------------------------------------------- //

    private void startCollectionDrive() {
        isCollecting = true;
        collectionTimer.resetTimer();

        // Turn on collector
        hardware.collector.setPower(1.0);
        hardware.shooter.setPower(-0.1);  // Reverse shooter slightly

        telemetry.addData("Collection Drive", "Started - driving to X=%.1f", collectionTargetX);
    }

    private void updateCollection() {
        if (!isCollecting) return;

        double currentX = follower.getPose().getX();
        // Blue (0°): forward = +X, Red (180°): forward = -X
        double distanceToTarget = alliance.equals("BLUE")
            ? collectionTargetX - currentX  // Blue: normal calculation (+X direction)
            : currentX - collectionTargetX;  // Red: flip calculation (-X direction)

        // Stop if reached target or timeout
        if (Math.abs(distanceToTarget) < 2.0 ||
                collectionTimer.getElapsedTimeSeconds() >= COLLECTION_TIME_SEC) {
            stopCollection();
            return;
        }

        // Calculate drive power
        double drive = Math.signum(distanceToTarget) * COLLECTION_DRIVE_SPEED;

        // Heading correction: maintain HandLoad heading (Blue: 0°, Red: 180°)
        double targetHeading = alliance.equals("BLUE") ? 0.0 : Math.toRadians(180);
        double currentHeading = follower.getPose().getHeading();
        double headingError = targetHeading - currentHeading;

        // Normalize heading error to [-PI, PI]
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        double turnCorrection = headingError * 0.3;  // P gain for heading

        // Apply drive + heading correction
        hardware.lf.setPower(drive + turnCorrection);
        hardware.rf.setPower(drive - turnCorrection);
        hardware.lr.setPower(drive + turnCorrection);
        hardware.rr.setPower(drive - turnCorrection);
    }

    private void stopCollection() {
        isCollecting = false;
        hardware.collector.setPower(0.0);
        hardware.shooter.setPower(0.0);
        hardware.lf.setPower(0);
        hardware.rf.setPower(0);
        hardware.lr.setPower(0);
        hardware.rr.setPower(0);
        telemetry.addData("Collection", "Stopped at X=%.1f", follower.getPose().getX());
    }
}
