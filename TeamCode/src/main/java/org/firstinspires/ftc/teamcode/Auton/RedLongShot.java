package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.AlignAprilTag;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.ThreeShots;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "RedLongShot", group = "Autonomous")
public class RedLongShot extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer, collectionTimer;
    private int pathState;
    private PathChain initialToScore, scoreToSpike, spikeCollectionEnd, collectionToScore;
    private RobotHardware hardware;
    private AlignAprilTag aligner;
    private ThreeShots threeShots;

    private static final int RED_TAG_ID = 24;
    private static final double COLLECTION_TIME_SEC = 4.0;
    private static final double COLLECTION_DRIVE_SPEED = 0.3;
    private boolean isCollecting = false;

    // Obelisk reading
    private int obeliskReading = -1;  // -1 = not read yet, 21/22/23 = spike mark
    private Pose targetSpike = null;
    private Pose targetSpikePost = null;
    private double collectionTargetX = 22.0;  // Will be set based on spike

    @Override
    public void init() {
        // Initialize hardware
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);

        // Initialize follower
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setPose(AutonConstants.redLongStart);
            telemetry.addData("Follower", "Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Follower failed: " + e.getMessage());
            return;
        }

        // Initialize alignment
        aligner = new AlignAprilTag(hardware, follower, telemetry);
        aligner.setTelemetryEnabled(true);

        // Initialize 3-shot
        threeShots = new ThreeShots(hardware);
        threeShots.setTelemetryEnabled(true);

        // Initialize timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        collectionTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", "RED");
        telemetry.addData("Start Pose", "X=%.1f Y=%.1f H=%.1f°",
                AutonConstants.redLongStart.getX(),
                AutonConstants.redLongStart.getY(),
                Math.toDegrees(AutonConstants.redLongStart.getHeading()));
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Continuously read obelisk during init
        readObelisk();

        telemetry.addData("=== OBELISK READING ===", "");
        if (obeliskReading == -1) {
            telemetry.addData("Status", "WAITING FOR TAG...");
            telemetry.addData("Looking for", "Tags 21, 22, or 23");
        } else {
            telemetry.addData("Status", "LOCKED IN");
            telemetry.addData("Tag Detected", obeliskReading);
            telemetry.addData("Spike Mark", getSpikeDescription(obeliskReading));
            telemetry.addData("Target Pose", "X=%.1f Y=%.1f",
                    targetSpike.getX(), targetSpike.getY());
        }

        telemetry.addData("", "");
        telemetry.addData("=== DETECTED TAGS ===", "");
        List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();
        if (detections.size() == 0) {
            telemetry.addData("Tags", "None detected");
        } else {
            for (AprilTagDetection tag : detections) {
                telemetry.addData("Tag " + tag.id, "Distance: %.1f\"",
                        tag.robotPose != null ? tag.robotPose.getPosition().z : -1);
            }
        }
        telemetry.update();
    }

    private void readObelisk() {
        List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();

        for (AprilTagDetection tag : detections) {
            // Look for obelisk tags (21, 22, 23)
            if (tag.id >= 21 && tag.id <= 23 && tag.robotPose != null) {
                obeliskReading = tag.id;

                // Set target spike based on reading
                switch (obeliskReading) {
                    case 21:  // G-P-P
                        targetSpike = AutonConstants.redSpike3;
                        targetSpikePost = AutonConstants.redSpike3Post;
                        break;
                    case 22:  // P-G-P
                        targetSpike = AutonConstants.redSpike2;
                        targetSpikePost = AutonConstants.redSpike2Post;
                        break;
                    case 23:  // P-P-G
                        targetSpike = AutonConstants.redSpike1;
                        targetSpikePost = AutonConstants.redSpike1Post;
                        break;
                }

                // Set collection target X based on spike post position
                collectionTargetX = targetSpikePost.getX();
                return;  // Keep most recent reading
            }
        }
    }

    private String getSpikeDescription(int tagId) {
        switch (tagId) {
            case 21: return "Spike 3 (G-P-P)";
            case 22: return "Spike 2 (P-G-P)";
            case 23: return "Spike 1 (P-P-G)";
            default: return "Unknown";
        }
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();

        // Default to middle spike if no reading
        if (obeliskReading == -1) {
            obeliskReading = 22;
            targetSpike = AutonConstants.redSpike2;
            targetSpikePost = AutonConstants.redSpike2Post;
            collectionTargetX = targetSpikePost.getX();
            telemetry.addData("WARNING", "No obelisk read - defaulting to Spike 2");
        }

        // Build paths now that we know the target
        buildPaths();
        setPathState(0);
    }

    private void buildPaths() {
        // Path from redLongStart to redLongScore
        initialToScore = follower.pathBuilder()
                .addPath(AutonConstants.redLongScorePath(AutonConstants.redLongStart))
                .build();

        telemetry.addData("Paths", "Built for Tag %d", obeliskReading);
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.update();

            if (threeShots.isBusy()) {
                threeShots.update(RED_TAG_ID);
            }

            updateCollection();
            autonomousPathUpdate();

            // Debug telemetry
            telemetry.addData("Alliance", "RED");
            telemetry.addData("Obelisk", "Tag %d - %s", obeliskReading, getSpikeDescription(obeliskReading));
            telemetry.addData("Path State", pathState);
            telemetry.addData("OpMode Time", "%.1fs", opmodeTimer.getElapsedTimeSeconds());
            telemetry.addData("X", "%.1f", follower.getPose().getX());
            telemetry.addData("Y", "%.1f", follower.getPose().getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Follower Busy", follower.isBusy());
            telemetry.addData("3-Shot Busy", threeShots.isBusy());
            telemetry.addData("Collecting", isCollecting);
            telemetry.addData("Shooter Power", "%.2f", hardware.shooter.getPower());
            telemetry.addData("Collector Power", "%.2f", hardware.collector.getPower());
            if (isCollecting) {
                telemetry.addData("Collection Time", "%.1fs / %.1fs",
                        collectionTimer.getElapsedTimeSeconds(), COLLECTION_TIME_SEC);
                telemetry.addData("Current X", "%.1f", follower.getPose().getX());
                telemetry.addData("Target X", "%.1f", collectionTargetX);
                telemetry.addData("Distance to Target", "%.1f",
                        collectionTargetX - follower.getPose().getX());
            }
            telemetry.update();
        }
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Drive to redLongScore
                telemetry.addData("Action", "Moving to score position");
                follower.followPath(initialToScore, true);
                setPathState(1);
                break;

            case 1:
                // Shoot preloaded samples
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "At score position, starting 3-shot");
                    threeShots.start(1600);
                    setPathState(2);
                }
                break;

            case 2:
                // Move to spike after shooting
                if (!threeShots.isBusy()) {
                    telemetry.addData("Action", "Moving to spike mark %d", obeliskReading);

                    // Turn on collector and shooter reverse
                    hardware.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hardware.collector.setPower(1.0);
                    hardware.shooter.setPower(-0.1);

                    // Build path to target spike with heading interpolation
                    Pose currentPose = follower.getPose();
                    com.pedropathing.paths.Path pathToSpike = new com.pedropathing.paths.Path(
                            new com.pedropathing.geometry.BezierLine(currentPose, targetSpike));
                    pathToSpike.setLinearHeadingInterpolation(currentPose.getHeading(), targetSpike.getHeading());

                    scoreToSpike = follower.pathBuilder()
                            .addPath(pathToSpike)
                            .build();
                    follower.followPath(scoreToSpike, false);
                    setPathState(3);
                }
                break;

            case 3:
                // Start collection drive at spike
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "At spike, starting collection drive");
                    startCollectionDrive();
                    setPathState(4);
                }
                break;

            case 4:
                // Drive slowly while collecting
                if (!isCollecting) {
                    telemetry.addData("Action", "Collection complete, returning to score");

                    hardware.shooter.setPower(0.0);

                    // Build path back to score
                    collectionToScore = follower.pathBuilder()
                            .addPath(AutonConstants.redLongScorePath(follower.getPose()))
                            .build();
                    follower.followPath(collectionToScore, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Shoot collected balls
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "At score position, starting final 3-shot");
                    threeShots.start(1600);
                    setPathState(6);
                }
                break;

            case 6:
                // Wait for final 3-shot to complete, then move to loading zone
                if (!threeShots.isBusy()) {
                    telemetry.addData("Action", "Moving to loading zone");

                    // Build path to redLongLoad
                    PathChain pathToLoad = follower.pathBuilder()
                            .addPath(AutonConstants.redLongLoadPath(follower.getPose()))
                            .build();
                    follower.followPath(pathToLoad, true);
                    setPathState(7);
                }
                break;

            case 7:
                // Wait to reach loading zone
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "Complete! At loading zone");
                    setPathState(-1);
                }
                break;
        }
    }

    private void startCollectionDrive() {
        isCollecting = true;
        collectionTimer.resetTimer();
        telemetry.addData("Collection Drive", "Started - driving to X=%.1f", collectionTargetX);
    }

    private void updateCollection() {
        if (!isCollecting) return;

        double currentX = follower.getPose().getX();
        double distanceToTarget = collectionTargetX - currentX;

        if (Math.abs(distanceToTarget) < 2.0 ||
                collectionTimer.getElapsedTimeSeconds() >= COLLECTION_TIME_SEC) {
            stopCollection();
            return;
        }

        // RED: driving to lower X values (backward)
        double drive = Math.signum(distanceToTarget) * COLLECTION_DRIVE_SPEED;

        hardware.lf.setPower(-drive);  // Inverted for red
        hardware.rf.setPower(-drive);
        hardware.lr.setPower(-drive);
        hardware.rr.setPower(-drive);

        telemetry.addData("Drive Direction", drive > 0 ? "FORWARD" : "BACKWARD");
        telemetry.addData("Drive Power", "%.2f", -drive);
    }

    private void stopCollection() {
        isCollecting = false;
        hardware.collector.setPower(0.0);
        hardware.lf.setPower(0);
        hardware.rf.setPower(0);
        hardware.lr.setPower(0);
        hardware.rr.setPower(0);
        telemetry.addData("Collection", "Stopped at X=%.1f", follower.getPose().getX());
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void stop() {
        if (follower != null) {
            Constants.autonFinalX = follower.getPose().getX();
            Constants.autonFinalY = follower.getPose().getY();
            Constants.autonFinalHeading = follower.getPose().getHeading();
        }

        if (hardware != null) {
            hardware.lf.setPower(0);
            hardware.rf.setPower(0);
            hardware.lr.setPower(0);
            hardware.rr.setPower(0);
            hardware.shooter.setPower(0);
            hardware.collector.setPower(0);
        }

        if (isCollecting) {
            stopCollection();
        }
    }
}
