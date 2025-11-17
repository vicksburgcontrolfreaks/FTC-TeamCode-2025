package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.AlignAprilTag;
import org.firstinspires.ftc.teamcode.LEDManager;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.ThreeShots;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "AutoLongShot", group = "Autonomous")
public class AutoLongShot extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer, collectionTimer;
    private int pathState;
    private PathChain initialToScore, scoreToSpike, spikeCollectionEnd, collectionToScore;
    private RobotHardware hardware;
    private AlignAprilTag aligner;
    private ThreeShots threeShots;
    private LEDManager ledManager;

    // Alliance detection
    private static final double DEFAULT_ALLIANCE_THRESHOLD = 200.0;
    public double allianceThreshold = DEFAULT_ALLIANCE_THRESHOLD;  // Adjustable threshold
    private String alliance = null;  // "BLUE" or "RED", determined by AprilTag position
    private Timer ledFlashTimer;
    private boolean ledFlashState = false;  // Toggle for flashing between red/blue

    private static final int BLUE_TAG_ID = 20;
    private static final int RED_TAG_ID = 24;
    private static final double COLLECTION_TIME_SEC = 4.0;
    private static final double COLLECTION_DRIVE_SPEED = 0.3;
    private boolean isCollecting = false;

    // Obelisk reading
    private int obeliskReading = -1;  // -1 = not read yet, 21/22/23 = spike mark
    private Pose targetSpike = null;
    private Pose targetSpikePost = null;
    private double collectionTargetX = 72.0;  // Will be set based on spike and alliance

    @Override
    public void init() {
        // Initialize hardware
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);

        // Initialize follower with a default pose (will be updated once alliance is detected)
        try {
            follower = Constants.createFollower(hardwareMap);
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
        ledFlashTimer = new Timer();
        opmodeTimer.resetTimer();
        ledFlashTimer.resetTimer();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance Threshold", "%.1f", allianceThreshold);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Detect alliance if not yet determined
        if (alliance == null) {
            detectAlliance();
            updateLEDsForDetection();
        } else {
            // Alliance detected - show solid color
            if (ledManager == null) {
                ledManager = new LEDManager(hardware.leds, alliance.equals("RED"));
                ledManager.setIdle();  // Will pulse the alliance color
            }
        }

        // Read obelisk continuously during init
        readObelisk();

        telemetry.addData("=== ALLIANCE DETECTION ===", "");
        if (alliance == null) {
            telemetry.addData("Status", "WAITING FOR TAG 20...");
            telemetry.addData("Threshold", "center.x > %.1f = RED", allianceThreshold);
            telemetry.addData("Threshold", "center.x <= %.1f = BLUE", allianceThreshold);
            telemetry.addData("Default", "RED if tag not visible");
        } else {
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Status", "LOCKED IN");
        }

        telemetry.addData("", "");
        telemetry.addData("=== OBELISK READING ===", "");
        if (obeliskReading == -1) {
            telemetry.addData("Status", "WAITING FOR TAG...");
            telemetry.addData("Looking for", "Tags 21, 22, or 23");
        } else {
            telemetry.addData("Status", "LOCKED IN");
            telemetry.addData("Tag Detected", obeliskReading);
            telemetry.addData("Spike Mark", getSpikeDescription(obeliskReading));
            if (targetSpike != null) {
                telemetry.addData("Target Pose", "X=%.1f Y=%.1f",
                        targetSpike.getX(), targetSpike.getY());
            }
        }

        telemetry.addData("", "");
        telemetry.addData("=== DETECTED TAGS ===", "");
        List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();
        if (detections.size() == 0) {
            telemetry.addData("Tags", "None detected");
        } else {
            boolean tag20Found = false;
            for (AprilTagDetection tag : detections) {
                if (tag.id == BLUE_TAG_ID) {
                    tag20Found = true;
                    String centerInfo = "";
                    if (tag.center != null) {
                        centerInfo = String.format(" | center.x=%.1f", tag.center.x);
                    }
                    telemetry.addData("Tag 20 (BLUE)", "FOUND%s", centerInfo);
                }
            }
            if (!tag20Found) {
                telemetry.addData("Tag 20 (BLUE)", "NOT VISIBLE");
            }

            // Show other detected tags
            for (AprilTagDetection tag : detections) {
                if (tag.id != BLUE_TAG_ID) {
                    String centerInfo = "";
                    if (tag.center != null) {
                        centerInfo = String.format(" | center.x=%.1f", tag.center.x);
                    }
                    telemetry.addData("Tag " + tag.id, "Distance: %.1f\"%s",
                            tag.robotPose != null ? tag.robotPose.getPosition().z : -1,
                            centerInfo);
                }
            }
        }
        telemetry.update();
    }

    private void detectAlliance() {
        List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();

        for (AprilTagDetection tag : detections) {
            // Only use tag 20 (BLUE) for alliance detection
            if (tag.id == BLUE_TAG_ID && tag.center != null) {
                if (tag.center.x > allianceThreshold) {
                    alliance = "RED";
                    follower.setPose(AutonConstants.redLongStart);
                    telemetry.addData("Alliance Detected", "RED (Tag 20 center.x=%.1f > %.1f)",
                            tag.center.x, allianceThreshold);
                } else {
                    alliance = "BLUE";
                    follower.setPose(AutonConstants.blueLongStart);
                    telemetry.addData("Alliance Detected", "BLUE (Tag 20 center.x=%.1f <= %.1f)",
                            tag.center.x, allianceThreshold);
                }
                return;  // Keep the first detection
            }
        }
    }

    private void updateLEDsForDetection() {
        // Flash between red and blue every 300ms when alliance not detected
        if (ledFlashTimer.getElapsedTimeSeconds() >= 0.3) {
            ledFlashState = !ledFlashState;
            if (ledFlashState) {
                hardware.leds.setPattern(com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RED);
            } else {
                hardware.leds.setPattern(com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
            ledFlashTimer.resetTimer();
        }
    }

    private void readObelisk() {
        if (alliance == null) return;  // Can't read obelisk without knowing alliance

        List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();

        for (AprilTagDetection tag : detections) {
            // Look for obelisk tags (21, 22, 23)
            if (tag.id >= 21 && tag.id <= 23 && tag.robotPose != null) {
                obeliskReading = tag.id;

                // Set target spike based on reading and alliance
                if (alliance.equals("BLUE")) {
                    switch (obeliskReading) {
                        case 21:  // G-P-P
                            targetSpike = AutonConstants.blueSpike3;
                            targetSpikePost = AutonConstants.blueSpike3Post;
                            break;
                        case 22:  // P-G-P
                            targetSpike = AutonConstants.blueSpike2;
                            targetSpikePost = AutonConstants.blueSpike2Post;
                            break;
                        case 23:  // P-P-G
                            targetSpike = AutonConstants.blueSpike1;
                            targetSpikePost = AutonConstants.blueSpike1Post;
                            break;
                    }
                } else {  // RED
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
                }

                // Set collection target X based on spike post position
                if (targetSpikePost != null) {
                    collectionTargetX = targetSpikePost.getX();
                }
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

        // Default to RED if no alliance detected (Tag 20 not visible)
        if (alliance == null) {
            alliance = "RED";
            follower.setPose(AutonConstants.redLongStart);
            telemetry.addData("WARNING", "No alliance detected (Tag 20 not visible) - defaulting to RED");
        }

        // Initialize LEDManager if not already done
        if (ledManager == null) {
            ledManager = new LEDManager(hardware.leds, alliance.equals("RED"));
        }
        ledManager.startMatch();  // Set to solid alliance color

        // Default to middle spike if no reading
        if (obeliskReading == -1) {
            obeliskReading = 22;
            if (alliance.equals("BLUE")) {
                targetSpike = AutonConstants.blueSpike2;
                targetSpikePost = AutonConstants.blueSpike2Post;
            } else {
                targetSpike = AutonConstants.redSpike2;
                targetSpikePost = AutonConstants.redSpike2Post;
            }
            collectionTargetX = targetSpikePost.getX();
            telemetry.addData("WARNING", "No obelisk read - defaulting to Spike 2");
        }

        // Build paths now that we know the target
        buildPaths();
        setPathState(0);
    }

    private void buildPaths() {
        if (alliance.equals("BLUE")) {
            // Blue alliance paths
            initialToScore = follower.pathBuilder()
                    .addPath(AutonConstants.blueLongScorePath(AutonConstants.blueLongStart))
                    .build();
        } else {
            // Red alliance paths
            initialToScore = follower.pathBuilder()
                    .addPath(AutonConstants.redLongScorePath(AutonConstants.redLongStart))
                    .build();
        }

        telemetry.addData("Paths", "Built for %s Alliance, Tag %d", alliance, obeliskReading);
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.update();

            // Update 3-shot if running
            if (threeShots.isBusy()) {
                int targetTagId = alliance.equals("BLUE") ? BLUE_TAG_ID : RED_TAG_ID;
                threeShots.update(targetTagId);
            }

            // Update collection
            updateCollection();

            autonomousPathUpdate();

            // Debug telemetry
            telemetry.addData("Alliance", alliance);
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
                // Drive to score position (alliance-specific)
                telemetry.addData("Action", "Moving to score position");
                follower.followPath(initialToScore, true);
                setPathState(1);
                break;

            case 1:
                // Wait to reach score position, then shoot preloaded samples
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "At score position, starting 3-shot");
                    threeShots.start(1600);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for 3-shot to complete, then move to spike
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
                // Wait until at spike, then start slow collection drive
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "At spike, starting collection drive");
                    startCollectionDrive();
                    setPathState(4);
                }
                break;

            case 4:
                // Drive slowly to collection target while collecting
                if (!isCollecting) {
                    telemetry.addData("Action", "Collection complete, returning to score");

                    // Stop shooter reverse
                    hardware.shooter.setPower(0.0);

                    // Build path back to score from current position (alliance-specific)
                    if (alliance.equals("BLUE")) {
                        collectionToScore = follower.pathBuilder()
                                .addPath(AutonConstants.blueLongScorePath(follower.getPose()))
                                .build();
                    } else {
                        collectionToScore = follower.pathBuilder()
                                .addPath(AutonConstants.redLongScorePath(follower.getPose()))
                                .build();
                    }
                    follower.followPath(collectionToScore, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Wait until back at score, then shoot collected balls
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

                    // Build path to alliance-specific LongLoad
                    PathChain pathToLoad;
                    if (alliance.equals("BLUE")) {
                        pathToLoad = follower.pathBuilder()
                                .addPath(AutonConstants.blueLongLoadPath(follower.getPose()))
                                .build();
                    } else {
                        pathToLoad = follower.pathBuilder()
                                .addPath(AutonConstants.redLongLoadPath(follower.getPose()))
                                .build();
                    }
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
        // Blue (180°): forward = -X, Red (0°): forward = +X
        double distanceToTarget = alliance.equals("BLUE")
            ? currentX - collectionTargetX  // Blue: flip calculation
            : collectionTargetX - currentX;  // Red: normal calculation

        if (Math.abs(distanceToTarget) < 2.0 ||
                collectionTimer.getElapsedTimeSeconds() >= COLLECTION_TIME_SEC) {
            stopCollection();
            return;
        }

        double drive = Math.signum(distanceToTarget) * COLLECTION_DRIVE_SPEED;

        // Heading correction: maintain spike heading (Blue: 180°, Red: 0°)
        double targetHeading = alliance.equals("BLUE") ? Math.toRadians(180) : 0.0;
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
