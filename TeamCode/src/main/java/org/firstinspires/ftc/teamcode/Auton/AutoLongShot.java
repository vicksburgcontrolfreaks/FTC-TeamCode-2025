package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
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
    private static final double COLLECTION_MAX_POWER = 0.3;  // Slow collection speed
    private boolean isCollecting = false;

    // Obelisk reading
    private int obeliskReading = -1;  // -1 = not read yet, 21/22/23 = spike mark
    private Pose targetSpike = null;
    private Pose targetSpikePost = null;
    private double collectionTargetX = 72.0;  // Will be set based on spike and alliance

    // Second collection spike
    private Pose secondSpike = null;
    private Pose secondSpikePost = null;
    private double secondCollectionTargetX = 72.0;

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
                if (tag.center.x < allianceThreshold) {
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
        hardware.flipper.setPosition(0.0);
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

        // Determine second collection spike (prefer spike 1, then 2, then 3)
        determineSecondSpike();

        // Build paths now that we know the target
        buildPaths();
        setPathState(0);
    }

    private void determineSecondSpike() {
        // Prefer spike 1 (tag 23), then spike 2 (tag 22), then spike 3 (tag 21)
        if (alliance.equals("BLUE")) {
            if (obeliskReading != 23) {
                secondSpike = AutonConstants.blueSpike1;
                secondSpikePost = AutonConstants.blueSpike1Post;
            } else if (obeliskReading != 22) {
                secondSpike = AutonConstants.blueSpike2;
                secondSpikePost = AutonConstants.blueSpike2Post;
            } else {
                secondSpike = AutonConstants.blueSpike3;
                secondSpikePost = AutonConstants.blueSpike3Post;
            }
        } else {  // RED
            if (obeliskReading != 23) {
                secondSpike = AutonConstants.redSpike1;
                secondSpikePost = AutonConstants.redSpike1Post;
            } else if (obeliskReading != 22) {
                secondSpike = AutonConstants.redSpike2;
                secondSpikePost = AutonConstants.redSpike2Post;
            } else {
                secondSpike = AutonConstants.redSpike3;
                secondSpikePost = AutonConstants.redSpike3Post;
            }
        }
        secondCollectionTargetX = secondSpikePost.getX();
        telemetry.addData("Second Spike", "X=%.1f Y=%.1f", secondSpike.getX(), secondSpike.getY());
    }

    private void buildPaths() {
        if (alliance.equals("BLUE")) {
            // Blue alliance paths - FASTER
            initialToScore = follower.pathBuilder()
                    .addPath(AutonConstants.blueLongScorePath(AutonConstants.blueLongStart))
                    .setConstraints(new com.pedropathing.paths.PathConstraints(0.99, 100, 1.5, 1))
                    .build();
        } else {
            // Red alliance paths - FASTER
            initialToScore = follower.pathBuilder()
                    .addPath(AutonConstants.redLongScorePath(AutonConstants.redLongStart))
                    .setConstraints(new com.pedropathing.paths.PathConstraints(0.99, 100, 1.5, 1))
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
                // Drive to score position (alliance-specific) AND start shooter immediately
                telemetry.addData("Action", "Moving to score position (shooter spinning up)");

                // Start shooter motor immediately
                hardware.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hardware.shooter.setPower(1.0);  // Full power for shooting

                follower.followPath(initialToScore, true);
                setPathState(1);
                break;

            case 1:
                // Wait to reach score position, then shoot preloaded samples
                // Keep shooter running while driving
                hardware.shooter.setPower(1.0);

                if (!follower.isBusy()) {
                    telemetry.addData("Action", "At score position, starting 3-shot");
                    threeShots.start(1300);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for 3-shot to complete, THEN turn on collector and move to spike
                if (!threeShots.isBusy()) {
                    telemetry.addData("Action", "3-shot complete, moving to spike mark %d", obeliskReading);

                    // NOW turn on collector and shooter reverse (after shooting is done)
                    hardware.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hardware.collector.setPower(1.0);
                    hardware.shooter.setPower(-0.10);  // Reverse to prevent jamming

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
                // Wait until at spike, then drive to spikePost while collecting
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "At spike, driving to spikePost while collecting (slow)");

                    // Limit power for slow collection
                    follower.setMaxPower(COLLECTION_MAX_POWER);

                    // Build path from spike to spikePost
                    PathChain chainToPost = follower.pathBuilder()
                            .addPath(new com.pedropathing.geometry.BezierLine(
                                    follower.getPose(),
                                    targetSpikePost))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetSpikePost.getHeading())
                            .build();
                    follower.followPath(chainToPost, false);
                    setPathState(4);
                }
                break;

            case 4:
                // Wait until at spikePost, then return to score
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "At spikePost, returning to score");

                    // Reset to full power for normal driving
                    follower.setMaxPower(1.0);

                    // Stop shooter reverse and collector
                    hardware.shooter.setPower(0.0);
                    hardware.collector.setPower(0.0);

                    // Build path back to SHORT score from current position (alliance-specific)
                    Pose currentPose = follower.getPose();
                    com.pedropathing.paths.Path pathToShortScore;
                    if (alliance.equals("BLUE")) {
                        pathToShortScore = new com.pedropathing.paths.Path(
                                new com.pedropathing.geometry.BezierLine(currentPose, AutonConstants.blueShortScore));
                        pathToShortScore.setLinearHeadingInterpolation(currentPose.getHeading(), AutonConstants.blueShortScore.getHeading());
                    } else {
                        pathToShortScore = new com.pedropathing.paths.Path(
                                new com.pedropathing.geometry.BezierLine(currentPose, AutonConstants.redShortScore));
                        pathToShortScore.setLinearHeadingInterpolation(currentPose.getHeading(), AutonConstants.redShortScore.getHeading());
                    }
                    collectionToScore = follower.pathBuilder()
                            .addPath(pathToShortScore)
                            .build();
                    follower.followPath(collectionToScore, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Wait until back at SHORT score, then shoot collected balls
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "At SHORT score position, starting 3-shot at 1100 TPS");
                    threeShots.start(1100);
                    setPathState(6);
                }
                break;

            case 6:
                // Wait for first cycle 3-shot to complete, THEN turn on collector and move to second spike
                if (!threeShots.isBusy()) {
                    telemetry.addData("Action", "First cycle complete, moving to second spike (X=%.1f Y=%.1f)",
                            secondSpike.getX(), secondSpike.getY());

                    // NOW turn on collector and shooter reverse for second collection (after shooting is done)
                    hardware.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hardware.collector.setPower(1.0);
                    hardware.shooter.setPower(-0.10);  // Reverse to prevent jamming

                    // Build path to second spike with heading interpolation
                    Pose currentPose = follower.getPose();
                    com.pedropathing.paths.Path pathToSecondSpike = new com.pedropathing.paths.Path(
                            new com.pedropathing.geometry.BezierLine(currentPose, secondSpike));
                    pathToSecondSpike.setLinearHeadingInterpolation(currentPose.getHeading(), secondSpike.getHeading());

                    PathChain chainToSecondSpike = follower.pathBuilder()
                            .addPath(pathToSecondSpike)
                            .build();
                    follower.followPath(chainToSecondSpike, false);
                    setPathState(7);
                }
                break;

            case 7:
                // Wait until at second spike, then drive to secondSpikePost while collecting
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "At second spike, driving to spikePost while collecting (slow)");

                    // Limit power for slow collection
                    follower.setMaxPower(COLLECTION_MAX_POWER);

                    // Build path from second spike to secondSpikePost
                    PathChain chainToSecondPost = follower.pathBuilder()
                            .addPath(new com.pedropathing.geometry.BezierLine(
                                    follower.getPose(),
                                    secondSpikePost))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), secondSpikePost.getHeading())
                            .build();
                    follower.followPath(chainToSecondPost, false);
                    setPathState(8);
                }
                break;

            case 8:
                // Wait until at secondSpikePost, then return to score
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "At second spikePost, returning to score");

                    // Reset to full power for normal driving
                    follower.setMaxPower(1.0);

                    // Stop shooter reverse and collector
                    hardware.shooter.setPower(0.0);
                    hardware.collector.setPower(0.0);

                    // Build path back to SHORT score from current position (alliance-specific)
                    Pose currentPose2 = follower.getPose();
                    com.pedropathing.paths.Path pathToShortScore2;
                    if (alliance.equals("BLUE")) {
                        pathToShortScore2 = new com.pedropathing.paths.Path(
                                new com.pedropathing.geometry.BezierLine(currentPose2, AutonConstants.blueShortScore));
                        pathToShortScore2.setLinearHeadingInterpolation(currentPose2.getHeading(), AutonConstants.blueShortScore.getHeading());
                    } else {
                        pathToShortScore2 = new com.pedropathing.paths.Path(
                                new com.pedropathing.geometry.BezierLine(currentPose2, AutonConstants.redShortScore));
                        pathToShortScore2.setLinearHeadingInterpolation(currentPose2.getHeading(), AutonConstants.redShortScore.getHeading());
                    }
                    collectionToScore = follower.pathBuilder()
                            .addPath(pathToShortScore2)
                            .build();
                    follower.followPath(collectionToScore, true);
                    setPathState(9);
                }
                break;

            case 9:
                // Wait until back at SHORT score, then shoot second batch of collected balls
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "At SHORT score position, starting second 3-shot at 1100 TPS");
                    threeShots.start(1100);
                    setPathState(10);
                }
                break;

            case 10:
                // Wait for second final 3-shot to complete, then move to gate release
                if (!threeShots.isBusy()) {
                    telemetry.addData("Action", "Moving to gate release");

                    // Build path to alliance-specific gate release
                    Pose currentPose = follower.getPose();
                    Pose gateRelease = alliance.equals("BLUE")
                            ? AutonConstants.blueGateRelease
                            : AutonConstants.redGateRelease;

                    com.pedropathing.paths.Path pathToGate = new com.pedropathing.paths.Path(
                            new com.pedropathing.geometry.BezierLine(currentPose, gateRelease));
                    pathToGate.setLinearHeadingInterpolation(currentPose.getHeading(), gateRelease.getHeading());

                    PathChain chainToGate = follower.pathBuilder()
                            .addPath(pathToGate)
                            .build();
                    follower.followPath(chainToGate, true);
                    setPathState(11);
                }
                break;

            case 11:
                // Wait to reach gate release
                if (!follower.isBusy()) {
                    telemetry.addData("Action", "Complete! At gate release");
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

        double turnCorrection = headingError * 0.6;  // Increased P gain for stronger heading correction

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
