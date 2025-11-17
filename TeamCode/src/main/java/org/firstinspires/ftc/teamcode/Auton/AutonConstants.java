package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class AutonConstants {
    // Blue poses
    public static final Pose blueLongStart = new Pose(54, 8, Math.toRadians(270)); // Start position from long shot88,8
    public static final Pose blueLongLoad = new Pose(32, 13, Math.toRadians(180)); // Load position on long shot110,13
    public static final Pose blueLongScore = new Pose(52, 14, Math.toRadians(291)); // Scoring position on long shot
    public static final Pose blueShortStart = new Pose(35, 133, Math.toRadians(0)); // Start position from short shot88,14
    public static final Pose blueSpike1 = new Pose(42, 83, Math.toRadians(180)); // P-P-G 23
    public static final Pose blueSpike2 = new Pose(42, 59, Math.toRadians(180)); // P-G-P 22
    public static final Pose blueSpike3 = new Pose(42, 35, Math.toRadians(180)); // G-P-P 21
    public static final Pose blueSpike1Post = new Pose(22, 83, Math.toRadians(180)); // P-P-G post
    public static final Pose blueSpike2Post = new Pose(22, 59, Math.toRadians(180)); // P-G-P post
    public static final Pose blueSpike3Post = new Pose(22, 35, Math.toRadians(180)); // G-P-P post
    public static final Pose blueShortScore = new Pose(86, 86, Math.toRadians(138)); // Scoring position on short shot
    public static final Pose blueGateRelease = new Pose(20, 70, Math.toRadians(180)); // Position to release the blue gate

    // Red poses (x=72 mirror, heading + 180° normalized)
    public static final Pose redLongStart = new Pose(88, 8, Math.toRadians(270)); //54,11
    public static final Pose redLongLoad = new Pose(110, 13, Math.toRadians(0)); // 0° + 180° = 180°
    public static final Pose redLongScore = new Pose(88, 14, Math.toRadians(250)); // 305° + 180° = 485° - 360° = 125°
    public static final Pose redShortStart = new Pose(109, 133, Math.toRadians(180)); // 0° + 180° = 180°
    public static final Pose redSpike1 = new Pose(102, 83, Math.toRadians(0)); // P-P-G 23
    public static final Pose redSpike2 = new Pose(102, 59, Math.toRadians(0)); // P-G-P 22
    public static final Pose redSpike3 = new Pose(102, 35, Math.toRadians(0)); // G-P-P 21
    public static final Pose redSpike1Post = new Pose(122, 83, Math.toRadians(0)); // 0° + 180° = 180°
    public static final Pose redSpike2Post = new Pose(122, 59, Math.toRadians(0)); // 0° + 180° = 180°
    public static final Pose redSpike3Post = new Pose(122, 35, Math.toRadians(0)); // 0° + 180° = 180°
    public static final Pose redShortScore = new Pose(98, 86, Math.toRadians(57)); // 320° + 180° = 500° - 360° = 140°
    public static final Pose redGateRelease = new Pose(120, 70, Math.toRadians(0)); // Position to release the red gate

    // Blue paths
    public static Path blueLongLoadPath(Pose start) {
        Path path = new Path(new BezierLine(start, blueLongLoad));
        path.setLinearHeadingInterpolation(start.getHeading(), blueLongLoad.getHeading());
        return path;
    }

    public static Path blueLongScorePath(Pose start) {
        Path path = new Path(new BezierLine(start, blueLongScore));
        path.setLinearHeadingInterpolation(start.getHeading(), blueLongScore.getHeading());
        return path;
    }

    public static Path blueSpikeOnePath(Pose start) {
        Path path = new Path(new BezierLine(start, blueSpike1));
        path.setLinearHeadingInterpolation(start.getHeading(), blueSpike1.getHeading());
        return path;
    }

    public static Path blueStartToSpikeOnePath(Pose start) {
        Path path = new Path(new BezierLine(start, blueSpike1));
        path.setLinearHeadingInterpolation(start.getHeading(), blueSpike1.getHeading());
        return path;
    }

    public static Path blueStartToSpikeTwoPath(Pose start) {
        Path path = new Path(new BezierLine(start, blueSpike2));
        path.setLinearHeadingInterpolation(start.getHeading(), blueSpike2.getHeading());
        return path;
    }

    public static Path blueStartToSpikeThreePath(Pose start) {
        Path path = new Path(new BezierLine(start, blueSpike3));
        path.setLinearHeadingInterpolation(start.getHeading(), blueSpike3.getHeading());
        return path;
    }

    public static Path blueSpikeToScoreOnePath(Pose start) {
        Path path = new Path(new BezierLine(start, blueShortScore));
        path.setLinearHeadingInterpolation(start.getHeading(), blueShortScore.getHeading());
        return path;
    }

    public static Path blueSpikeToScoreTwoPath(Pose start) {
        Path path = new Path(new BezierLine(start, blueShortScore));
        path.setLinearHeadingInterpolation(start.getHeading(), blueShortScore.getHeading());
        return path;
    }

    public static Path blueSpikeToScoreThreePath(Pose start) {
        Path path = new Path(new BezierLine(start, blueShortScore));
        path.setLinearHeadingInterpolation(start.getHeading(), blueShortScore.getHeading());
        return path;
    }

    // Red paths
    public static Path redLongLoadPath(Pose start) {
        Path path = new Path(new BezierLine(start, redLongLoad));
        path.setLinearHeadingInterpolation(start.getHeading(), redLongLoad.getHeading());
        return path;
    }

    public static Path redLongScorePath(Pose start) {
        Path path = new Path(new BezierLine(start, redLongScore));
        path.setLinearHeadingInterpolation(start.getHeading(), redLongScore.getHeading());
        return path;
    }

    public static Path redStartToSpikeOnePath(Pose start) {
        Path path = new Path(new BezierLine(start, redSpike1));
        path.setLinearHeadingInterpolation(start.getHeading(), redSpike1.getHeading());
        return path;
    }

    public static Path redStartToSpikeTwoPath(Pose start) {
        Path path = new Path(new BezierLine(start, redSpike2));
        path.setLinearHeadingInterpolation(start.getHeading(), redSpike2.getHeading());
        return path;
    }

    public static Path redStartToSpikeThreePath(Pose start) {
        Path path = new Path(new BezierLine(start, redSpike3));
        path.setLinearHeadingInterpolation(start.getHeading(), redSpike3.getHeading());
        return path;
    }

    public static Path redSpikeToScoreOnePath(Pose start) {
        Path path = new Path(new BezierLine(start, redShortScore));
        path.setLinearHeadingInterpolation(start.getHeading(), redShortScore.getHeading());
        return path;
    }

    public static Path redSpikeToScoreTwoPath(Pose start) {
        Path path = new Path(new BezierLine(start, redShortScore));
        path.setLinearHeadingInterpolation(start.getHeading(), redShortScore.getHeading());
        return path;
    }

    public static Path redSpikeToScoreThreePath(Pose start) {
        Path path = new Path(new BezierLine(start, redShortScore));
        path.setLinearHeadingInterpolation(start.getHeading(), redShortScore.getHeading());
        return path;
    }
    private Path scorePreload;

}