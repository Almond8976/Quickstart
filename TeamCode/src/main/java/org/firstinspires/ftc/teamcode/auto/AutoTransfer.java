package org.firstinspires.ftc.teamcode.auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoTransfer extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_START_POS_SHOOT_POS,
        SHOOT_PRELOAD,
        SPIKE_ONE,
        SPIKE_TWO,
        SPIKE_THREE
    }

    PathState pathState;

    private final Pose startPose = new Pose(113.93684210526318,129.4315789473684, Math.toRadians(128.118));
    private final Pose shootPose = new Pose(96.25263157894737,83.49473684210525, Math.toRadians(0));
    private final Pose spike1 = new Pose(130, 84, Math.toRadians(0));
    private final Pose spike2 = new Pose(135,59,Math.toRadians(0));

    private PathChain driveStartPosShootPos;

    private PathChain spikeOne, spikeTwo, spikeThree;


    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        spikeOne = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, spike1))
                .build();



    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_START_POS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                // check is follower done its path?
                if (!follower.isBusy()) {
                    // TODO add logic to flywheel shooter
                    //TODO intake on
                    follower.followPath(spikeOne, true);
                    setPathState(PathState.SPIKE_ONE);
                    //TODO intake off
                }
                break;
            case SPIKE_ONE:
                if (!follower.isBusy()) {
                    // TODO add logic to flywheel shooter
                    //TODO intake on

                    setPathState(PathState.SPIKE_TWO);
                    //TODO intake off
                }
                break;
            case SPIKE_TWO:
                if (!follower.isBusy()) {
                    // TODO add logic to flywheel shooter
                    //TODO intake on
                    setPathState(PathState.SPIKE_THREE);
                    //TODO intake off
                }
                break;
            case SPIKE_THREE:
                // TODO add logic to flywheel shooter
            default:
                telemetry.addLine("Auto Finished");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_START_POS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }
}
