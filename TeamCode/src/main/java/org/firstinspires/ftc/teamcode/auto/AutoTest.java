package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Mortar;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Util;

@Autonomous
public class AutoTest extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;



    Util util;
    Kicker kicker;
    Mortar shooter;
    Turret turret;
    Intake intake;
    Gate gate;
    Hood hood;
    public static int KICKER_WAIT_TIME = 600;

    private int shooterTargetSpeed;
    private int launchCount, shootPoseCount, launchIf;
    private double target;
    private boolean closeGate = false, stopLaunch = false;

    ElapsedTime time1 = new ElapsedTime();
    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_START_POS_SHOOT_POS,
        SHOOT_PRELOAD,
        SPIKE_ONE,
        RETURN_SHOOT1,
        SET_UP2,
        SPIKE_TWO,
        RETURN_SHOOT2,
        SET_UP3,
        SPIKE_THREE,
        RETURN_SHOOT3,
        SET_UP_HUMAN,
        HUMAN,
        RETURN_SHOOT_HUMAN,
        DONE
    }

    PathState pathState;

    private final Pose startPose = new Pose(113.93684210526318,129.4315789473684, Math.toRadians(42.5));
    private final Pose shootPose = new Pose(93.64210526315794,83.49473684210525, Math.toRadians(0));
    private final Pose spike1 = new Pose(130, 84, Math.toRadians(0));
    private final Pose setUp2 = new Pose(96.25263157894737, 59.284210526315775, Math.toRadians(0));
    private final Pose spike2 = new Pose(135,59,Math.toRadians(0));
    private final Pose setUp3 = new Pose(96.25263157894737,35.368421052631575, Math.toRadians(0));
    private final Pose spike3 = new Pose(135, 35, Math.toRadians(0));
    private final Pose setUpH = new Pose(135, 52, Math.toRadians(270));
    private final Pose humanPose = new Pose(135,7, Math.toRadians(270));

    private PathChain driveStartPosShootPos;

    private PathChain spikeOne, spikeTwo, spikeThree;
    private PathChain returnToShoot1, returnToShoot2, returnToShoot3, setUpTwo, setUpThree, setUpHuman, human, returnShootHuman;


    @Override
    public void runOpMode() throws InterruptedException {
        // init all subsystems (switch to using wrappers if you want parallel movement)
        pathState = PathState.DRIVE_START_POS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        // TODO add in any other init mechanisms

        follower = Constants.createFollower(hardwareMap);
        util = new Util();
        kicker = new Kicker(hardwareMap, util.deviceConf);
        shooter = new Mortar(hardwareMap, util.deviceConf);
        turret = new Turret(hardwareMap, util.deviceConf, new Pose(64.5, 16.4, Math.toRadians(42.5)));
        intake = new Intake(hardwareMap, util.deviceConf);
        gate = new Gate(hardwareMap, util.deviceConf);
        hood = new Hood(hardwareMap, util.deviceConf, new Pose(64.5, 16.4, Math.toRadians(42.5)));

        turret.setBasketPos(Turret.redBasket);

        buildPaths();
        follower.setPose(startPose);

        Thread update = new Thread( ()-> updateAll(turret, shooter, kicker, intake, gate, follower));

        // TODO: move everything to start position (after init, before program start)



        waitForStart();
        update.start();
        shooter.setVelocity(1420);
        Turret.tracking = true;

        switch(pathState) {
            case DRIVE_START_POS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                // check is follower done its path?
                if (!follower.isBusy()) {
                    if(opModeTimer.getElapsedTime() >= 1750 && !stopLaunch) {
                        Launch();
                        shootPoseCount++;
                        follower.followPath(spikeOne, true);
                        intake.setAllPower(1);
                        setPathState(PathState.SPIKE_ONE);
                    }
                }
                break;
            case SPIKE_ONE:
                if (!follower.isBusy()) {
                    follower.followPath(returnToShoot1, true);
                    intake.setAllPower(0);
                    setPathState(PathState.RETURN_SHOOT1);
                }
                break;
            case RETURN_SHOOT1:
                if (!follower.isBusy()) {
                    Launch();
                    shootPoseCount++;
                    telemetry.addLine("two" + opModeTimer.getElapsedTime());
                    follower.followPath(setUpTwo, true);
                    intake.setAllPower(0);
                    setPathState(PathState.SET_UP2);
                }
                break;
            case SET_UP2:
                if (!follower.isBusy()) {
                    follower.followPath(spikeTwo, true);
                    intake.setAllPower(1);
                    setPathState(PathState.SPIKE_TWO);
                }
            case SPIKE_TWO:
                if (!follower.isBusy()) {
                    follower.followPath(returnToShoot2, true);
                    intake.setAllPower(0);
                    setPathState(PathState.RETURN_SHOOT2);
                }
            case RETURN_SHOOT2:
                if (!follower.isBusy()) {
                    Launch();
                    shootPoseCount++;
                    follower.followPath(setUpThree, true);
                    intake.setAllPower(0);
                    setPathState(PathState.SET_UP3);
                }
                break;
            case SET_UP3:
                if (!follower.isBusy()) {
                    follower.followPath(spikeThree, true);
                    intake.setAllPower(1);
                    setPathState(PathState.SPIKE_THREE);
                }
            case SPIKE_THREE:
                if (!follower.isBusy()) {
                    follower.followPath(returnToShoot3, true);
                    intake.setAllPower(0);
                    setPathState(PathState.RETURN_SHOOT3);
                }
            case RETURN_SHOOT3:
                if (!follower.isBusy()) {
                    Launch();
                    shootPoseCount++;
                    telemetry.addLine("four" + opModeTimer.getElapsedTime());
                    follower.followPath(setUpHuman, true);
                    intake.setAllPower(0);
                    setPathState(PathState.SET_UP_HUMAN);
                }
            case SET_UP_HUMAN:
                if (!follower.isBusy()) {
                    follower.followPath(human, true);
                    intake.setAllPower(1);
                    setPathState(PathState.HUMAN);
                }
            case HUMAN:
                if (!follower.isBusy()) {
                    follower.followPath(returnShootHuman);
                    setPathState(PathState.RETURN_SHOOT_HUMAN);
                    intake.setAllPower(0);
                }
            case RETURN_SHOOT_HUMAN:
                if (!follower.isBusy()) {
                    Launch();
                    shootPoseCount++;
                    telemetry.addLine("five" + opModeTimer.getElapsedTime());
                    setPathState(PathState.DONE);
                    intake.setAllPower(0);
                }
            default:
                telemetry.addLine("Auto Finished" + opModeTimer.getElapsedTime());
                shooter.setFlyMotorPower(0);
                shooter.setFlyMotor2Power(0);
                break;
        }

    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    // Define all functions here (if you call subsystems movements from here it wont be parallel)

    public void Launch() {
        shooter.setVelocity(1420);
        intake.setAllPower(0);
        do {
            gate.setPosition(Gate.OPEN);
        }
        while (shooter.getVelocity() < shooterTargetSpeed - Mortar.THRESH || shooter.getVelocity()>shooterTargetSpeed);
        intake.setAllPower(1);
        sleep(KICKER_WAIT_TIME);
        //intake.setIntakePower(0);
        //kicker.setPosition(Kicker.UP);
        //intake.setIntakePower(0);
        //sleep(500);
        //kicker.setPosition(Kicker.DOWN);

        gate.setPosition(Gate.CLOSE);
        //intake.setIntakePower(1);
    }

    public void updateAll(Turret turret, Mortar shooter, Kicker kicker, Intake intake, Gate gate, Follower follower){
        while (opModeInInit() || opModeIsActive())
        {
            shooter.update();
            kicker.update();
            turret.update();
            intake.update();
            gate.update();
            follower.update();
            statePathUpdate();

            telemetry.addData("path state", pathState.toString());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Gate Position", gate.getPosition());
            telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
            telemetry.addData("Follower Busy: ", follower.isBusy());
            telemetry.addData("Target Speed", shooter.getTargetVelocity());
            telemetry.addData("Flywheel Velocity", shooter.getVelocity());
            telemetry.addData("Hood Position", hood.getHoodPosition());
            telemetry.addData("shootPoseCount", shootPoseCount);
            telemetry.addData("launchCount", ((double)launchCount/3.0));
            telemetry.addData("LaunchIf", launchIf);
        }
    }
    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration(5)
                .build();
        spikeOne = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, spike1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), spike1.getHeading())
                .build();
        returnToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(spike1, shootPose))
                .setLinearHeadingInterpolation(spike1.getHeading(), shootPose.getHeading())
                .build();
        setUpTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, setUp2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), setUp2.getHeading())
                .build();
        spikeTwo = follower.pathBuilder()
                .addPath(new BezierLine(setUp2,spike2))
                .setLinearHeadingInterpolation(setUp2.getHeading(), spike2.getHeading())
                .build();
        returnToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(spike2, shootPose))
                .setLinearHeadingInterpolation(spike2.getHeading(), shootPose.getHeading())
                .build();
        setUpThree = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, setUp3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), setUp3.getHeading())
                .build();
        spikeThree = follower.pathBuilder()
                .addPath(new BezierLine(setUp3, spike3))
                .setLinearHeadingInterpolation(setUp3.getHeading(), spike3.getHeading())
                .build();
        returnToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3, shootPose))
                .setLinearHeadingInterpolation(spike3.getHeading(), shootPose.getHeading())
                .build();
        setUpHuman = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, setUpH))
                .setLinearHeadingInterpolation(shootPose.getHeading(), setUpH.getHeading())
                .build();
        human = follower.pathBuilder()
                .addPath(new BezierLine(setUpH, humanPose))
                .setLinearHeadingInterpolation(setUpH.getHeading(), humanPose.getHeading())
                .build();
        returnShootHuman = follower.pathBuilder()
                .addPath(new BezierLine(humanPose, shootPose))
                .setLinearHeadingInterpolation(humanPose.getHeading(), shootPose.getHeading())
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
                    if(opModeTimer.getElapsedTime() >= 1750 && !stopLaunch) {
                        Launch();
                        shootPoseCount++;
                        follower.followPath(spikeOne, true);
                        intake.setAllPower(1);
                        setPathState(PathState.SPIKE_ONE);
                    }
                }
                break;
            case SPIKE_ONE:
                if (!follower.isBusy()) {
                    follower.followPath(returnToShoot1, true);
                    intake.setAllPower(0);
                    setPathState(PathState.RETURN_SHOOT1);
                }
                break;
            case RETURN_SHOOT1:
                if (!follower.isBusy()) {
                    Launch();
                    shootPoseCount++;
                    telemetry.addLine("two" + opModeTimer.getElapsedTime());
                    follower.followPath(setUpTwo, true);
                    intake.setAllPower(0);
                    setPathState(PathState.SET_UP2);
                }
                break;
            case SET_UP2:
                if (!follower.isBusy()) {
                    follower.followPath(spikeTwo, true);
                    intake.setAllPower(1);
                    setPathState(PathState.SPIKE_TWO);
                }
            case SPIKE_TWO:
                if (!follower.isBusy()) {
                    follower.followPath(returnToShoot2, true);
                    intake.setAllPower(0);
                    setPathState(PathState.RETURN_SHOOT2);
                }
            case RETURN_SHOOT2:
                if (!follower.isBusy()) {
                    Launch();
                    shootPoseCount++;
                    follower.followPath(setUpThree, true);
                    intake.setAllPower(0);
                    setPathState(PathState.SET_UP3);
                }
                break;
            case SET_UP3:
                if (!follower.isBusy()) {
                    follower.followPath(spikeThree, true);
                    intake.setAllPower(1);
                    setPathState(PathState.SPIKE_THREE);
                }
            case SPIKE_THREE:
                if (!follower.isBusy()) {
                    follower.followPath(returnToShoot3, true);
                    intake.setAllPower(0);
                    setPathState(PathState.RETURN_SHOOT3);
                }
            case RETURN_SHOOT3:
                if (!follower.isBusy()) {
                    Launch();
                    shootPoseCount++;
                    telemetry.addLine("four" + opModeTimer.getElapsedTime());
                    follower.followPath(setUpHuman, true);
                    intake.setAllPower(0);
                    setPathState(PathState.SET_UP_HUMAN);
                }
            case SET_UP_HUMAN:
                if (!follower.isBusy()) {
                    follower.followPath(human, true);
                    intake.setAllPower(1);
                    setPathState(PathState.HUMAN);
                }
            case HUMAN:
                if (!follower.isBusy()) {
                    follower.followPath(returnShootHuman);
                    setPathState(PathState.RETURN_SHOOT_HUMAN);
                    intake.setAllPower(0);
                }
            case RETURN_SHOOT_HUMAN:
                if (!follower.isBusy()) {
                    Launch();
                    shootPoseCount++;
                    telemetry.addLine("five" + opModeTimer.getElapsedTime());
                    setPathState(PathState.DONE);
                    intake.setAllPower(0);
                }
            default:
                telemetry.addLine("Auto Finished" + opModeTimer.getElapsedTime());
                shooter.setFlyMotorPower(0);
                shooter.setFlyMotor2Power(0);
                break;
        }
    }
}
