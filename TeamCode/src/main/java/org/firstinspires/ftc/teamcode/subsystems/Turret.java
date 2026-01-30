package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


import java.util.HashMap;

@Configurable
public class Turret {

    //private DcMotor turret;
    private Servo servoTurret;
    private Servo servoTurret2;
    private static Follower follower;
    //private PinpointLocalizer localizer;

    private double ticksPerRad = (384.5*((double)85/25)) / (2*Math.PI);;

    private double rotationLimit = Math.PI * 208;

    public static double rotationSpeed = 1, maxRange = Math.toRadians(350);
    private double x, y, heading, turretHeading, turretHeadingRelative;

    public static boolean tracking = false;

    private double pos;

    private Pose pose;

    public static Pose blueBasket = new Pose(-71,-71);
    public static Pose redBasket = new Pose(-71, 71);
    public static Pose curBasket;

    public static double angleOffset = 0;

    public Turret(HardwareMap hardwareMap, HashMap<String, String> config, Pose startPos) {
        //drive = new MecanumDrive(hardwareMap, startPos); // TODO: set this to whatever position auton will end at
        pose = startPos;
        servoTurret = hardwareMap.get(Servo.class, config.get("turret"));
        servoTurret2 = hardwareMap.get(Servo.class, config.get("turret2"));
//        turret = hardwareMap.get(DcMotor.class, config.get("turretMotor"));
//
//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Pose getPose() {
        return pose;
    }
    public double getTurretHeading() {
        return turretHeading;
    }

    public double getTurretHeadingRelative() { return turretHeadingRelative; }

    public void setBasketPos(Pose pos) {
        curBasket = pos;
    }
    public Pose distanceToBasket() {
        return new Pose(curBasket.getY() -y, curBasket.getX() -x);
    }

    public void setPosition(double degrees) {
        pos = Math.toRadians(degrees);
    }

//    public void resetEncoder() {
//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }

    public void resetRobotPose(Pose newPos) {
        follower.setPose(newPos);
    }

//    public int getTargetPosition() {
//        return turret.getTargetPosition();
//    }

    public void update() {

        //drive.updatePoseEstimate();
        follower.update();
        //pose = drive.localizer.getPose();
        pose = follower.getPose();
        x = follower.getPose().getX();
        y = follower.getPose().getY();
        heading = pose.getHeading();

        turretHeadingRelative = Math.atan2(curBasket.getY() -y, curBasket.getX() -x);
        turretHeading = turretHeadingRelative - heading;

        turretHeading+=angleOffset;

        turretHeading %= 2*Math.PI;
        if(turretHeading>Math.PI) {
            turretHeading -= 2*Math.PI;
        }
        else if(turretHeading<-Math.PI) {
            turretHeading += 2*Math.PI;
        }
        if(turretHeading < -Math.PI/2) {
            turretHeading = -Math.PI/2;
        }
        if(turretHeading > Math.PI/2) {
            turretHeading = Math.PI/2;
        }

        if(tracking) {
            servoTurret.setPosition((-turretHeading / (Math.PI/2))*.27 + .49);
            servoTurret2.setPosition((-turretHeading / (Math.PI/2))*.27 + .49);
        }
        else {
            servoTurret.setPosition((-pos/maxRange) + .49);
            servoTurret2.setPosition((-pos/maxRange) + .49);
        }

        //for turret with motor
//        if (tracking) {
//            turret.setTargetPosition((int) ((turretHeading * ticksPerRad)));
//            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turret.setPower(rotationSpeed);
//        }
//        else {
//            turret.setTargetPosition(pos);
//            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turret.setPower(rotationSpeed);
//        }

    }
}
