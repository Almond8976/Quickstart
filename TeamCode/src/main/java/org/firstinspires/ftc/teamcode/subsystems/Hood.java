package org.firstinspires.ftc.teamcode.subsystems;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.HashMap;

public class Hood {

    private Servo hood;
    private double x, y;

    public static boolean tracking = false;
    private double pos;

    public static double minHood = .97, maxHood = .35, closeHood = .6, farHood = .40;
    private static Pose pose;
    public static boolean TEST = false;
    public static double dist;

    public Hood(HardwareMap hardwareMap, HashMap<String, String> config, Pose startPos) {
        pose = startPos;
        hood = hardwareMap.get(Servo.class, config.get("hood"));
    }

    public void setHoodPosition(double pos) {
        this.pos = pos;
    }

    /*
    public void calcHoodPos(double dist) {
            pos = dist < 115 ? closeHood : farHood;
    }
    */
    public void testing(boolean t) {
        TEST = t;
    }

    public double getHoodPosition() {
        return pos;
    }

    public void hoodIncrement (double inc, double pos) {
        if (!(pos + inc > minHood) && !(pos + inc < maxHood)) {
            this.pos = pos + inc;
        }
        else if (pos + inc > minHood) {
            this.pos = minHood;
        }
        else {
            this.pos = maxHood;
        }
    }

    public void update() {
        hood.setPosition(pos);
    }
}
