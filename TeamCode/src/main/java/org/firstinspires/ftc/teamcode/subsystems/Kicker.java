package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

@Configurable
public class Kicker {
    ElapsedTime time1 = new ElapsedTime();
    private Servo kicker;

    public static double DOWN = 0.75, UP = 1.0;
    private double position = DOWN;

    public Kicker(HardwareMap hardwareMap, HashMap<String, String> config) {
        kicker = hardwareMap.get(Servo.class, config.get("kicker"));
    }

    public void setPosition(double kickerPositon){
        position = kickerPositon;
    }

    public void sweep(){
        time1.reset();
        kicker.setPosition(UP);
        if(time1.milliseconds() > 500) {
            kicker.setPosition(DOWN);
        }
    }

    public void update() {
        kicker.setPosition(position);
    }

    public double getPosition() {
        return kicker.getPosition();
    }
}
