package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class CargoDetector {
    private final static double  SKYSTONE_COLOR_INDEX = 2;
    private ColorSensor colorSensor;
    private OpMode curOpMode;


    public CargoDetector(ColorSensor sensorColor, OpMode opMode) {
        this.colorSensor = sensorColor;
        this.curOpMode = opMode;
    }

    public boolean isCargobucketFull() {
//        double cc = (colorSensor.red() * colorSensor.green()) / (colorSensor.blue()*colorSensor.blue());
//        return cc <= SKYSTONE_COLOR_INDEX;
        if (curOpMode.gamepad2.right_trigger > 0) {
            return true;
        }
        return false;
    }

    private double getCCValue(ColorSensor colorSensor) {
        return (colorSensor.red() * colorSensor.green()) / (colorSensor.blue()*colorSensor.blue());
    }

    private double getCCOpenOrSS(ColorSensor colorSensor) {
        return Math.round((colorSensor.red() * colorSensor.green() * colorSensor.blue())/10000);
    }

}
