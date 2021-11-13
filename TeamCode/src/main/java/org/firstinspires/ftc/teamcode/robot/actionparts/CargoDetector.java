package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CargoDetector {
    private final static double  SKYSTONE_COLOR_INDEX = 2;
    private ColorSensor colorSensor;
    private OpMode curOpMode;
    private DistanceSensor disSensor;


    public CargoDetector(ColorSensor sensorColor, DistanceSensor distanceSensor, OpMode opMode) {
        this.colorSensor = sensorColor;
        this.disSensor = distanceSensor;
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

    public double getCargoDistance(){
        return disSensor.getDistance(DistanceUnit.MM);
    }

    private double getCCValue(ColorSensor colorSensor) {
        return (colorSensor.red() * colorSensor.green()) / (colorSensor.blue()*colorSensor.blue());
    }

    private double getCCOpenOrSS(ColorSensor colorSensor) {
        return Math.round((colorSensor.red() * colorSensor.green() * colorSensor.blue())/10000);
    }

}
