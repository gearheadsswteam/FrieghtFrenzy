package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CargoDetector {
    private DistanceSensor disSensor;
    private double EMPTY_CARGO_DIST = 50; //Empty distance is 57 mm, anything less means bucket has something


    public CargoDetector(DistanceSensor distanceSensor) {
        this.disSensor = distanceSensor;
    }

    public  boolean isCargoBucketFull(){
        if(disSensor.getDistance(DistanceUnit.MM) < EMPTY_CARGO_DIST){
            return true;
        }else{
            return false;
        }
    }
}
