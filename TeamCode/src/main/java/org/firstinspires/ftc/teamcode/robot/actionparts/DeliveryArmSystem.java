package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class DeliveryArmSystem {

    //The Motor to lift the Elevator
    private DcMotor liftElevator;

    //Servo to tilt the bucket
    private Servo tiltBucket;

    //Servo Positions
    //todo change positions
    private final double TILT_BUCKET = 0.22;
    private final double UNTILT_BUCKET = 0.55;


    /**
     * Constructor
     * @param liftElevator
     * @param tiltBucket
     */
    public DeliveryArmSystem(DcMotor liftElevator, Servo tiltBucket) {
        this.liftElevator = liftElevator;
        this.tiltBucket = tiltBucket;
    }

    /**
     * Initialize the Delivery Arm System
     */

    public void initialize(){
        liftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * setting the delivery arms position
     */
    private void setElevatorHeight(int elevatorHeight){
        //liftElevator.setTargetPosition(elevatorHeight);
        liftElevator.setPower(0.5);
    }

    /**
     * position is in the low setting
     */
    public void setElevatorLow() {
        setElevatorHeight(1);

    }

    /**
     * position is in the medium setting
     */
    public void setElevatorMedium() {
        setElevatorHeight(2);
    }

    /**
     * position is in the high setting
     */
    public void setElevatorHigh() {
        setElevatorHeight(3);
    }


    public void tiltBucket() {tiltBucket.setPosition(TILT_BUCKET);
    }

    /**
     * Untilts
     */
    public void unTiltBucket() { tiltBucket.setPosition(UNTILT_BUCKET);
    }


}
