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


    //sample position values
    //todo change these values

    private final int ELEVATOR_POSITION_LOW = 0;
    private final int ELEVATOR_POSITION_MED = 1440;
    private final int ELEVATOR_POSITION_HIGH = 2*1440;



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
        setLiftElevatorLow();
    }

    public void setLiftElevatorLow () {
        setElevatorHeight(this.ELEVATOR_POSITION_LOW);
    }

    public void setLiftElevatorMedium () {
        setElevatorHeight(this.ELEVATOR_POSITION_MED);
    }

    public void setLiftElevatorHigh () {
        setElevatorHeight(this.ELEVATOR_POSITION_HIGH);
    }
    /**
     * setting the delivery arms position
     */
    private void setElevatorHeight(int elevatorHeight){
        liftElevator.setTargetPosition(elevatorHeight);
        // Turn On RUN_TO_POSITION
        liftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftElevator.setPower(Math.abs(0.5));
    }






    public void tiltBucket() {tiltBucket.setPosition(TILT_BUCKET);
    }

    /**
     * Untilts
     */
    public void unTiltBucket() { tiltBucket.setPosition(UNTILT_BUCKET);
    }


}
