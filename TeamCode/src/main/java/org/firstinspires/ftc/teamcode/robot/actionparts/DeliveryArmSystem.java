package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class DeliveryArmSystem {

    //The Motor to lift the Elevator
    private DcMotor liftElevator;

    //Servo to tilt the bucket
    private Servo tiltBucket;

    private boolean isTilted = false;

    //Servo Positions
    private final double TILT_BUCKET = 0.83;
    private final double UNTILT_BUCKET = 0.36;


    //position values

    private final int ELEVATOR_POSITION_LOW = 0;
    private final int ELEVATOR_POSITION_MED = -361;
    private final int ELEVATOR_POSITION_HIGH = -918;



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

    boolean bucketMoveComplete = true;

    public void moveBucket() {
        if(bucketMoveComplete) {
            bucketMoveComplete = false;
            if (isTilted) {
                unTiltBucket();
            } else {
                tiltBucket();
            }
            bucketMoveComplete = true;
        }
    }

    public void tiltBucket() {
        if(!isTilted) {
            tiltBucket.setPosition(TILT_BUCKET);
            isTilted = true;
        }
    }

    /**
     * Untilts
     */
    public void unTiltBucket() {
        if(isTilted) {
            tiltBucket.setPosition(UNTILT_BUCKET);
            isTilted = false;
        }
    }
}
