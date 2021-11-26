package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class DeliveryArmSystem {

    //The Motor to lift the Elevator
    private DcMotor liftElevator;

    //Servo to tilt the bucket
    private Servo tiltBucket;

    private boolean isTilted = false;

    //Bucket Positions
    private final double BUCKET_DOWN = 0.80;
    private final double BUCKET_UP = 0.58;
    private final double BUCKET_REST = 0.36;


    //position values
    private final int ELEVATOR_POSITION_LOW = 0;
    private final int ELEVATOR_POSITION_MED = -360;
    private final int ELEVATOR_POSITION_HIGH = -940;

    private LinearOpMode curOpMode;


    /**
     * Constructor
     *
     * @param liftElevator
     * @param tiltBucket
     */
    public DeliveryArmSystem(DcMotor liftElevator, Servo tiltBucket, LinearOpMode curOpMode) {
        this.liftElevator = liftElevator;
        this.tiltBucket = tiltBucket;
        this.curOpMode = curOpMode;
    }

    /**
     * Initialize the Delivery Arm System
     */

    public void initialize() {
        liftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftElevator.setPower(1);
        setLiftElevatorLow();
    }

    public void setLiftElevatorLow() {
        setElevatorHeight(this.ELEVATOR_POSITION_LOW);
    }

    public void setLiftElevatorMedium() {
        setElevatorHeight(this.ELEVATOR_POSITION_MED);
    }

    public void setLiftElevatorHigh() {
        setElevatorHeight(this.ELEVATOR_POSITION_HIGH);
    }

    /**
     * setting the delivery arms position
     */
    private void setElevatorHeight(int elevatorHeight) {
        liftElevator.setTargetPosition(elevatorHeight);
        // Turn On RUN_TO_POSITION
        liftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftElevator.setPower(Math.abs(1));
    }

    boolean bucketMoveComplete = true;

    public void moveBucket() {
        if (bucketMoveComplete) {
            bucketMoveComplete = false;
            if (isTilted) {
                bucketUp();
            } else {
                bucketDown();
            }
            bucketMoveComplete = true;
        }
    }

    public void bucketDown() {
        tiltBucket.setPosition(BUCKET_DOWN);
        curOpMode.sleep(700);
    }

    /**
     * Untilts
     */
    public void bucketUp() {
        tiltBucket.setPosition(BUCKET_UP);
        curOpMode.sleep(700);
    }

    /**
     * Untilts
     */
    public void bucketRest() {
        tiltBucket.setPosition(BUCKET_REST);
        curOpMode.sleep(700);
    }
}
