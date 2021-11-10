package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DuckrotationSystem {
    //The Servo to lift the arm up and down
    private CRServo duckCRServo;
    private static int ENCODER_VALUE_FOR_1_CAROUSEL_TURN = 1000;


    public DuckrotationSystem(CRServo duckServo) {
        this.duckCRServo = duckServo;
    }

    /**
     * Initialize the Delivery Arm System
     */

    public void initialize() {
        duckCRServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Grabs the wobble goal
     */
    public void rotateClockWise() {
        duckCRServo.setPower(1);
    }

    public void rotateAntiClockWise() {
        duckCRServo.setPower(-1);
    }

    public void stop() {
        duckCRServo.setPower(0);
    }
}
