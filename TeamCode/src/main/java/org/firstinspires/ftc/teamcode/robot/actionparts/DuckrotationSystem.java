package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.Servo;

public class DuckrotationSystem {
    //The Servo to lift the arm up and down
    private Servo duckServo;



    public DuckrotationSystem(Servo duckServo){
        this.duckServo = duckServo;
    }

    /**
     * Initialize the Capstone System
     */
    public void initialize(){
    //    duckServo.setDirection(Servo.Direction.FORWARD);
    }

    /**
     * Grabs the wobble goal
     */
    public void rotateClockWise() {
      //  duckServo.setPosition();
    }

    /**
     * Grabs the wobble goal
     */
    public void rotateAntiClockWise() {
      //  duckServo.setPosition();
    }
}
