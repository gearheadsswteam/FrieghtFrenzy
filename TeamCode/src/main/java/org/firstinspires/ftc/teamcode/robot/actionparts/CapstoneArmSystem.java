package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.Servo;

public class CapstoneArmSystem {

    //The Servo to lift the arm up and down
    private Servo liftServo;

    //Servo used to grab the capstone
    private Servo grabServo;

    //Servo Positions
    private final double INIT_POSITON = 0.43;
    private final double FLOOR_POSITION = 0.79;
    private final double CLOSED_POSITION = 0.6;
    private final double OPEN_POSITION = 0.47;

    private boolean isOpen = true;

    /**
     * Constructor
     *
     * @param liftServo
     * @param grabServo
     */
    public CapstoneArmSystem(Servo liftServo, Servo grabServo) {
        this.liftServo = liftServo;
        this.grabServo = grabServo;
    }

    /**
     * Initialize the Capstone System
     */
    public void initialize() {
        ungrabCapstone();
        armUp();
    }

    /**
     * Grabs the wobble goal
     */
    public void grabCapstone() {
        grabServo.setPosition(CLOSED_POSITION);
        isOpen = false;
    }

    /**
     * Ungrabs the wobble goal
     */
    public void ungrabCapstone() {
        grabServo.setPosition(OPEN_POSITION);
        isOpen = true;
    }

    /**
     * Lifts the wobble goal post
     */

    public void armUp() {
        liftServo.setPosition(INIT_POSITON);
    }

    /**
     * Sets the wobble goal post down
     */
    public void armDown() {
        liftServo.setPosition(FLOOR_POSITION);
    }


    public void moveCapstoneArm(int capstonearmState) {

        switch (capstonearmState) {
            case 0:
                ungrabCapstone();
                armUp();
                break;

            case 1:
                ungrabCapstone();
                armDown();
                break;

            case 2:
                grabCapstone();
                armDown();
                break;

            case 3:
                grabCapstone();
                armUp();
                break;
        }
    }

    public void setArmPositon(double postionToSet) {
        if(postionToSet < FLOOR_POSITION && postionToSet > INIT_POSITON) {
            liftServo.setPosition(postionToSet);
        }
    }


    public void lowerArm() {
        double curPositon = liftServo.getPosition();
        double newPositon = curPositon - 0.05;
        if (newPositon > this.INIT_POSITON) {
            liftServo.setPosition(newPositon);
        }
    }

    public void toogleGrip(){
        if(isOpen){
            grabServo.setPosition(CLOSED_POSITION);
            isOpen = false;
        }
        if(!isOpen){
            grabServo.setPosition(OPEN_POSITION);
            isOpen = true;
        }
    }
}
