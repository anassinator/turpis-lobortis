/**
 * Claw.java
 * Grabs, lifts and drops objects
 * @author  Anass Al-Wohoush
 * @version 0.1
 */

import lejos.nxt.*;
import lejos.util.*;

public class Claw {

    // SLAVE NXT
    public NXTRegulatedMotor claw;

    public Claw(Robot robot) {
        this.claw = robot.claw;
    }

    public void drop() {
        claw.setSpeed(100);
        claw.rotate(4*180 - 45);
    }

    public void grab() {
        claw.setSpeed(100);
        claw.rotate(-4*360);
    }
}
