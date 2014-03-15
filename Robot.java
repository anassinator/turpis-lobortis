/**
 * Robot.java
 * Contains robot's properties
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 0.1
 */

import lejos.nxt.*;
import lejos.util.*;

public class Robot {
    public final double leftRadius = 2.16;                      // RADIUS OF LEFT WHEEL
    public final double rightRadius = 2.13;                     // RADIUS OF RIGHT WHEEL
    public final double width = 16.30;                          // DISTANCE BETWEEN WHEELS
    public final double distanceBetweenColorSensors = 14.25;    // DISTANCE BETWEEN DOWN-FACING COLOR SENSORS
    public final double distanceToUltrasonicSensors = 14.25;    // DISTANCE FROM CENTER TO ULTRASONIC SENSORS
    
    // STORE MOTORS AND ULTRASONIC SENSOR
    public NXTRegulatedMotor claw = Motor.C,leftMotor = Motor.B, rightMotor = Motor.A;

    public static UltrasonicSensor leftSonic    = new UltrasonicSensor(SensorPort.S1);
    public static UltrasonicSensor centerSonic  = new UltrasonicSensor(SensorPort.S2);
    public static UltrasonicSensor rightSonic   = new UltrasonicSensor(SensorPort.S3);

    public static ColorSensor leftColor         = new ColorSensor(SensorPort.S3);
    public static ColorSensor centerColor       = new ColorSensor(SensorPort.S1);
    public static ColorSensor rightColor        = new ColorSensor(SensorPort.S2);

    // STORE WHETHER TRYING TO LOCALIZE OR NOT
    public boolean localizing = false;
}
