import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.RangeFinderAdaptor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.objectdetection.Feature;
import lejos.robotics.objectdetection.FeatureDetector;
import lejos.robotics.objectdetection.FeatureListener;
import lejos.robotics.objectdetection.RangeFeatureDetector;
 
 
/*
 * This class will allow the EV3 to travel on a surface and to avoid 
 * obstacles that it detects using the infrared sensor.
 * 
 * The program can be exited any time using the EV3 default exit shortcut of pressing 
 * the down button and the enter button simultaneously.
 * The escape button on the EV3 can also be used to exit
 */
public class ObjectAvoid {
     
    static final float MAX_DISTANCE = 50f;
    static final int DETECTOR_DELAY = 1000;
 
    public static void main(String[] args) {  
    	//Creating a new pilot which defines the motor ports as well as wheel diameter and track width
        final DifferentialPilot robot = new DifferentialPilot(4.0,18.0,Motor.B, Motor.A);
        //Initializing the IR sensor for the EV3.
        EV3IRSensor ir = new EV3IRSensor(SensorPort.S4);
        RangeFeatureDetector detector = new RangeFeatureDetector(new RangeFinderAdaptor(ir.getDistanceMode()), MAX_DISTANCE, DETECTOR_DELAY);
        
        //Enables detection and starts the Ev3 moving forward
        detector.enableDetection(true);
        robot.forward();
         
        //When Ev3 detects obstacle it will travel backwards and rotate to avoid obstacle.
        detector.addListener(new FeatureListener() {
            public void featureDetected(Feature feature, FeatureDetector detector) {
                detector.enableDetection(false);
                robot.travel(-30);
                robot.rotate(30);
                detector.enableDetection(true);
                robot.forward();
            }       
        });
         
        while(Button.ESCAPE.isUp()) Thread.yield();
        ir.close();
    }
}