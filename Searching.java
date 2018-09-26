package Project;


import java.util.concurrent.TimeUnit;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


import lejos.hardware.Sound;
import lejos.hardware.device.LMotor;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * @author Team 3
 * 
 * Class that manages the our inital search algorithm
 * 
 * @implements UltrasonicController, Runnable
 */

public class Searching implements UltrasonicController, Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odo;
	private static Navigate navigate;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor servoMotor;
	private static final double CHEAT_DISTANCE = 30;
	private static double TURN_ANGLE;
	private static final double TILE_SIZE = 30.48;
	private int distance ;

	public double LLX = 2;
	public double LLY = 2;
	public double URX = 5;
	public double URY = 5;
	double dimension = URY - LLY;
	//double dimensionX = URX - LLX;
	public double position[] = new double[3];

	public ArrayList<Double> dist2 = new ArrayList<Double>();
	public ArrayList<Double> dist1 = new ArrayList<Double>();
	public ArrayList<Double> angle1 = new ArrayList<Double>();
	public ArrayList<Double> angle2 = new ArrayList<Double>();


	double wheel_radius = ProjectMain.WHEEL_RAD;
	double width = ProjectMain.TRACK;
	private static final int FORWARD_SPEED = 150;
	private static final int ROTATE_SPEED = 75;
	public double odo_x,odo_y, odo_theta;
	public double curx, cury;

	private static Lock lock = new ReentrantLock(true);
	private volatile boolean isReseting = false;
	private Condition doneReseting = lock.newCondition();

	public Odometer odometer;

	public Searching(Odometer odometer){
		this.odometer = odometer;
	}


	public void run(){
		try {
			experimentalSquareDriver();
		}
		catch(InterruptedException e) {
			e.printStackTrace();
		}
	}

	/**
	 * @author Kartik Misra
	 * 
	 * travel path around search zone and scan objects until target block is found
	 * 
	 * @void
	 * @throws InterruptedException
	 */
	public void experimentalSquareDriver() throws InterruptedException {
		if (!(odometer.getXYT()[0] == LLX && odometer.getXYT()[1] == LLY)) {
			travelTo(LLX, LLY);
		}
		travelTo(LLX, LLY+dimension);
		firstCorner();			
		goToBlock1();
		/*travelTo(LLX+dimensionX, LLY);
		oppositeCorner();
		goToBlock2();*/		
		Navigate.rightMotor.stop();
		Navigate.leftMotor.stop();
	}

	/**
	 * @author Alexis Franche, Wiam El Ouadi
	 * 
	 * travels to the bottom left corner of the search zone where the search algorithm starts 
	 * 
	 * @void
	 */
	public void firstCorner() {
		travelTo(LLX+dimension, LLY+dimension);
		turnTo(90);
		scanObject();
	}

	/**
	 * @author Alexis Franche, Wiam El Ouadi
	 * 
	 * travels to the bottom top right of the search zone where we search again
	 * 
	 * @void
	 */
	public void oppositeCorner() {
		travelTo(LLX, LLY);
		turnTo(90);
		scanObject2();
	}

	/**
	 * @author Alexis Franche, Wiam El Ouadi
	 * 
	 * When a block is detected during the 90 degrees sweep, save angle and distance in their respective arraylists (first corner)
	 * 
	 * @void
	 */
	public void scanObject() {
		for(int k=10; k<91 ; k+=10) {
			turnTo((10));
			if (this.distance < dimension*20) {
				double temp = this.distance;
				double tempAngle = odometer.getTheta();
				dist1.add(new Double(temp));
				angle1.add(new Double(tempAngle));
			}
		}
	}

	/**
	 * @author Alexis Franche, Wiam El Ouadi
	 * 
	 * When a block is detected during the 90 degrees sweep, save angle and distance in their respective arraylists (second corner)
	 * 
	 * @void
	 */
	public void scanObject2() {
		for(int k=10; k<91 ; k+=10) {
			turnTo((10));
			if (this.distance < dimension*20) {
				double temp = this.distance;
				double tempAngle = odometer.getTheta();
				dist2.add(new Double(temp));
				angle2.add(new Double(tempAngle));
			}
		}
	}

	/**
	 * @author Alexis Franche, Wiam El Ouadi
	 * 
	 * travels the distance recorded in the scanObject method at the right angle which 
	 * correspond to a block that has to be identified (from upper corner)
	 * 
	 * @void
	 */
	public void goToBlock1() {
		if (angle1.size() > 0 && dist1.size()>0) {
			for(int k = 0; k<angle1.size(); k++) {
				turnTo(angle1.get(k).doubleValue() - odometer.getTheta());
				goTo(dist1.get(k).doubleValue());
				if(this.distance < 6) {
					turnTo(180);
				}
				if (ColorID.yellowDetected()) {
					travelTo(URX,URY);
					Navigate.rightMotor.stop();
					Navigate.leftMotor.stop();
					break;
				}
				else {
					travelTo(URX,URY);
					continue;
				}
			}
		} 
	}

	/**
	 * @author Alexis Franche, Wiam El Ouadi
	 * 
	 * travels the distance recorded in the scanObject method at the right angle which 
	 * correspond to a block that has to be identified (from lower corner)
	 * 
	 * @void
	 */
	public void goToBlock2() {
		if (angle2.size() > 0 && dist2.size()>0) {
			for(int k = 0; k<angle2.size(); k++) {
				turnTo(angle2.get(k).doubleValue()- odometer.getTheta());
				goTo(dist2.get(k).doubleValue());
				if(this.distance < 6) {
					turnTo(180);
				}
				if (ColorID.yellowDetected()) {
					travelTo(LLX,LLY);
					travelTo(URX,URY);
					Navigate.rightMotor.stop();
					Navigate.leftMotor.stop();
					break;
				}
				else {
					travelTo(LLX,LLY);
					continue;
				}
			}
		} 
	}


	/**
	 * @author Alexis Franche, Wiam El Ouadi
	 * 
	 * Calculates the distance and angle to reach the input coordinates, converts them in cm and commands the robot to move to 
	 * that direction.
	 *  
	 * @param x coordinate to travel to
	 * @param y coordinate to travel to
	 * @void
	 */
	public void travelTo(double x, double y) {
		x = x*TILE_SIZE;
		y = y*TILE_SIZE;
		position=odometer.getXYT();	
		double currX = position[0];
		double currY = position[1];	
		double distX = x - currX;
		double distY = y - currY;
		double destTheta = Math.toDegrees(Math.atan2(distX, distY));
		double distance = Math.sqrt( Math.pow(distX, 2) + Math.pow(distY, 2));
		turnTo(destTheta);
		Navigate.leftMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, (distance)), true);
		Navigate.rightMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, (distance)), false);
		//		while(Navigate.isNavigating()) {
		//			continue;
		//		}
	}

	/**
	 * @author Alexis Franche, Kartik Misra
	 * 
	 * Commands the robot to move exactly the input distance
	 * 
	 * @param distance
	 */
	public void goTo(double distance){
		Navigate.leftMotor.setSpeed(FORWARD_SPEED);
		Navigate.rightMotor.setSpeed(FORWARD_SPEED);

		Navigate.leftMotor.rotate(Navigate.convertDistance(wheel_radius, distance), true);
		Navigate.rightMotor.rotate(Navigate.convertDistance(wheel_radius, distance), false);
	}

	/**
	 * @author Alexis Franche, Kartik Misra
	 * 
	 * Commands the robot to rotate to the input distance.
	 * 
	 * @param theta
	 */
	public void turnTo(double theta){
		Navigate.leftMotor.setSpeed(ROTATE_SPEED);
		Navigate.rightMotor.setSpeed(ROTATE_SPEED);
		double turnTheta = theta - position[2];
		if(turnTheta <-180) {
			turnTheta += 360;
		}	
		else if(turnTheta>180) {
			turnTheta-=360;
		}
		Navigate.leftMotor.rotate(Navigate.convertAngle(ProjectMain.WHEEL_RAD , ProjectMain.TRACK, turnTheta), true);
		Navigate.rightMotor.rotate(-Navigate.convertAngle(ProjectMain.WHEEL_RAD, ProjectMain.TRACK,turnTheta), false);
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
