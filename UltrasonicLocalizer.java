package Project;


import java.awt.Button;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lejos.hardware.*;

/**
 * @author Kartik Misra, Alexis Franche
 * 
 * The robot will approximate location of 0 degree point using the
 * ultrasonic sensor
 * 
 * @implements UltrasonicController, Runnable
 */
public class UltrasonicLocalizer implements UltrasonicController, Runnable {
	public static float ROTATE_SPEED = 70;

	public UltrasonicPoller poller;
	
	public static boolean locdone = false;

	private double UPDATED_ANGLE;

	public double FIRST_ANGLE = 0;
	public double SECOND_ANGLE = 0;

	private int filterControl;

	private Navigate navigation;

	public enum LocType {FALLING_EDGE, RISING_EDGE}
	public LocType locType;
	private int distance;

	private double firstAngleFE, secondAngleFE, firstAngleRE, secondAngleRE;

	public Odometer odometer;

	public UltrasonicLocalizer(Odometer odometer, LocType locType, UltrasonicPoller poller){ //constructor
		this.odometer = odometer;
		this.locType = locType;
		this.poller = poller;
	}

	/**
	 * @author Kartik Misra, Alexis Franche
	 * 
	 * run method that executes the full ultrasonic localization operation
	 * 
	 * @run
	 */
	public void run(){
		//		Lab5.rightMotor.setSpeed(Lab5.LOC_SPEED);
		//		Lab5.leftMotor.setSpeed(Lab5.LOC_SPEED);
		if (this.locType == LocType.FALLING_EDGE) {
			try {
				fallingEdge();
				//				Lab5.leftMotor.setSpeed(0);
				//				Lab5.rightMotor.setSpeed(0);
				Navigate.leftMotor.stop();
				Navigate.rightMotor.stop();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		else if (this.locType == LocType.RISING_EDGE) {
			risingEgde(distance);
		}
		locdone = true;
	}

	/** 
	 * @author Kartik Misra, Alexis Franche
	 * 
	 * The robot will rotate and scan the falling edge of the wall to figure our where it is relative
	 * to the (0,0) point. Falling edge will switch directions when it detects a wall
	 * 
	 * @param input distance from constructor
	 * @throws InterruptedException 
	 * @void 
	 */
	public void fallingEdge () throws InterruptedException {
		Sound.beep();
		ProjectMain.usDistance.fetchSample(ProjectMain.usData, 0);
		this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
		TimeUnit.SECONDS.sleep(2);
		//rotate until can't detect wall
		while (this.distance <= 38) {
			ProjectMain.usDistance.fetchSample(ProjectMain.usData, 0);
			this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
			Navigate.leftMotor.forward();
			Navigate.rightMotor.backward();
		}
		//rotate until it sees a wall
		while (this.distance > 38) {
			ProjectMain.usDistance.fetchSample(ProjectMain.usData, 0);
			this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
			//Sound.beep();
			Navigate.leftMotor.forward();
			Navigate.rightMotor.backward();
		}
		//get angle
		firstAngleFE = odometer.getTheta();
		//rotate until can't see wall
		while (this.distance <= 38) {
			ProjectMain.usDistance.fetchSample(ProjectMain.usData, 0);
			this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
			//Sound.beepSequence();
			Navigate.leftMotor.backward();
			Navigate.rightMotor.forward();
		}
		//rotate until can see wall
		while (this.distance > 38) {
			ProjectMain.usDistance.fetchSample(ProjectMain.usData, 0);
			this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
			//Sound.beepSequence();
			Navigate.leftMotor.backward();
			Navigate.rightMotor.forward();
		}
		//get angle
		secondAngleFE = odometer.getTheta();
		double avgAngleFE = (firstAngleFE + secondAngleFE)/2;
		double zeroPointFE =  secondAngleFE - avgAngleFE - 45;
		Navigate.turnTo(zeroPointFE);
		Navigate.turnTo(90);
		//		odometer.setXYT(0, 0, 0);
		//
		//		Lab5.leftMotor.rotate(Navigate.convertAngle(2.1, 13.6, 90), true);
		//		Lab5.rightMotor.rotate(-Navigate.convertAngle(2.1, 13.6, 90), false);
		//		Lab5.leftMotor.rotate(Navigate.convertDistance(2.1, 6), true);
		//		Lab5.rightMotor.rotate(Navigate.convertDistance(2.1, 6), false);
		//		Lab5.leftMotor.rotate(-Navigate.convertAngle(2.1, 13.6, 90), true);
		//		Lab5.rightMotor.rotate(Navigate.convertAngle(2.1, 13.6, 90), false);
		//		Lab5.leftMotor.rotate(Navigate.convertDistance(2.1, 6), true);
		//		Lab5.rightMotor.rotate(Navigate.convertDistance(2.1, 6), false);
		//		Lab5.leftMotor.rotate(Navigate.convertAngle(2.1, 13.6, 90), true);
		//		Lab5.rightMotor.rotate(-Navigate.convertAngle(2.1, 13.6, 90), false);
		//		odometer.setXYT(0, 0, 0);
		Navigate.leftMotor.stop();
		Navigate.rightMotor.stop();
	}

	/**
	 * @author Kartik Misra, Alexis Franche
	 * 
	 * The robot will rotate and scan the rising edge of the wall to figure our where it is relative
	 * to the (0,0) point. Rising edge will switch directions when it detects no wall
	 * 
	 * @param input distance from constructor
	 * @void 
	 */
	public void risingEgde(int distance) {
		//		Lab5.leftMotor.setSpeed(ROTATE_SPEED);
		//		Lab5.rightMotor.setSpeed(ROTATE_SPEED);
		Sound.beep();
		//rotate until can't detect wall
		while (this.distance >= 40) {
			Navigate.leftMotor.forward();
			Navigate.rightMotor.backward();
			System.out.println("          " + this.distance);
		}
		//rotate until it sees a wall
		while (this.distance < 40) {
			Navigate.leftMotor.forward();
			Navigate.rightMotor.backward();
			System.out.println("          " + this.distance);
		}
		//get angle
		firstAngleRE = odometer.getTheta();
		//rotate until can't see wall
		while (this.distance >= 40) {
			Navigate.leftMotor.backward();
			Navigate.rightMotor.forward();
			System.out.println("          " + this.distance);
		}
		//rotate until can see wall
		while (this.distance < 40) {
			Navigate.leftMotor.backward();
			Navigate.rightMotor.forward();
			System.out.println("          " + this.distance);
		}
		//get angle
		secondAngleRE = odometer.getTheta();
		if(firstAngleRE > firstAngleRE){
			firstAngleRE = firstAngleRE - 360;
		}
		double avgAngleRE = (firstAngleRE + secondAngleRE)/2;
		double zeroPointRE =  secondAngleRE - avgAngleRE + 45;
		//turnTo(zeroPointRE + 80);
		odometer.setXYT(0, 0, 0);
		Navigate.leftMotor.rotate(Navigate.convertAngle(2.1, 13.6, 90), true);
		Navigate.rightMotor.rotate(-Navigate.convertAngle(2.1, 13.6, 90), false);
		Navigate.leftMotor.rotate(Navigate.convertDistance(2.1, 6), true);
		Navigate.rightMotor.rotate(Navigate.convertDistance(2.1, 6), false);
		Navigate.leftMotor.rotate(-Navigate.convertAngle(2.1, 13.6, 90), true);
		Navigate.rightMotor.rotate(Navigate.convertAngle(2.1, 13.6, 90), false);
		Navigate.leftMotor.rotate(Navigate.convertDistance(2.1, 6), true);
		Navigate.rightMotor.rotate(Navigate.convertDistance(2.1, 6), false);
		odometer.setXYT(0, 0, 0);
		Navigate.leftMotor.stop();
		Navigate.rightMotor.stop();
	}

	/**
	 * @author Team 3
	 * 
	 * Calculates the angle at which to turn to after detecting both walls 
	 * 
	 * @param first_angle, detect wall the first time
	 * @param second_angle, detect wall the second time 
	 * @void
	 */
	public void updateAngle(double first_angle, double second_angle) {
		// Computes the correct angle to turn using the the angles collected
		if (first_angle < second_angle) {
			UPDATED_ANGLE = 45 - ((first_angle + second_angle) / 2);
			Navigate.turn(UPDATED_ANGLE);
		}
		if (first_angle > second_angle) {
			UPDATED_ANGLE = 225 - ((first_angle + second_angle) / 2);
			Navigate.turn(UPDATED_ANGLE);
		}
		if (locType == LocType.FALLING_EDGE) {
			UPDATED_ANGLE = UPDATED_ANGLE + 2;
			Navigate.turn(UPDATED_ANGLE);
		}
		else if (locType == LocType.RISING_EDGE) {
			UPDATED_ANGLE = UPDATED_ANGLE + 200;
			Navigate.turn(UPDATED_ANGLE);
		}
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
