package Project;
/*
 * OdometryCorrection.java
 */

import java.util.concurrent.TimeUnit;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * @author Kartik Misra
 * 
 * Class that performs line correction to realign the robot using 2 light sensors
 *
 * @implement Runnable
 */
public class OdometryCorrection implements Runnable {
	private Odometer odometer;
	private boolean lineDetected;
	private boolean done;
	private boolean leaningLeft;

	private double initVal, secondVal;

	private double theta;

	/**
	 * @author Kartik Misra
	 * 
	 * This is the default class constructor. An existing instance of the odometer is used. This is to
	 * ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
	}

	/**
	 * @author Kartik Misra
	 * 
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	public void run() {
		lineDetected = false;
		done = false;
		correctY();
	}

	/**
	 * @author Kartik Misra
	 * 
	 * Modification of the initial odometry correction class, here we will use two sensors
	 * to correct the robot's angle, but not x and y position
	 * 
	 * @void
	 */
	public void correctY() {
		//		Lab5.leftMotor.setSpeed(80);
		//		Lab5.rightMotor.setSpeed(80);
		Navigate.leftMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, -7), true);
		Navigate.rightMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, -7), false);
		Sound.beep();
		//TimeUnit.SECONDS.sleep(2);
		Navigate.leftMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, 30),true);
		Navigate.rightMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, 30), true);
		while(true) {
			ProjectMain.leftSensorVal.fetchSample(ProjectMain.leftSensorValData, 0);
			ProjectMain.rightSensorVal.fetchSample(ProjectMain.rightSensorValData, 0);
			if(ProjectMain.leftSensorValData[0]<0.27 && !lineDetected) {
				Sound.beep();
				ProjectMain.rightSensorVal.fetchSample(ProjectMain.rightSensorValData, 0);
				initVal = odometer.getXYT()[1];
				lineDetected = true;
				while (ProjectMain.rightSensorValData[0] > 0.27) {
					ProjectMain.rightSensorVal.fetchSample(ProjectMain.rightSensorValData, 0);
					if (ProjectMain.rightSensorValData[0]<0.27) {
						Sound.beep();
						secondVal = odometer.getXYT()[1];
						theta = (Math.atan((secondVal-initVal)/ProjectMain.offset))*(180/Math.PI);
						done = true;
						leaningLeft = false;
						break;
					}
				}
			}
			if(ProjectMain.rightSensorValData[0]<0.27 && !lineDetected) {
				Sound.beep();
				initVal = odometer.getXYT()[1];
				lineDetected = true;
				while(ProjectMain.leftSensorValData[0] > 0.27) {
					ProjectMain.leftSensorVal.fetchSample(ProjectMain.leftSensorValData, 0);
					if (ProjectMain.leftSensorValData[0]<0.27) {
						Sound.beep();
						secondVal = odometer.getXYT()[1];
						theta = (Math.atan((secondVal-initVal)/ProjectMain.offset))*(180/Math.PI);
						done = true;
						leaningLeft = true;
						break;
					}
				}
			}
			if (done) {
				break;
			}
		}
		if (leaningLeft) {
			Navigate.turnTo(theta);
		} else {
			Navigate.turnTo(-theta);
		}
		//TimeUnit.SECONDS.sleep(2);
		ProjectMain.leftSensorVal.fetchSample(ProjectMain.leftSensorValData, 0);
		Navigate.leftMotor.backward();
		Navigate.rightMotor.backward();
		//		Lab5.leftMotor.setSpeed(50);
		//		Lab5.rightMotor.setSpeed(50);
		while (ProjectMain.leftSensorValData[0] > 0.27) {
			ProjectMain.leftSensorVal.fetchSample(ProjectMain.leftSensorValData, 0);
			if(ProjectMain.leftSensorValData[0] < 0.27) {
				Sound.twoBeeps();
				Navigate.leftMotor.stop(true);
				Navigate.rightMotor.stop(false);
			}
		}
		Navigate.leftMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, 2.5),true);
		Navigate.rightMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, 2.5), false);
		Navigate.leftMotor.stop(true);
		Navigate.rightMotor.stop(false);
		//TimeUnit.SECONDS.sleep(1);
	}

	/**
	 * @author Kartik Misra
	 * 
	 * Command the robot to stop when either of the two sensors detects a line
	 * 
	 * @void
	 * @throws InterruptedException
	 */
	public void detectLine() {
		//TimeUnit.SECONDS.sleep(2);
		Navigate.leftMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, 20),true);
		Navigate.rightMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, 20), true);
		ProjectMain.leftSensorVal.fetchSample(ProjectMain.leftSensorValData, 0);
		while (ProjectMain.leftSensorValData[0] > 0.27) {
			ProjectMain.leftSensorVal.fetchSample(ProjectMain.leftSensorValData, 0);
			if(ProjectMain.leftSensorValData[0] < 0.27) {
				Sound.twoBeeps();
				Navigate.leftMotor.stop(true);
				Navigate.rightMotor.stop(false);
				//TimeUnit.SECONDS.sleep(2);
			}
		}
	}

}
