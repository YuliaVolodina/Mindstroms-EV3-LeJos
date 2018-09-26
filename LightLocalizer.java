package Project;


import java.util.concurrent.TimeUnit;
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
 * @uthors Kartik MIsra, Alexis Franche
 * 
 * The robot will locate the (0,0) and 0 degree point precisely
 * by scanning grid lines 
 * 
 * @implements UltrasonicController, Runnable
 */

public class LightLocalizer implements UltrasonicController, Runnable {
	private int distance;

	private static final double OFFSET = 14.2;
	public double odo_x,odo_y, odo_theta;
	public double curx, cury;
	public boolean isDone = false;

	private double point1, point2, point3, point4;
	private double thetaY, thetaX, thetaFinal;
	private double xPos, yPos;

	private static Lock lock = new ReentrantLock(true);
	private volatile boolean isReseting = false;
	private Condition doneReseting = lock.newCondition();

	public Odometer odometer;

	public LightLocalizer(Odometer odometer) throws OdometerExceptions{ //constructor
		this.odometer = Odometer.getOdometer();
	}

	public void run(){
		try {
			lightLocalizer();

		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	/**
	 * @author Kartik Misra, Alexis Franche
	 * 
	 * Localization using the light sensor, after completing a 360 degree rotation
	 * and scanning the grid lines the robot will find the origin of the board
	 * 
	 * @void
	 */
	public void lightLocalizer() throws InterruptedException {
		Navigate.leftMotor.stop();
		Navigate.rightMotor.stop();
		odometer.setXYT(0, 0, 0);
		TimeUnit.SECONDS.sleep(2);
		double turnedAngle;
		int lineCount = 0;
		while(lineCount < 4) {
			while (true) {
				ProjectMain.leftSensorVal.fetchSample(ProjectMain.leftSensorValData, 0);
				Navigate.leftMotor.forward();
				Navigate.rightMotor.backward();
				Navigate.rightMotor.setSpeed(ProjectMain.LOC_SPEED);
				Navigate.leftMotor.setSpeed(ProjectMain.LOC_SPEED);

				if (ProjectMain.leftSensorValData[0] < 0.42) {
					//Sound.beep();
					lineCount++;
					if(lineCount == 1) point1 = odometer.getTheta();
					if(lineCount == 2) point2 = odometer.getTheta();
					if(lineCount == 3) point3 = odometer.getTheta();
					if(lineCount == 4) point4 = odometer.getTheta();
				}

				if(odometer.getTheta() > 358 && odometer.getTheta() <= 360) {
					//TODO check this
					Navigate.leftMotor.stop(true);
					Navigate.rightMotor.stop(false);
					break;
				}
				if(isDone) {break;}
			}
		}
		turnedAngle = odometer.getTheta();

		thetaX = point4 - point2;
		thetaY = point3 - point1;

		xPos = -OFFSET*(Math.cos(Math.PI*thetaX/360));
		yPos = -OFFSET*(Math.cos(Math.PI*thetaY/360));

		thetaFinal = 180 - thetaY/2 - point1;

		odometer.setXYT(xPos, yPos, 0);

		travelTo(0,0);
		Navigate.turnToLoc(280);
	}


	/**
	 * @author Kartik Misra, ALexis Franche
	 * 
	 * Calculates the distance and angle to reach the input coordinates, converts them in cm and commands the robot to move to 
	 * that direction 
	 * 
	 * @param x and y values to travel to
	 * @void commands the robot to rotate and set the travel distance to reach destination
	 * 
	 */
	public void travelTo(double pointX, double pointY){
		double[] location = new double[3];
		location = odometer.getXYT();
		double acutalTheta = location[2];
		double actualX = location[0];
		double actualY = location[1];
		odo_x = actualX;
		odo_y = actualY;
		odo_theta = acutalTheta;
		double finalX = pointX;
		double finalY = pointY;
		double yDiff = finalY-odo_y;
		double xDiff = finalX-odo_x;
		double angleCalc = Math.toDegrees(Math.atan2(xDiff,yDiff));
		double distFinal = Math.hypot(xDiff,yDiff);
		double angleFinal = angleCalc - odo_theta;
		if(angleFinal < -180){ 
			Navigate.turnToLoc(angleFinal + 360);
		}
		else if(angleFinal > 180){
			Navigate.turnToLoc(angleFinal - 360);
		}
		else{
			Navigate.turnToLoc(angleFinal);
		}
		Navigate.goTo(distFinal);
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
