package Project;

/**
 * Ultrasonic controller interface
 */
public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
