package frc.robot.utils;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Motor extends CANSparkMax {
  private double gearRatio;
  private double wheelDiameter;
  private PIDController positionPID;
  private PIDController velocityPID;
  private SimpleMotorFeedforward feedforward;

  public Motor(int port, boolean reversed, double gearRatio, double wheelDiameter, PIDValues positionPID, PIDValues velocityPID, SimpleMotorFeedforward feedforward) 
  {
    super(port, MotorType.kBrushless);
    
    this.gearRatio = gearRatio;
    this.wheelDiameter = wheelDiameter;
    this.positionPID = new PIDController(positionPID.getP(), positionPID.getI(), positionPID.getD());
    this.velocityPID = new PIDController(velocityPID.getP(), velocityPID.getI(), velocityPID.getD());
    this.feedforward = feedforward;

    setInverted(reversed);
  }

  public Motor(int port, boolean reversed, double gearRatio, double wheelDiameter, PIDValues positionPID, PIDValues velocityPID)
  {
    this(port, reversed, gearRatio, wheelDiameter, positionPID, velocityPID, new SimpleMotorFeedforward(0, 0));
  } 
  public Motor(int port, boolean reversed, double gearRatio, double wheelDiameter)
  {
    this(port, reversed, gearRatio, wheelDiameter, new PIDValues(0, 0, 0), new PIDValues(0, 0, 0));
  }
  public Motor(int port, boolean reversed)
  {
    this(port, reversed, 1, 1);
  }

  public void setMetersPerSecond(double metersPerSecond) {
    double output = velocityPID.calculate(getMetersPerSecond(), metersPerSecond) + feedforward.calculate(metersPerSecond);
    setVoltage(output);
  }

  public void setMeters(double meters) {
    double output = positionPID.calculate(getMeters(), meters) + feedforward.ks;
    setVoltage(output);
  }

  public void setVelocityP(double kp)
  {
    velocityPID.setP(kp);
  }

  public void setVelocityI(double ki)
  {
    velocityPID.setP(ki);
  }

  public void setVelocityD(double kd)
  {
    velocityPID.setD(kd);
  }

  public double getVelocityP() {
    return velocityPID.getP();
  }
  
  public double getVelocityI() {
    return velocityPID.getI();
  }
  
  public double getVelocityD() {
    return velocityPID.getD();
  }

  public PIDController getVelocityPID() {
    return velocityPID; 
  }

  public PIDController getPositionPID() {
      return positionPID;
  }

  public double getRotations() {
    return getEncoder().getPosition() * gearRatio;
  }

  public double getMeters() {
    return getRotations() * wheelDiameter * Math.PI;
  }

  public double getRPM() {
    return getEncoder().getVelocity() * gearRatio;
  }
  
  public double getMetersPerSecond() {
    return getRPM() * wheelDiameter * Math.PI / 60;
  }

  public double getGearRatio() {
    return gearRatio;
  }

  public double getwheelDiameter() {
    return wheelDiameter;
  }

  public double getVelocityError() {
    return velocityPID.getVelocityError();
  }

  public boolean isReversed()
  {
    return getInverted();
  }
}
