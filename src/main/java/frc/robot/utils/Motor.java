package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Motor extends CANSparkMax {
  private double gearRatio;
  private double wheelDiameter;
  private PIDController positionPID;
  private PIDController velocityPID;
  private SimpleMotorFeedforward feedforward;

  public Motor(int port, boolean reversed, double gearRatio, double wheelDiameter, PIDController positionPID, PIDController velocityPID, SimpleMotorFeedforward feedforward) 
  {
    super(port, MotorType.kBrushless);
    
    this.gearRatio = gearRatio;
    this.wheelDiameter = wheelDiameter;
    this.positionPID = positionPID;
    this.velocityPID = velocityPID; // TODO: Decide whether I want to use this PID or the integrated PID
    this.feedforward = feedforward;

    setInverted(reversed);
  }

  public Motor(int port, boolean reversed, double gearRatio, double wheelDiameter, PIDController positionPID, PIDController velocityPID)
  {
    this(port, reversed, gearRatio, wheelDiameter, positionPID, velocityPID, new SimpleMotorFeedforward(0, 0));
  } 
  public Motor(int port, boolean reversed, double gearRatio, double wheelDiameter)
  {
    this(port, reversed, gearRatio, wheelDiameter, new PIDController(0, 0, 0), new PIDController(0, 0, 0));
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

  @Override
  public SparkMaxPIDController getPIDController() {
    // TODO Auto-generated method stub
    return super.getPIDController();
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

  public boolean isReversed()
  {
    return getInverted();
  }
}
