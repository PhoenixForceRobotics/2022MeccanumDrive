package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.UtilConstants;

public class Falcon extends WPI_TalonFX {

  public Falcon(int portID, boolean isInverted) {
    super(portID);
    setInverted(isInverted);

    configPeakOutputForward(1);
    configPeakOutputReverse(-1);
  }

  public void PIDconfig(FalconConfigFields falconConfigFields) {
    // Disable motor
    setPercentage(0);

    // Resets to remove any errors
    configFactoryDefault();

    // Setting it to the minimum allows for more precision
    configNeutralDeadband(0.001);

    // Assigns the PID values to each slot
    int index = 0;
    for (PIDValues falconPID : falconConfigFields.getFalconPIDs()) {
      config_kP(index, falconPID.getD());
      config_kI(index, falconPID.getI());
      config_kD(index, falconPID.getD());
      config_kF(index, falconPID.getFF());
      configClosedLoopPeakOutput(index, falconPID.getPeakOutput());

      // sets the speed of each loop. can be slowed if sensor updates are too fast
      // or  derivative is changing too slowly
      configClosedLoopPeriod(index, UtilConstants.CLOSED_LOOP_SPEED_MS);
    }
  }

  public void setCoast() {
    setNeutralMode(NeutralMode.Coast);
  }

  public void setBrake() {
    setNeutralMode(NeutralMode.Brake);
  }

  public void setPercentage(double percentage) {
    set(ControlMode.PercentOutput, percentage);
  }

  public void setCurrent(double current) {
    set(ControlMode.Current, current);
  }

  @Override
  public void setVoltage(double outputVolts) {
    // TODO Auto-generated method stub
    super.setVoltage(outputVolts);
  }

  public void setVelocity(double velocity) {
    set(ControlMode.Velocity, convertRPMToCTRE(velocity));
  }

  public void setPosition(double position) {
    set(ControlMode.Position, convertRotationsToCTRE(position));
  }

  // Returns number of rotations
  public double getPosition() {
    return super.getSelectedSensorPosition() / UtilConstants.FALCON_ENCODER_RESOLUTION;
  }

  // Returns number of rotations per minute
  public double getVelocity() {
    return (super.getSelectedSensorVelocity() / UtilConstants.FALCON_ENCODER_RESOLUTION) * 600;
  }

  public double getRawVelocity() {
    return getSelectedSensorVelocity();
  }

  public static double convertRotationsToCTRE(double rotations) {
    return rotations * UtilConstants.FALCON_ENCODER_RESOLUTION;
  }

  public static double convertRPMToCTRE(double rpm) {
    return rpm * UtilConstants.FALCON_ENCODER_RESOLUTION / 600;
  }
}
