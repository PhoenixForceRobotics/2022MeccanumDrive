package frc.robot.utils;

public class PIDValues {
  private double P;
  private double I;
  private double D;
  private double FF;
  private double peakOutput;

  public PIDValues(double P, double I, double D, double FF, double peakOutput) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.FF = FF;
    this.peakOutput = peakOutput;
  }

  public double getP() {
    return P;
  }

  public double getI() {
    return I;
  }

  public double getD() {
    return D;
  }

  public double getFF() {
    return FF;
  }

  public double getPeakOutput() {
    return peakOutput;
  }
}
