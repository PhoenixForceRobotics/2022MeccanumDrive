package frc.robot.utils;

public class PIDValues {
  private double P;
  private double I;
  private double D;

  public PIDValues(double P, double I, double D) {
    this.P = P;
    this.I = I;
    this.D = D;
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
}
