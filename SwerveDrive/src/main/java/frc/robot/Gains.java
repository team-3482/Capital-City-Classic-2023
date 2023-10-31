package frc.robot;

public class Gains {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;
    public final int kIzone;
    public final double kPeakOutput;

    /**
     * Creates and instance of Gains for motion magic
     * 
     * @param _kP
     * @param _kI
     * @param _kD
     * @param _kF
     * @param _kIzone
     * @param _kPeakOutput
     */
    public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput) {
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kF = _kF;
        kIzone = _kIzone;
        kPeakOutput = _kPeakOutput;
    }
}