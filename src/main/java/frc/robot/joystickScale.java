package frc.robot;
import edu.wpi.first.math.MathUtil;

public class joystickScale {

    public static double ScaleJoystick(double input, double deadBand, double exponent) {

        double output = 0;
        double sign = 0;

        if (input < 0) {
            sign = -1;
            input = input * sign;
        } else if(input > 0) {
            sign = 1;
        }

        MathUtil.applyDeadband(input, deadBand);
        output = Math.pow(input * sign, 1.8);
        
        return output;
    }
}
