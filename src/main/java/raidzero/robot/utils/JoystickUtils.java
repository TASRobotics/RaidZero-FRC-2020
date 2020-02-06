package raidzero.robot.utils;

import raidzero.robot.Constants;

public class JoystickUtils {

    /**
     * Returns a deadbanded input value for joysticks.
     * 
     * @param input joystick input in [-1.0, 1.0]
     * @return deadbanded input
     */
    public static double deadband(double input) {
        if (Math.abs(input) < Constants.JOYSTICK_DEADBAND) {
            return 0.0;
        }
        return input;
    }

    /**
     * Scales a joystick input using a monomial.
     * 
     * @param input joystick input in [-1, 1]
     * @param exp power of the monomial
     * @param coef coefficient of the monomial
     * @return scaled input
     */
    public static double monomialScale(double input, double exp, 
        double coef) {
        return Math.copySign(coef * Math.pow(input, exp), input);
    }
}