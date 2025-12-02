package frc.robot.util;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

/**
 * Contains classes for defining control constants and tunable controllers.
 * <p>
 * Uses AdvantageKit's {@link LoggedTunableNumber} for runtime tuning of control.
 */
public class TunableControls {
    /**
     * Contains
     */
    public static class ControlConstants {
        // PID gains
        double kP = 0;
        double kI = 0;
        double kD = 0;
        double tolerance = 0;
        double velTolerance = Double.POSITIVE_INFINITY;
        double iZone = Double.POSITIVE_INFINITY;
        double iMin = Double.NEGATIVE_INFINITY;
        double iMax = Double.POSITIVE_INFINITY;
        double period = 0.02;

        // feedforward gains
        double kV, kA = 0;

        // physical gains
        double kS, kG = 0;

        // trapezoid profile
        boolean profiled = false;
        double maxVel = 0;
        double maxAcc = 0;

        // continuous control
        boolean isContinuous = false;
        double maxInput;
        double minInput;

        public ControlConstants(ControlConstants constants) {
            this.kP = constants.kP;
            this.kI = constants.kI;
            this.kD = constants.kD;
            this.tolerance = constants.tolerance;
            this.velTolerance = constants.velTolerance;
            this.iZone = constants.iZone;
            this.iMax = constants.iMax;
            this.iMin = constants.iMin;
            this.period = constants.period;
            this.kV = constants.kV;
            this.kA = constants.kA;
            this.kS = constants.kS;
            this.kG = constants.kG;
            this.maxVel = constants.maxVel;
            this.maxAcc = constants.maxAcc;
            this.isContinuous = constants.isContinuous;
            this.maxInput = constants.maxInput;
            this.minInput = constants.minInput;
        }

        /**
         * Sets the PID constants.
         * 
         * @param kP proportional gain (units of output per unit error)
         * @param kI integral gain (units of output per unit accumulated error)
         * @param kD derivative gain (units of output per unit error per second)
         * @return this ControlConstants object for chaining
         */
        public ControlConstants withPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this;
        }

        /**
         * Sets the feedforward constants.
         * 
         * @param kV output to give for a velocity of 1 unit per second
         * @param kA output to give for an acceleration of 1 unit per second squared
         * @return this ControlConstants object for chaining
         */
        public ControlConstants withFeedforward(double kV, double kA) {
            this.kV = kV;
            this.kA = kA;
            return this;
        }

        /**
         * Sets the "physical" feedforward constants.
         * 
         * @param kS output to overcome static friction
         * @param kG output to counteract gravity
         * @return this ControlConstants object for chaining
         */
        public ControlConstants withPhysical(double kS, double kG) {
            this.kS = kS;
            this.kG = kG;
            return this;
        }

        /**
         * Sets the trapezoidal motion profile parameters. Will also set the controller
         * to be profiled.
         * 
         * @param maxVel maximum velocity (units per second)
         * @param maxAcc maximum acceleration (units per second squared)
         * @return this ControlConstants object for chaining
         */
        public ControlConstants withProfile(double maxVel, double maxAcc) {
            this.profiled = true;
            this.maxVel = maxVel;
            this.maxAcc = maxAcc;
            return this;
        }

        /**
         * Sets whether the controller is set up to be profiled.
         * 
         * @param profiled whether the controller is profiled. Automatically set to true
         *                 when {@link #withProfile(double, double)} is called.
         * @return this ControlConstants object for chaining
         */
        public ControlConstants withProfiled(boolean profiled) {
            this.profiled = profiled;
            return this;
        }

        /**
         * Sets the position tolerance for the controller.
         * 
         * @param tolerance position tolerance
         * @return this ControlConstants object for chaining
         * 
         * @see #withTolerance(double, double)
         */
        public ControlConstants withTolerance(double tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        /**
         * Sets the tolerances for the controller.
         * 
         * @param tolerance    position tolerance
         * @param velTolerance velocity tolerance
         * @return this ControlConstants object for chaining
         */
        public ControlConstants withTolerance(double tolerance, double velTolerance) {
            this.tolerance = tolerance;
            this.velTolerance = velTolerance;

            return this;
        }

        /**
         * Sets the I-Zone (where to use integral control) for the controller.
         * 
         * @param iZone maximum error for which integral control is applied
         * @return this ControlConstants object for chaining
         */
        public ControlConstants withIZone(double iZone) {
            this.iZone = iZone;
            return this;
        }

        /**
         * Sets the integral range (limiting integral output) for the controller.
         * 
         * @param iMin minimum accumulated error
         * @param iMax maximum accumulated error
         * @return this ControlConstants object for chaining
         */
        public ControlConstants withIRange(double iMin, double iMax) {
            this.iMin = iMin;
            this.iMax = iMax;
            return this;
        }

        /**
         * Sets the period for the controller.
         * 
         * @param period period in seconds (default 0.02s)
         * @return this ControlConstants object for chaining
         */
        public ControlConstants withPeriod(double period) {
            this.period = period;
            return this;
        }

        /**
         * Sets the controller to be continuous over the given input range. Primarily
         * used for angular control. (e.g. 0 and 359 degrees should be considered
         * adjacent)
         * 
         * @param minInput minimum input value
         * @param maxInput maximum input value
         * @return this ControlConstants object for chaining
         */
        public ControlConstants withContinuous(double minInput, double maxInput) {
            this.isContinuous = true;
            this.minInput = minInput;
            this.maxInput = maxInput;
            return this;
        }

        /**
         * Creates a new {@link PIDController} with the configured constants.
         */
        public PIDController getPIDController() {
            PIDController controller = new PIDController(kP, kI, kD);
            controller.setTolerance(tolerance);
            controller.setIntegratorRange(iMin, iMax);
            controller.setIZone(iZone);

            return controller;
        }

        /**
         * Creates a new {@link ProfiledPIDController} with the configured constants.
         */
        public ProfiledPIDController getProfiledPIDController() {
            ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD,
                    new TrapezoidProfile.Constraints(maxVel, maxAcc));
            controller.setTolerance(tolerance);
            controller.setIntegratorRange(iMin, iMax);
            controller.setIZone(iZone);

            return controller;
        }

        /**
         * Creates a new {@link ElevatorFeedforward} with the configured constants.
         */
        public ElevatorFeedforward getElevatorFeedforward() {
            return new ElevatorFeedforward(kS, kG, kV, kA);
        }

        /**
         * Creates a new {@link SimpleMotorFeedforward} with the configured constants.
         */
        public SimpleMotorFeedforward getSimpleFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }
    }

    /**
     * Wrapper around a set of control constants that exposes them as
     * {@link LoggedTunableNumber}s for runtime tuning and logging.
     */
    public static class TunableControlConstants {
        // PID gains
        LoggedTunableNumber kP;
        LoggedTunableNumber kI;
        LoggedTunableNumber kD;
        LoggedTunableNumber tolerance;
        LoggedTunableNumber velTolerance;
        LoggedTunableNumber iZone;
        LoggedTunableNumber iMin;
        LoggedTunableNumber iMax;
        double period;

        // feedforward gains
        LoggedTunableNumber kV;
        LoggedTunableNumber kA;

        // physical gains
        LoggedTunableNumber kS;
        LoggedTunableNumber kG;

        // trapezoid profile
        boolean profiled;
        LoggedTunableNumber maxVel;
        LoggedTunableNumber maxAcc;

        // continuous control
        boolean isContinuous;
        double maxInput;
        double minInput;

        /**
         * Creates a new TunableControlConstants object, initializing the tunable
         * numbers with the values from the given {@link ControlConstants}.
         * 
         * @param key       base NetworkTables key for the {@link LoggedTunableNumber}s.
         *                  Should <i>not</i> end with a slash.
         * @param constants initial ControlConstants to copy values from
         */
        public TunableControlConstants(String key, ControlConstants constants) {
            this.kP = new LoggedTunableNumber(key + "/kP", constants.kP);
            this.kI = new LoggedTunableNumber(key + "/kI", constants.kI);
            this.kD = new LoggedTunableNumber(key + "/kD", constants.kD);
            this.tolerance = new LoggedTunableNumber(key + "/tolerance", constants.tolerance);
            this.velTolerance = new LoggedTunableNumber(key + "/velTolerance", constants.velTolerance);
            this.iZone = new LoggedTunableNumber(key + "/iZone", constants.iZone);
            this.iMax = new LoggedTunableNumber(key + "/maxIntegral", constants.iMax);
            this.iMin = new LoggedTunableNumber(key + "/minIntegral", constants.iMin);
            this.period = constants.period;
            this.kV = new LoggedTunableNumber(key + "/kV", constants.kV);
            this.kA = new LoggedTunableNumber(key + "/kA", constants.kA);
            this.kS = new LoggedTunableNumber(key + "/kS", constants.kS);
            this.kG = new LoggedTunableNumber(key + "/kG", constants.kG);
            this.profiled = constants.profiled;
            this.maxVel = new LoggedTunableNumber(key + "/maxVel", constants.maxVel);
            this.maxAcc = new LoggedTunableNumber(key + "/maxAcc", constants.maxAcc);
            this.isContinuous = constants.isContinuous;
            this.maxInput = constants.maxInput;
            this.minInput = constants.minInput;
        }

        /**
         * Returns an array of all {@link LoggedTunableNumber}s in this
         * TunableControlConstants.
         * 
         * @return array of all LoggedTunableNumbers in the order of: kP, kI, kD,
         *         tolerance,
         *         velTolerance, iZone, iMin, iMax, kV, kA, kS, kG, maxVel, maxAcc
         */
        public LoggedTunableNumber[] getAllTunableNumbers() {
            return new LoggedTunableNumber[] {
                    kP, kI, kD, tolerance, velTolerance, iZone, iMin, iMax, kV, kA, kS, kG, maxVel, maxAcc
            };
        }

        /**
         * Creates a new {@link PIDController} (not {@link TunablePIDController}) with
         * the configured constants.
         * 
         * @see {@link TunablePIDController} for creating a tunable PID controller.
         */
        public PIDController getPIDController() {
            PIDController controller = new PIDController(kP.get(), kI.get(), kD.get());
            controller.setTolerance(tolerance.get());
            controller.setIntegratorRange(iMin.get(), iMax.get());
            controller.setIZone(iZone.get());

            return controller;
        }

        /**
         * Creates a new {@link ProfiledPIDController} (not
         * {@link TunableProfiledController}) with the configured constants.
         * 
         * @see {@link TunableProfiledController} for creating a tunable profiled PID
         *      controller.
         */
        public ProfiledPIDController getProfiledPIDController() {
            ProfiledPIDController controller = new ProfiledPIDController(
                    kP.get(), kI.get(), kD.get(), new TrapezoidProfile.Constraints(maxVel.get(), maxAcc.get()));
            controller.setTolerance(tolerance.get());
            controller.setIntegratorRange(iMin.get(), iMax.get());
            controller.setIZone(iZone.get());

            return controller;
        }

        /**
         * Creates a new {@link ElevatorFeedforward} with the configured constants.
         * 
         * <p>
         * Note: This is not necessary when using a {@link TunableProfiledController}.
         */
        public ElevatorFeedforward getElevatorFeedforward() {
            return new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
        }

        /**
         * Creates a new {@link SimpleMotorFeedforward} with the configured constants.
         * 
         * <p>
         * Note: This is not necessary when using a {@link TunableProfiledController}.
         */
        public SimpleMotorFeedforward getSimpleFeedforward() {
            return new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
        }
    }

    /**
     * Wrapper around WPILib's {@link PIDController} that updates the controller
     * with runtime-tunable parameters provided by a
     * {@link TunableControlConstants}.
     * 
     * <p>
     * <i>Note: For motion profiling, use {@link TunableProfiledController}.</i>
     */
    public static class TunablePIDController {
        private final TunableControlConstants params;
        private final PIDController pidController;

        /**
         * Creates a new TunablePIDController with the given
         * {@link TunableControlConstants}.
         * 
         * @param tunableParams The {@link TunableControlConstants} to configure this
         *                      controller.
         */
        public TunablePIDController(TunableControlConstants tunableParams) {
            this.params = tunableParams;

            pidController = new PIDController(
                    tunableParams.kP.get(), tunableParams.kI.get(), tunableParams.kD.get(), tunableParams.period);

            pidController.setTolerance(tunableParams.tolerance.get(), tunableParams.velTolerance.get());

            if (tunableParams.isContinuous) {
                pidController.enableContinuousInput(tunableParams.minInput, tunableParams.maxInput);
            }
        }

        /**
         * Returns the {@link TunableControlConstants} used to configure this
         * controller.
         *
         * @return The {@link TunableControlConstants} of this controller.
         */
        public TunableControlConstants getParams() {
            return params;
        }

        /**
         * Updates the {@link PIDController}'s parameters from the
         * {@link TunableControlConstants}.
         */
        public void updateParams() {
            pidController.setP(params.kP.get());
            pidController.setI(params.kI.get());
            pidController.setD(params.kD.get());
            pidController.setTolerance(params.tolerance.get(), params.velTolerance.get());
            pidController.setIZone(params.iZone.get());
            pidController.setIntegratorRange(params.iMin.get(), params.iMax.get());
            pidController.setTolerance(params.tolerance.get(), params.velTolerance.get());
        }

        /**
         * Returns the accumulated error used in the integral calculation of this
         * controller.
         *
         * @return The accumulated error of this controller.
         * @see PIDController#getAccumulatedError()
         */
        public double getAccumulatedError() {
            return pidController.getAccumulatedError();
        }

        /**
         * Sets the goal for the PIDController.
         *
         * @param goal The desired goal.
         */
        public void setSetpoint(double goal) {
            pidController.setSetpoint(goal);
            updateParams();
        }

        /**
         * Returns the current goal of the PIDController.
         *
         * @return The current goal.
         * @see PIDController#getSetpoint()
         */
        public double getSetpoint() {
            return pidController.getSetpoint();
        }

        /**
         * @return Whether the error is within the acceptable bounds.
         * @see PIDController#atSetpoint()
         */
        public boolean atSetpoint() {
            return pidController.atSetpoint();
        }

        /**
         * Returns the difference between the goal and the measurement.
         *
         * @return The error.
         * @see PIDController#getError()
         */
        public double getPositionError() {
            return pidController.getError();
        }

        /**
         * @return The velocity error.
         * @see PIDController#getErrorDerivative()
         */
        public double getVelocityError() {
            return pidController.getErrorDerivative();
        }

        /**
         * Returns the next output of the PID controller.
         *
         * @param measurement The current measurement of the process variable.
         * @param goal        The new goal of the controller.
         * @return The next controller output.
         * 
         * @see PIDController#calculate(double, double)
         */
        public double calculate(double measurement, double goal) {
            return pidController.calculate(measurement, goal);
        }

        /**
         * Returns the next output of the PID controller.
         *
         * @param measurement The current measurement of the process variable.
         * @return The next controller output.
         * 
         * @see PIDController#calculate(double)
         */
        public double calculate(double measurement) {
            return pidController.calculate(measurement);
        }

        /**
         * Resets the previous error and the integral term.
         * 
         * @see PIDController#reset()
         */
        public void reset() {
            pidController.reset();
        }
    }

    /**
     * Wrapper around WPILib's {@link ProfiledPIDController} and that configures the
     * controller using runtime-tunable values provided by a
     * {@link TunableControlConstants}. It also includes feedforward control
     * (defined by kS, kV, kA, kG).
     * 
     * <p>
     * <i>Note: For non-motion-profiled control, use
     * {@link TunablePIDController}.</i>
     */
    public static class TunableProfiledController {
        private final ProfiledPIDController profiledPIDController;
        private final TunableControlConstants params;

        private double previousVelocity = 0;

        /**
         * Creates a new TunableProfiledController with the given
         * {@link TunableControlConstants}.
         * 
         * @param tunableParams The {@link TunableControlConstants} to configure this
         *                      controller.
         */
        public TunableProfiledController(TunableControlConstants tunableParams) {
            this.params = tunableParams;

            if (!tunableParams.profiled) {
                throw new IllegalArgumentException(
                        "TunableControlConstants must be profiled to use TunableProfiledController");
            }

            profiledPIDController = new ProfiledPIDController(
                    tunableParams.kP.get(),
                    tunableParams.kI.get(),
                    tunableParams.kD.get(),
                    new TrapezoidProfile.Constraints(tunableParams.maxVel.get(), tunableParams.maxAcc.get()),
                    tunableParams.period);

            profiledPIDController.setTolerance(tunableParams.tolerance.get(), tunableParams.velTolerance.get());

            if (tunableParams.isContinuous) {
                profiledPIDController.enableContinuousInput(tunableParams.minInput, tunableParams.maxInput);
            }
        }

        /**
         * Returns the {@link TunableControlConstants} used to configure this
         * controller.
         *
         * @return The {@link TunableControlConstants} of this controller.
         */
        public TunableControlConstants getParams() {
            return params;
        }

        /**
         * Updates the controller's parameters based on the current values in the
         * {@link TunableControlConstants}.
         */
        public void updateParams() {
            profiledPIDController.setP(params.kP.get());
            profiledPIDController.setI(params.kI.get());
            profiledPIDController.setD(params.kD.get());

            profiledPIDController.setConstraints(
                    new TrapezoidProfile.Constraints(params.maxVel.get(), params.maxAcc.get()));
            profiledPIDController.setIZone(params.iZone.get());
            profiledPIDController.setIntegratorRange(params.iMin.get(), params.iMax.get());
            profiledPIDController.setTolerance(params.tolerance.get(), params.velTolerance.get());
        }

        /**
         * Returns the accumulated error used in the integral calculation of this
         * controller.
         *
         * @return The accumulated error of this controller.
         * @see ProfiledPIDController#getAccumulatedError()
         */
        public double getAccumulatedError() {
            return profiledPIDController.getAccumulatedError();
        }

        /**
         * Sets the goal for the ProfiledPIDController.
         *
         * @param goal The desired goal.
         */
        public void setGoal(double goal) {
            profiledPIDController.setGoal(goal);
            previousVelocity = profiledPIDController.getSetpoint().velocity;
            updateParams();
        }

        /**
         * Returns the current goal of the ProfiledPIDController.
         *
         * @return The current goal.
         * @see ProfiledPIDController#getGoal()
         */
        public double getGoal() {
            return profiledPIDController.getGoal().position;
        }

        /**
         * Returns the current setpoint of the ProfiledPIDController.
         *
         * @return The current setpoint.
         * @see ProfiledPIDController#getSetpoint()
         */
        public State getSetpoint() {
            return profiledPIDController.getSetpoint();
        }

        /**
         * @return Whether the error is within the acceptable bounds.
         * @see ProfiledPIDController#atGoal()
         */
        public boolean atGoal() {
            return profiledPIDController.atGoal();
        }

        /**
         * Returns the difference between the goal and the measurement.
         *
         * @return The error.
         * @see ProfiledPIDController#getPositionError()
         */
        public double getPositionError() {
            return profiledPIDController.getPositionError();
        }

        /**
         * Returns the error derivative.
         *
         * @return The error derivative.
         * @see ProfiledPIDController#getVelocityError()
         */
        public double getVelocityError() {
            return profiledPIDController.getVelocityError();
        }

        /**
         * Calculates the feedforward output based on the current setpoint and kS, kG,
         * kV, and kA.
         *
         * @return The feedforward output.
         */
        public double calculateFeedforward() {
            State setpoint = profiledPIDController.getSetpoint();
            double accel = (setpoint.velocity - previousVelocity) / profiledPIDController.getPeriod();
            previousVelocity = setpoint.velocity;

            return params.kS.get() * Math.signum(setpoint.velocity)
                    + params.kG.get()
                    + params.kV.get() * setpoint.velocity
                    + params.kA.get() * accel;
        }

        /**
         * Returns the next output of the PID controller.
         *
         * @param measurement The current measurement of the process variable.
         * @param goal        The new goal of the controller.
         * @return The next controller output.
         * 
         * @see ProfiledPIDController#calculate(double, double)
         * @see #calculateFeedforward()
         */
        public double calculate(double measurement, double goal) {
            return profiledPIDController.calculate(measurement, goal) + calculateFeedforward();
        }

        /**
         * Returns the next output of the PID controller.
         *
         * @param measurement The current measurement of the process variable.
         * @return The next controller output.
         * 
         * @see ProfiledPIDController#calculate(double)
         * @see #calculateFeedforward()
         */
        public double calculate(double measurement) {
            return profiledPIDController.calculate(measurement) + calculateFeedforward();
        }

        /**
         * Resets the previous error and the integral term.
         * 
         * @see ProfiledPIDController#reset(double, double)
         */
        public void reset(double measuredPos, double measuredVel) {
            profiledPIDController.reset(measuredPos, measuredVel);
            previousVelocity = measuredVel;
        }

        /**
         * Resets the previous error and the integral term.
         * 
         * @see ProfiledPIDController#reset(double)
         */
        public void reset(double measuredPos) {
            reset(measuredPos, 0);
        }
    }

}
