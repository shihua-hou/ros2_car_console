"""PID controller with anti-windup and output limiting.

Each instance controls a single axis (x, y, or yaw).
Visual servo uses three PIDController instances.
"""


class PIDController:
    """Proportional-Integral-Derivative controller with anti-windup.

    Usage:
        pid = PIDController(kp=0.8, ki=0.0, kd=0.05, integral_limit=0.5)
        output = pid.update(error=0.3, dt=0.05)
        pid.reset()  # clears integral accumulator and previous error
    """

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 integral_limit: float = 1.0, output_limit: float | None = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral_limit = integral_limit
        self._output_limit = output_limit

        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True

    def update(self, error: float, dt: float) -> float:
        """Compute the PID output for the given error and time step.

        Args:
            error: Current error (setpoint - measurement).
            dt: Time elapsed since previous call (seconds).

        Returns:
            PID output (clamped to output_limit if set).
        """
        if dt <= 0.0:
            return 0.0

        # Proportional
        p_out = self.kp * error

        # Integral with anti-windup and clamped accumulation
        self._integral += error * dt
        # Clamp integral to prevent windup
        if self._integral > self._integral_limit:
            self._integral = self._integral_limit
        elif self._integral < -self._integral_limit:
            self._integral = -self._integral_limit
        i_out = self.ki * self._integral

        # Derivative (on measurement, not setpoint — avoids derivative kick)
        if self._first_call:
            d_out = 0.0
            self._first_call = False
        else:
            d_error = (error - self._prev_error) / dt
            d_out = self.kd * d_error
        self._prev_error = error

        output = p_out + i_out + d_out

        # Output limiting
        if self._output_limit is not None:
            if output > self._output_limit:
                output = self._output_limit
            elif output < -self._output_limit:
                output = -self._output_limit

        return output

    def reset(self):
        """Clear integral accumulator and previous error."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True

    def set_gains(self, kp: float | None = None,
                  ki: float | None = None, kd: float | None = None):
        """Update gains at runtime. Pass None to keep current value."""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

    @property
    def integral(self) -> float:
        return self._integral
