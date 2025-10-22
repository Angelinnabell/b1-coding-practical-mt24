class PDController:
    def __init__(self, Kp: float = 0.15, Kd: float = 0.6):
        """
        Initialize PD controller with gains.
        Args:
            Kp: Proportional gain (default: 0.15)
            Kd: Derivative gain (default: 0.6)
        """
        self.Kp = Kp
        self.Kd = Kd
        self.previous_error = 0.0
    
    def compute(self, reference: float, measurement: float) -> float:
        """
        Compute control action using PD control law.
        Args:
            reference: Target depth r[t]
            measurement: Current depth y[t]
        Returns:
            float: Control action u[t]
        """
        # Calculate current error
        error = reference - measurement
        
        # Calculate control action
        u = self.Kp * error + self.Kd * (error - self.previous_error)
        
        # Store error for next iteration
        self.previous_error = error
        
        return u