import time

class PID:
    """ A PID control loop
    :param Pv: the proportional constant
    :param Iv: the integral constant
    :param Dv: the derivative constant
    :keyword bias: add/remove this much form the output
    :keyword iter_time: the amount of time between each calculation"""

    def __init__(self, Pv, Iv, Dv, **kwargs):
        self.P = Pv
        self.I = Iv
        self.D = Dv
        self.bias = 0
        self.iteration_time = 0.05

        if 'bias' in kwargs.keys():
            self.bias = kwargs['bias']

        if 'iter_time' in kwargs.keys():
            self.iteration_time = kwargs['iter_time']

        self.target = 0
        self.position = 0

        self.error_prior = 0
        self.integral_prior = 0

        self.max_power = 1
        self.min_power = -1

        self.power_out = 0
        self.enabled = True

        self.last_time = time.time()

        self.on_target_power_error = 1
        self.on_target_pos_error = 1

    def update_position(self, new_position):
        self.position = new_position

    def set_target(self, new_target):
        self.target = new_target

    def main_loop(self):
        current_time = time.time()
        if self.enabled and current_time >= self.last_time + self.iteration_time:
            time_change = current_time-self.last_time
            error = self.target - self.position

            integral = self.integral_prior + (error * time_change)

            derivative = (error - self.error_prior) / time_change

            output = self.P * error + self.I * integral + self.D * derivative + self.bias
            self.power_out = max(self.min_power, min(output, self.max_power))
            self.error_prior = error
            self.integral_prior = integral

            self.last_time = current_time

    def get_power(self):
        return self.power_out

    def set_P(self, new_val):
        self.P = new_val

    def set_I(self, new_val):
        self.I = new_val

    def set_D(self, new_val):
        self.D = new_val

    def set_max_power(self, new_max):
        self.max_power = new_max

    def set_min_power(self, new_min):
        self.min_power = new_min

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False
        self.error_prior = 0
        self.integral_prior = 0

    def on_target(self):
        if (self.target - self.on_target_pos_error < self.position < self.target + self.on_target_pos_error
                and -self.on_target_power_error < self.power_out < self.on_target_power_error):
            return True
        else:
            return False

    def set_on_target_error(self, position, power):
        self.on_target_power_error = power
        self.on_target_pos_error = position

    def reset_int(self):
        self.integral_prior = 0
        self.error_prior = 0
