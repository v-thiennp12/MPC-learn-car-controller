import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options                     = {}
options['FIG_SIZE']         = [8,8]
options['FULL_RECALCULATE'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon    = 20
        self.dt         = 0.2

        # Reference or set point the controller will achieve.
        self.reference = [30, 0, 0]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        v_t = prev_state[3] # m/s

        a_t     = pedal
        v_dot_t = a_t

        x_t_1 = x_t + v_t*dt
        v_t_1 = v_t + v_dot_t*dt - v_t/25.0

        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self, u, *args):
        state   = args[0]
        ref     = args[1]
        cost    = 0.0
        for k in range(0, self.horizon):
            #v_start = state[3]
            state = self.plant_model(state, self.dt, u[k*2], u[k*2 + 1])
                
            #position cost
            #cost  += (state[0] - ref[0])**2
            error = ref[0] -state[0]
            if error < 0:
                cost  += 100*abs(error)
            else:
                cost  += abs(error)

            #speed limit
            speed_kph = state[3]*3.6
            k_speed   = 1
            if (speed_kph > 10.0):
                #cost += -abs(speed_kph - 10)*k_speed
                cost += 100
        
        return cost

sim_run(options, ModelPredictiveControl)
