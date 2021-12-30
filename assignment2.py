import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options                     = {}
options['FIG_SIZE']         = [8,8]
options['OBSTACLES']        = False
options['FULL_RECALCULATE'] = True

#initial state {x y psi v}
state_i         = np.array([[0,0,np.pi/2,0]])
#state_i         = np.array([[0,0,0,0]])

class ModelPredictiveControl:
    def __init__(self):
        self.horizon    = 20
        self.dt         = 0.1

        # Reference or set point the controller will achieve.
        self.reference1 = [2, 0, 3.14/2]
        #self.reference1 = [10, 10, 3.14/2]
        self.reference2 = None #

    def plant_model(self,prev_state, dt, pedal, steering):
        psi_t   = prev_state[2]
        v_t     = prev_state[3]

        #position updating
        x_t_1     = prev_state[0] + dt*v_t*np.cos(psi_t)
        y_t_1     = prev_state[1] + dt*v_t*np.sin(psi_t)     

        #speed updating
        a_t     = pedal
        v_dot_t = a_t
        v_t_1   = v_t + v_dot_t*dt - v_t/25.0

        #heading angle updating
        L           = 2.5 #m, wheelbase
        beta_t      = steering #rad     
        psi_dot_t   = v_t*np.tan(beta_t)/L
        psi_t_1     = psi_t + dt*psi_dot_t

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self, u, *args):
        state       = args[0]
        ref         = args[1]
        cost        = 0.0

        prev_steering = u[1]

        for k in range(0, self.horizon):
            v_start = state[3]
            state = self.plant_model(state, self.dt, u[k*2], u[k*2 + 1])
                
            #position cost
            #square of euclidian-distance
            err_pos = (ref[0] - state[0])**2 + (ref[1] - state[1])**2

            #angular cost
            err_psi = abs(ref[2] - state[2])

            #longi accel cost
            accel_cost = abs(state[3] - v_start)**2

            #angular speed cost
            steering_cost = abs(u[k*2 + 1] - prev_steering)**2

            #normalization of cost is by-passed
            cost    += err_pos
            cost    += err_psi
            cost    += accel_cost
            cost    += steering_cost

            prev_steering = u[k*2 + 1]

        return cost

sim_run(options, ModelPredictiveControl, state_i)
