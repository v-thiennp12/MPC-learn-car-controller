import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE']         = [8,8]
options['OBSTACLES']        = True
options['FULL_RECALCULATE'] = False

#initial state {x y psi v}
state_i         = np.array([[5,0,np.pi/2,0]])

# 10000 obs , 20 horizon , obs 3 2, state 5 0 pi/2 0, dest 10

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt      = 0.1

        # Reference or set point the controller will achieve.
        self.reference1 = [7.5, 0, 3.14/2]
        #self.reference2 = None

        self.x_obs = 3
        self.y_obs = 1

    def plant_model(self, prev_state, dt, pedal, steering):
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

    def cost_function(self,u, *args):
        state       = args[0]
        ref         = args[1]
        cost        = 0.0

        prev_steering = u[1]

        for k in range(0, self.horizon):
            v_start = state[3]
            state = self.plant_model(state, self.dt, u[k*2], u[k*2 + 1])
                
            #position cost
            #square of euclidian-distance
            err_pos = 0*((ref[0] - state[0])**2 + (ref[1] - state[1])**2)

            err_pos_y = 0
            if (abs(ref[1] - state[1]) > 1):
                err_pos_y =  50*((ref[1] - state[1])**2)

            elif (abs(ref[0] - state[0]) < 0.25):
                err_pos_y =  50*((ref[1] - state[1])**2)

            err_pos_x = 100*((ref[0] - state[0])**2)

            #angular cost
            if (abs(ref[2] - state[2]) > np.pi/16) :
                err_psi = 100*((ref[2] - state[2])**2)
            else:
                err_psi = 200*((ref[2] - state[2])**2)

            #longi accel cost
            accel_cost = abs(state[3] - v_start)**2

            #angular speed cost
            steering_cost = abs(u[k*2 + 1] - prev_steering)**2

            #obstacle
            tol_zero = 1e-8
            if ((self.x_obs - state[0])**2 + (self.y_obs - state[1])**2 < tol_zero):
                obstacle_cost = 1000
            else:
                obstacle_cost = 100*1/((self.x_obs - state[0]) + (self.y_obs - state[1]))
                if  (abs(ref[0] - state[0]) < 0.5):
                    obstacle_cost = 0
                #obstacle_cost = 10*1/((self.x_obs - state[0])**2)

            #normalization of cost is by-passed
            cost    += err_pos
            cost    += err_psi
            cost    += err_pos_x
            cost    += err_pos_y
            #cost    += accel_cost
            #cost    += steering_cost
            cost    += obstacle_cost

            prev_steering = u[k*2 + 1]

        return cost

sim_run(options, ModelPredictiveControl, state_i)
