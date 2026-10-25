import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import os

D_MIN = 0.5      # m
SLACK_L1 = 1e3   # exact-penalty weight on the lower slack. Project-derived:

def _R_from_quat_np(q):
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qw*qz),      2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),       1 - 2*(qx**2 + qz**2),  2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),       2*(qy*qz + qw*qx),      1 - 2*(qx**2 + qy**2)],
    ])

class NMPC:

    def __init__(self, N=20, dt=0.05, q_pixel=0.0, q_pixel_vel=0.0,
                 mass=1.25, g=9.81, Ixx=0.01, Iyy=0.01, Izz=0.02,
                 l=0.125, k_tau=20.0, k_mu=None, camera=None,export_dir=None,
                 verbose=False):
        """
        Every drone parameter defaults to the NOVA airframe, so existing
        callers see no change. Pass them to fly a different quadrotor, or use
        from_mujoco_model() to read them straight out of an XML.
        """
        # NMPC parameters  {}
        self.N = int(N)
        self.dt = float(dt)
        self.Tf = self.N * self.dt
        # Drone params 
        self.mass = mass        # kg   <inertial mass="...">][]
        self.g = g           
        self.Ixx, self.Iyy, self.Izz = Ixx, Iyy, Izz   # <inertial diaginertia="...">
        self.l = l             
        self.k_tau = k_tau      
        if k_mu is None:
            self.k_mu = 0.022 * self.k_tau
        else:
            self.k_mu = k_mu
        # --- Dimensions ---
        self.nx = 13 
        self.nu = 4 
        self._p_verified = False
        self.camera  = camera
        self.verbose = verbose
        if export_dir is None:
            export_dir = os.path.join(
                os.path.expanduser("~"), ".cache", "quad_nmpc")
        self.export_dir = os.path.abspath(export_dir)
        os.makedirs(self.export_dir, exist_ok=True)
        if camera is not None: 
            self.q_pixel = q_pixel
            self.q_pixel_vel = q_pixel_vel
            self.T_BC = camera['t_bc']
            self.FX = camera['fx']
            self.FY= camera['fy']
            self.Q_BC = camera['q_bc']
            # R_BC is a CONSTANT, so it must be exactly orthonormal. Q_BC as written in
            # perception_feed.py is unit only to ~1e-8; the quat→matrix formula is quadratic,
            # so an unnormalised constant gives a non-orthonormal R_BC and a ~1e-8 bias in s.
            # Caught by the oracle fuzz. Normalise once, here.
            self._R_BC = _R_from_quat_np(np.asarray(self.Q_BC) / np.linalg.norm(self.Q_BC))

        # ---- Build the OCP  Optimal Control Problem (OCP)----
        ocp = AcadosOcp()
        ocp.model = self._build_model()
        ocp.solver_options.N_horizon = self.N

        self.build_cost(ocp)

        self.build_constraints(ocp)

        self.set_solver_options(ocp)

        # ---- Build the solver ----
        ocp.parameter_values = np.zeros(3)
        ocp.code_export_directory = os.path.join(
            self.export_dir, "c_generated_code")
        self.solver = AcadosOcpSolver(
            ocp, json_file=os.path.join(self.export_dir, "quad_nmpc_ocp.json"))

        if self.verbose:
            try:
                print(f"[NMPC] np (parameter dim) = {self.solver.acados_ocp.dims.np}")
            except AttributeError:
                try:
                    print(f"[NMPC] np (parameter dim) = {ocp.dims.np}")
                except AttributeError:
                    print("[NMPC] np unreadable — verify via c_generated_code timestamp")
        u_init = np.full(self.nu, self.u_hover) # np.full(4, 0.619) makes a 4-vector [0.619, 0.619, 0.619, 0.619] — all four motors at hover throttle. This is the input we'll use to prime( initialize guess of) every node in the input trajectory.
        for k in range(self.N + 1): 
            self.solver.set(k, "x", self.x0) # This loops over all N + 1 nodes (k = 0, 1, ..., 20) and sets the state at each one to our placeholder x0 (origin, level, stationary). The result: the solver starts thinking "the drone is at the origin and stays there for the entire horizon."
        for k in range(self.N):
            self.solver.set(k, "u", u_init)
    

    def build_cost(self,ocp):   #   Define cost matrices Q (State), R (Input), Q_o (orbiting)
        """Fill ocp.cost. Also sets self.yref, self.yref_e, self.u_hover."""
        Q = np.diag([
            450.0, 450.0, 400.0,   
            10.0,  10.0,  10.0,   
            50.0,  50.0,  50.0, 50.0,
            1.0,   1.0,   1.0,    ])
        R = np.diag([1.0, 1.0, 1.0, 1.0]) 
        Q_o  = 20.0

        Q_e = Q.copy() # Q_e is the terminal cost
        Q_e[0:3, 0:3] *= 2.0 # pos at terminal is far more important so scaled ||ly vel 
        Q_e[3:6, 3:6] *= 1.5

        if self.camera is not None:
            PIXEL_NORM = 320.0   # IMG_W / 2 — half image width in pixels
            _eff_q_pixel     = self.q_pixel     / (PIXEL_NORM ** 2)
            _eff_q_pixel_vel = self.q_pixel_vel / (PIXEL_NORM ** 2)
            Q_p   = np.diag([_eff_q_pixel, _eff_q_pixel, _eff_q_pixel_vel, _eff_q_pixel_vel])
            Q_p_e = Q_p.copy()   # Q_e != Q in this file, so Q_p_e is a FREE parameter --

            ny     = self.nx + self.nu + 1 + 4   # 13 + 4 + 1(orbit) + 4(z) = 22
            ny_e   = self.nx + 4                 # 13 + 4(z)                = 17
            _i_orb = self.nx + self.nu           # 17 — orbiting slot
            _i_z   = self.nx + self.nu + 1       # 18 — start of z = [s, sdot]

        else:
            # No camera: the 4-element z = [s, sdot] block is absent from both
            # cost vectors, so there is no _i_z to index.
            ny     = self.nx + self.nu + 1    # 13 + 4 + 1(orbit)        = 18
            ny_e   = self.nx                  # 13                       = 13
            _i_orb = self.nx + self.nu        # 17 — orbiting slot

        ocp.cost.cost_type   = 'NONLINEAR_LS' # we use non linear least squares to do orbiting cost. 
        ocp.cost.cost_type_e = 'NONLINEAR_LS'

        # --- Define the Total Weight matrix W and W_e in acados OCP 
        W = np.zeros((ny, ny))
        state = slice(0, self.nx)                    # y[0:13]
        motor = slice(self.nx, self.nx + self.nu)    # y[13:17]
        W[state, state] = Q
        W[motor, motor] = R
        W[_i_orb, _i_orb] = Q_o                      # y[17], the orbit term

        if self.camera is not None:
            W[_i_z:_i_z + 4, _i_z:_i_z + 4] = Q_p     # Delta 2

        ocp.cost.W = W

        W_e = np.zeros((ny_e, ny_e))                                      # Delta 2: was Q_e (13x13)
        W_e[:self.nx, :self.nx] = Q_e
        if self.camera is not None:
            W_e[self.nx:self.nx + 4, self.nx:self.nx + 4] = Q_p_e
        ocp.cost.W_e = W_e
        if self.verbose:
            print("W   diag:", np.diag(W))
            print("W_e diag:", np.diag(W_e))

        u_hover = np.sqrt(self.mass * self.g / (4 * self.k_tau)) # yref is the target vector NMPC wants, NMPC will minimize |y - yref|².
        yref = np.zeros(ny)
        yref_e = np.zeros(ny_e)
        yref[0:3] = [0.0, 0.0, 1.0]  # Hover at 1 M, init val ||ly no rotation (0,0,0,1) 
        yref_e[0:3] = [0.0, 0.0, 1.0]
        yref[6]   = yref_e[6]   = 1.0              
        yref[13:17] = u_hover # we do this so hover thrust is not penalized

        if self.camera is not None:
            yref[_i_z:_i_z + 4]         = 0.0
            yref_e[self.nx:self.nx + 4] = 0.0

        ocp.cost.yref = yref
        ocp.cost.yref_e = yref_e   
        self.yref = yref 
        self.yref_e = yref_e
        self.u_hover = u_hover 


    def set_solver_options(self,ocp):
        ocp.solver_options.tf = self.Tf # tf is the terminal time of the prediction horizon, in seconds.
        ocp.solver_options.integrator_type        = 'ERK'
        ocp.solver_options.nlp_solver_type        = 'SQP_RTI'
        ocp.solver_options.qp_solver              = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx         = 'GAUSS_NEWTON'
        ocp.solver_options.sim_method_num_stages  = 4 # 4-stage Runge-Kutta, i.e. RK4. The classical 4th-order method. Per-step truncation error is O(h⁵).
        ocp.solver_options.sim_method_num_steps   = 1 # This is the number of integration steps acados takes to advance the dynamics from one node to the next. one integration step per shooting interval.
        ocp.solver_options.qp_solver_iter_max     = 50 # Max HPIPM iterations (usually converges in 10–20); hitting 50 means a bug, not a reason to raise it.
        ocp.solver_options.print_level            = 0 # 0 = silent. Set to 1 for debugging (dumps QP matrices every tick — will flood terminal).

    def build_constraints(self,ocp):
        # Input constraints 
        ocp.constraints.lbu   = np.zeros(self.nu) # lbu (lower bound vector for u). Four zeros: [0, 0, 0, 0]. Each motor's minimum command.
        ocp.constraints.ubu   = np.ones(self.nu) # ubu (upper bound vector for u). Four ones: [1, 1, 1, 1]. Each motor's maximum command.
        ocp.constraints.idxbu = np.arange(self.nu) # idx-index b-bound u-input i.e Indices of Bounded Inputs.If needed to apply specific bounds.
    
        # State constraints
        lbx   = np.array([-20, -20, -0.4, -10, -10, -10, -10, -10, -10]) # Postion bounds are how far drone can be from target. Dsafe within this.  # absolute drone velocity limit same in target and drone frame 
        ubx   = np.array([ 20,  20,  20,  10,  10,  10,  10,  10,  10])
        idxbx = np.array([0, 1, 2, 3, 4, 5, 10, 11, 12]) # indices of the state vector that we want to apply bounds on 10,11,12 are for angular velocities
        ocp.constraints.lbx,   ocp.constraints.ubx,   ocp.constraints.idxbx   = lbx, ubx, idxbx
        ocp.constraints.lbx_e, ocp.constraints.ubx_e, ocp.constraints.idxbx_e = lbx, ubx, idxbx # _e suffix means — terminal bounds. 

        if self.camera is not None:
            ocp.model.con_h_expr  = self._d_expr          # scalar, set in _build_model
            ocp.constraints.lh    = np.array([D_MIN])
            ocp.constraints.uh    = np.array([1e6])       # effectively +inf
            ocp.constraints.idxsh = np.array([0])         # slack the one h-constraint
            ocp.cost.zl = np.array([SLACK_L1])            # L1 on lower slack
            ocp.cost.Zl = np.array([0.0])                 # pure L1 -- add small L2 if QP chatters
            ocp.cost.zu = np.array([0.0])                 # upper slack unused (uh = +inf)
            ocp.cost.Zu = np.array([0.0])

        # initial state placeholder
        x0 = np.zeros(self.nx) # x_0 — the state at node 0 of the prediction horizon.
        x0[6] = 1.0#  sets the single element at index 6 to 1.0, which corresponds to q_w,the scalar part of the quaternion. This means (0,0,0,1) quaternion represents no rotation 
        ocp.constraints.x0 = x0 
        self.x0 = x0

    @classmethod
    def from_mujoco_model(cls,model):
        import mujoco
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
        mass = model.body_mass[bid]
        inertia = model.body_inertia[bid]
        k_tau = model.actuator_ctrlrange[0,1]
        k_mu = abs( model.actuator_gear[0,5])*k_tau
        sites = np.array([model.site_pos[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thrust{i}")]
            for i in range(4)])
        l = np.abs(sites[:, 1]).mean()
        return cls(mass=mass, Ixx=inertia[0], Iyy=inertia[1], Izz=inertia[2],
                   l=l, k_tau=k_tau, k_mu=k_mu)

    def perception_output_map(self,p, v, q, omega):
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        R_TB = ca.vertcat(                       # identical algebra to R_WB below
            ca.horzcat(1 - 2*(qy**2 + qz**2),  2*(qx*qy - qw*qz),      2*(qx*qz + qw*qy)),
            ca.horzcat(2*(qx*qy + qw*qz),      1 - 2*(qx**2 + qz**2),  2*(qy*qz - qw*qx)),
            ca.horzcat(2*(qx*qz - qw*qy),      2*(qy*qz + qw*qx),      1 - 2*(qx**2 + qy**2)),
        )
        R_bc, t_bc = ca.DM(self._R_BC), ca.DM(self.T_BC)
        Cp  = -R_bc.T @ (R_TB.T @ p + t_bc)                     # P1  target in C
        Om  =  R_bc.T @ omega                                   # P2  camera ang. rate
        vC  =  R_bc.T @ (R_TB.T @ v + ca.cross(omega, t_bc))    # P3  camera lin. vel wrt T
        Cpd = -ca.cross(Om, Cp) - vC                            # P4  transport theorem

        X, Y, d    = Cp[0],  Cp[1],  Cp[2]                      # P5  pinhole + quotient rule
        dX, dY, dd = Cpd[0], Cpd[1], Cpd[2]
        s    = ca.vertcat(self.FX * X / d,  self.FY * Y / d)
        sdot = ca.vertcat(self.FX * (dX*d - X*dd) / d**2,
                        self.FY * (dY*d - Y*dd) / d**2)
        return s, sdot, d

    def set_target_trajectory(self, positions, velocities, yaws):
            for k in range(self.N + 1):
                qw, qz = np.cos(yaws[k] / 2), np.sin(yaws[k] / 2) 
                if k < self.N: # if we are not at the last node
                    ref = self.yref.copy()       
                    ref[0:3]   = positions[k]
                    ref[3:6]   = velocities[k]
                    ref[6], ref[9] = qw, qz
                    self.solver.set(k, "yref", ref) # Updating Referece fro solver node K but here each node gets different reference cause now we predicty the traget. Each Node in NMPC horizon has a estimate now.
                else: # for the last node we use yref_e
                    ref_e = self.yref_e.copy()    # 13-vector, no input slot
                    ref_e[0:3] = positions[k]
                    ref_e[3:6] = velocities[k]
                    ref_e[6], ref_e[9] = qw, qz
                    self.solver.set(self.N, "yref", ref_e)   # ← "yref", not "yref_e" i.e we set the reference for the last node 

    def compute_action(self, x_meas, a_tgt): # x_meas is the currently measured state of the drone, a 13-vector laid out exactly like our NMPC state:
        if a_tgt is None:
            a_arr = np.zeros((self.N + 1, 3))
        else:
            a_arr = np.asarray(a_tgt, dtype=float)
            if a_arr.ndim == 1:                       # single vector -> hold across horizon
                a_arr = np.tile(a_arr, (self.N + 1, 1))
        for k in range(self.N + 1):
            self.solver.set(k, "p", a_arr[k])
        if self.verbose and a_tgt is not None and not self._p_verified:
            try:
                print(f"[MPC] p readback n0={self.solver.get(0,'p')} nN={self.solver.get(self.N,'p')}")
            except Exception as e:
                print(f"[MPC] p readback unsupported ({e}) — use the delta_u probe")
            self._p_verified = True
        self.solver.set(0, "lbx", x_meas) # we set up lbx_0 = ubx_0 = x0 to fix the initial state via a degenerate box constraint. Now we overwrite those bounds with the actual measured state — same trick, different value
        self.solver.set(0, "ubx", x_meas) # SAME TRICK HERE i.e we fix the initial state
        status = self.solver.solve() # SOLVE THE OPTIMIZATION PROBLEM
        if status != 0: # If the solver returned a non-zero status, print an error message
            print(f"[NMPC] solver returned status {status}")
        u = self.solver.get(0, "u") # get the optimal input for the first node ie 0th node (Remember we planned for N+1 nodes but we only apply the first node's input), This is the receding horizon principle at the heart of MPC. The solver always plans a full 1-second trajectory, but only the first control is actually applied. 
        # this is the actual command we send to the drone. 
        return np.clip(u, 0.0, 1.0) # is the cheap insurance policy: explicitly squash any tiny violations into [0, 1]. One CPU cycle, no surprises downstream.

    def _build_model(self):
        '''
         # why seperate function for building the model ?
         # Because we need to uilding the dynamics model is conceptually distinct from "configure the OCP." Keeping _build_model isolated means later, if you want to swap dynamics (e.g. add motor lag, drag, or switch to NOVA's target-frame formulation in Stage 2), you only touch this method. The leading underscore is Python convention for "internal helper, don't call from outside the class."
        '''
        p     = ca.SX.sym('p', 3)       # position, TARGET frame (target-translated, world-axis-aligned).
                                        # Drone position relative to target — target sits at origin by definition.
                                        # p = drone_pos_W − target_pos_W, computed by the caller before compute_action.
        v     = ca.SX.sym('v', 3)       # linear velocity, TARGET frame. v = drone_vel_W − target_vel_W.
                                        # Under NOVA's assumption a_tgt_T ≈ 0, the time derivative of p_T is v_T.
                                        # If target accelerates significantly, we may need to add an a_tgt_T
                                        # feedforward (option B); for now we absorb it as disturbance.
        q     = ca.SX.sym('q', 4)       # attitude quaternion, scalar-first. Rotates body → target frame.
                                        # NUMERICALLY equal to body → world because target axes = world axes.
                                        # We use q_TB notation to match NOVA paper, but the underlying matrix
                                        # is the same R_WB we always had.
        omega = ca.SX.sym('omega', 3)   # angular velocity, body frame. Body-relative → totally frame-agnostic.
        a_tgt = ca.SX.sym('a_tgt', 3)   # target acceleration, world/target axes (m/s^2)
        x     = ca.vertcat(p, v, q, omega) # this is our state vector, it contains all the information about the drone that we need to track, position, velocity, attitude and angular velocity
        # stacks the four pieces into one 13×1 symbolic vector. From here on, x is the thing the ODE will be a function of: xdot = f(x, u).
        
        u    = ca.SX.sym('u', 4) # symbolic input vector, u = [u1,u2,u3,u4].
        # why dimensionless ?? -> because we are scaling the thrusts by the mass and gravity so we don't need to feed the actual thrust values to the controller  because we can use teh setup for any drone by just changing the mass and gravity
        u_sq = u * u # element-wise square. In CasADi, * between two SX vectors of the same length is the Hadamard product (component-wise), not a dot product.
        # u_sq = [u0², u1², u2², u3²] as a 4-vector

        f    = self.k_tau * u_sq    # per-motor thrust in Newtons. k_tau is a scalar, u_sq is a 4-vector, so f is also a 4-vector. At u = 1 you get f = 8 N per motor 
        T_total     = f[0] + f[1] + f[2] + f[3] # scalar. Just adds the four motor thrusts. This is the total lift the drone produces, in Newtons.
        thrust_body = ca.vertcat(0, 0, T_total) #  3D vector in the body frame. All thrust points along body-z (props push straight up out of their plane). This is (0, 0, T_total) only in body coords — in the world frame it points wherever body-z is currently aimed, which depends on the quaternion. 

        tau = ca.vertcat( # torque = force × lever arm × sin(angle between them)for our drone angle is 90 degrees so sin(90) = 1 so we just multiply the force * lever arm
             self.l * ( f[0] + f[1] - f[2] - f[3]),  #roll
             self.l * (-f[0] + f[1] + f[2] - f[3]),  #pitch
            self.k_mu * (u_sq[0] - u_sq[1] + u_sq[2] - u_sq[3]), #yaw
        )  # this is the torque vector in body coordinates # ca.vertcat is CasADi's function for stacking scalars or vectors into a single column vector
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        R_WB = ca.vertcat(#This builds the 3×3 rotation matrix R_WB from the attitude quaternion q
            ca.horzcat(1 - 2*(qy**2 + qz**2),  2*(qx*qy - qw*qz),  2*(qx*qz + qw*qy)), # So we're literally assembling the 3×3 matrix one row at a time, top to bottom.
            ca.horzcat(2*(qx*qy + qw*qz),  1 - 2*(qx**2 + qz**2),  2*(qy*qz - qw*qx)),
            ca.horzcat(2*(qx*qz - qw*qy),  2*(qy*qz + qw*qx),  1 - 2*(qx**2 + qy**2)),#Eg. these lines it converts roll-pitch tilt into a lateral force component in world Y. If this line had a sign error, your drone would lurch left when commanded right and the NMPC would never converge.
        )

        v_y_body = ca.mtimes(R_WB.T, v)[1] # Get teh velicities y component of the droen in its own body frame To avoid orbioting, v is actually in the target frame ie (vy — how fast the drone is moving in Y relative to the target)(not world frame), but since the target frame axes are aligned with world axes, they're numerically identical 
        g_vec = ca.vertcat(0, 0, -self.g) # gravity in TARGET frame. Numerically identical to world frame
        dp = v # the position derivative is the velocity
        dv = (R_WB @ thrust_body) / self.mass + g_vec - a_tgt   # drone/mass (Force in world frame/mass plus gravity vector in world frame). Newton's 2nd law. he forces acting on the drone are exactly two: thrust and gravity.
        #  a = ΣF/m simplified above equation
        # F = thrust_world = R_WB @ thrust_body
        # The @ operator is matrix multiplication in CasADi/NumPy (3×3 matrix times 3-vector → 3-vector).
        dq = 0.5 * ca.vertcat( # thsi basically if I know the current orientation and the current angular velocity , what is my new orientation, here orientation is represented using quaternoin  that why * by 0.5 as a quaternion encodes half the rotation angle
            -qx*omega[0] - qy*omega[1] - qz*omega[2], # q̇_w = −0.5(q_x ω_x + q_y ω_y + q_z ω_z) = −0.5(q_w_dot)
             qw*omega[0] + qy*omega[2] - qz*omega[1], # q̇_x = 0.5 (q_w ω_x + q_y ω_z − q_z ω_y)
             qw*omega[1] + qz*omega[0] - qx*omega[2], # q̇_y = 0.5 (q_w ω_y + q_z ω_x − q_x ω_z)
             qw*omega[2] + qx*omega[1] - qy*omega[0], # q̇_z = 0.5 (q_w ω_z + q_x ω_y − q_y ω_x)
        )

        J     = ca.diag(ca.vertcat(self.Ixx, self.Iyy, self.Izz)) # this is  3x3 matrix 
        J_inv = ca.diag(ca.vertcat(1.0/self.Ixx, 1.0/self.Iyy, 1.0/self.Izz))
        domega = J_inv @ (tau - ca.cross(omega, J @ omega))
        # ca.cross(omega, J @ omega) — CasADi's cross product. 
        # @ means matrix multiplication in CasADi/NumPy (3×3 matrix times 3-vector → 3-vector)  
        # domega means angular acceleration
        xdot = ca.SX.sym('xdot', self.nx) # we declare xdot as a fresh symbol that represents "the derivative of the state.
        f_expl = ca.vertcat(dp, dv, dq, domega) # stack the four derivative pieces into a single 13-vector. This is the explicit ODE:
        #f_expl is purely a function of the state`. The right-hand side has no reference to xdot itself — that's what makes it "explicit."

        if self.camera is not None:
            s_cam, sdot_cam, d_cam = self.perception_output_map(p, v, q, omega)
            z_cam = ca.vertcat(s_cam, sdot_cam)      # (4,)
            self._d_expr = d_cam                     # Delta 3: Eq. 30f constraint expression

        model = AcadosModel() # Creates an empty container.

        if self.camera is None:
            model.cost_y_expr   = ca.vertcat(x, u, v_y_body)  
            model.cost_y_expr_e = ca.vertcat(x)      

        else:     
            model.cost_y_expr   = ca.vertcat(x, u, v_y_body, z_cam)   # [x(13), u(4), v_y(1), z(4)] = 22
            model.cost_y_expr_e = ca.vertcat(x, z_cam)                # [x(13), z(4)]               = 17

        model.f_expl_expr = f_expl #give acados the explicit ODE ẋ = f(x, u). This is what RK4 uses to step the state forward.
        model.f_impl_expr = xdot - f_expl # Same dynamics rewritten as 0 = xdot - f(x, u). Required by acados for compatibility even though we use ERK (which doesn't need it).

        # The following lines tell acados which variables are which, in preparation for C code generation.
        model.x = x # The symbolic state, the 13-vector we built up from p, v, q, omega
        model.xdot = xdot #The symbolic derivative, the 13-vector of time derivatives (dp, dv, dq, domega).
        model.u = u # the symbolic input, our 4-vector of normalized motor commands
        model.p = a_tgt
        model.name = 'quad_13state_drone' # string label. acados uses it as a prefix for codegen artifacts: it generates C source files like nova_quad_13state_dyn_ode_expl_f_fun.c etc., compiles them into a shared library, and packages everything into a JSON. The name shows up in the generated file paths, so make it descriptive and avoid spaces/special characters.
        return model # return the AcadosModel object, which now contains all the symbolic dynamics and metadata that Acados needs to generate C code.handes it to init
        # we have now built teh entire dynamics of droen symbolically . next acados uses this model object to what we wnat teh droen to do ie cost and constraints. 
        