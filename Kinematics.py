import numpy as np
import math
from sympy import pprint

class ArmKinematics:
    def __init__(self, L1 = 4.5, L2 = 9.3, L3 = 8.7, L5 = 6.0):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L5 = L5
        self.a1 = np.deg2rad(90)
        self.a4 = np.deg2rad(90)
    
# -----------------------------
# Rotation helpers
# -----------------------------
    def Xrot(self, a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    
    def Yrot(self, a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    
    def Zrot(self, a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c,-s,0],[s,c,0],[0,0,1]])

# -----------------------------
# DH transform
# -----------------------------
    def Transform_func(self, theta, alpha, a, d):
        ct, st, ca, sa = [np.round(v, 10) for v in (np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha))]
        return np.array([
            [ ct ,-st*ca , st*sa , a*ct],
            [ st , ct*ca ,-ct*sa , a*st],
            [ 0  ,  sa   ,  ca   ,  d  ],
            [ 0  ,   0   ,   0   ,  1  ]
        ])
    
# -----------------------------
# Forward Kinematics
# -----------------------------   
    def forward_kinematics(self, t1, t2, t3, t4, t5):
        H1 = self.Transform_func(t1, self.a1, 0, self.L1)
        H2 = self.Transform_func(t2, 0, self.L2, 0)
        H3 = self.Transform_func(t3, 0, self.L3, 0)
        H4 = self.Transform_func(t4, self.a4, 0, 0)
        H5 = self.Transform_func(t5, 0, 0, self.L5)

        pprint(np.rad2deg(t1))
        pprint(np.rad2deg(t2))
        pprint(np.rad2deg(t3))
        pprint(np.rad2deg(t4))
        pprint(np.rad2deg(t5))
    
        T0 = np.eye(4)
        T1 = H1
        T2 = H1 @ H2
        T3 = H1 @ H2 @ H3
        T4 = H1 @ H2 @ H3 @ H4
        T5 = H1 @ H2 @ H3 @ H4 @ H5 

        return[T0, T1, T2, T3, T4, T5]

    @staticmethod
    def normalize_angle(a):
        """Normalize to [-pi, pi]"""
        return (a + math.pi) % (2*math.pi) - math.pi

    def extract_t4_t5_analytic_from_T03(self, T0_3, T_target):
        """
        Analytically extract t4,t5 so that H4(t4)*H5(t5) == T36 (where T36 = inv(T0_3)*T_target)
        This uses the same DH convention as your Transform_func (H4 has alpha=a4, H5 has d=L5).
        Returns (t4, t5) in radians, normalized.
        """
        T36 = np.linalg.inv(T0_3) @ T_target
        R36 = T36[:3,:3]

        # From the algebra for your DH frames:
        # R36[2,0] = sin(t5)
        # R36[2,1] = cos(t5)
        # R36[0,2] = sin(t4)
        # R36[1,2] = -cos(t4)
        # So:
        t5 = math.atan2(R36[2,0], R36[2,1])
        t4 = math.atan2(R36[0,2], -R36[1,2])

        return self.normalize_angle(t4), self.normalize_angle(t5)
    
    @staticmethod
    def approach_quality(q, fk_fnc):
        """
        Lower = better. Measures alignment of EE z-axis with world downward vector.
        q: tuple/list (t1,t2,t3,t4,t5) in radians
        Uses forward_kinematics to get end-effector frame.
        """
        T = fk_fnc(q[0], q[1], q[2], q[3], q[4])[-1]
        z = T[:3,2]                # end-effector z-axis
        down = np.array([0.0, 0.0, -1.0])
        # dot in [-1,1] ; when dot==1 -> perfect top-down (z aligned with -Z)
        dot = np.dot(z, down)
        # Convert to score where smaller is better: 1 - dot (so 0 is ideal)
        return 1.0 - float(dot)

    # -----------------------------
    # Revised inverse_kinematics
    # -----------------------------
    def inverse_kinematics(self, T_target, elbow='down', return_both=False, prefer='top'):
        """
        Corrected IK for DH convention, with analytic wrist extraction and approach preference.
        Inputs:
        - T_target : 4x4 numpy array target homogeneous transform
        - elbow : 'down' or 'up' chooses which elbow branch when returning a single solution
        - return_both : if True returns (sol_down, sol_up)
        - prefer : 'top' (default) tries to choose solution approaching from above; fallback is side then bottom
        Returns:
        - (t1,t2,t3,t4,t5) in radians for the selected solution, OR tuple(sol_down, sol_up) if return_both=True
        Notes:
        - Uses global L1, L2, L3, L5, a4 and forward_kinematics/Transform_func in your script.
        """
        R = T_target[:3, :3]
        px, py, pz = T_target[:3, 3]

        # wrist center Pw = P - L5 * z_axis_of_end_effector
        z_axis = R[:, 2]
        Pw = np.array([px, py, pz]) - self.L5 * z_axis

        # theta1
        t1 = math.atan2(Pw[1], Pw[0])
        c1 = math.cos(t1); s1 = math.sin(t1)

        # planar coordinates for 2-link solve (subtract shoulder height L1)
        K1 = Pw[0] * c1 + Pw[1] * s1
        K2 = Pw[2] - self.L1

        # Law of cosines for theta3
        D = (K1**2 + K2**2 - self.L2**2 - self.L3**2) / (2.0 * self.L2 * self.L3)
        # if D outside [-1,1] target is out of reach (clip to boundary)
        if abs(D) > 1.0 + 1e-9:
            # unreachable: clip D so math.sqrt works but warn (still produce boundary solution)
            D = np.clip(D, -1.0, 1.0)

        # two possible solutions for theta3 (Instead of theta3 = arcos(D), Using pythagorean)
        t3_down = math.atan2( math.sqrt(max(0.0, 1.0 - D**2)), D )    # Elbow down soln
        t3_up   = math.atan2(-math.sqrt(max(0.0, 1.0 - D**2)), D )    # Elbow up Soln

        # corresponding theta2 values (standard two-link solution)
        def theta2_from_theta3(th3):
            return math.atan2(K2, K1) - math.atan2(self.L3 * math.sin(th3), self.L2 + self.L3 * math.cos(th3))

        t2_down = theta2_from_theta3(t3_down)
        t2_up   = theta2_from_theta3(t3_up)

        # analytically compute wrist angles t4,t5 using T0_3
        def extract_t4_t5_for_branch(t1, t2, t3):
            # build T0_3 using your forward_kinematics (t4,t5 set to 0)
            T0_3 = self.forward_kinematics(t1, t2, t3, 0.0, 0.0)[3]
            return self.extract_t4_t5_analytic_from_T03(T0_3, T_target)

        t4_d, t5_d = extract_t4_t5_for_branch(t1, t2_down, t3_down)
        t4_u, t5_u = extract_t4_t5_for_branch(t1, t2_up,   t3_up)

        sol_down = (self.normalize_angle(t1), self.normalize_angle(t2_down), self.normalize_angle(t3_down),
                    self.normalize_angle(t4_d), self.normalize_angle(t5_d))
        sol_up   = (self.normalize_angle(t1), self.normalize_angle(t2_up),   self.normalize_angle(t3_up),
                    self.normalize_angle(t4_u), self.normalize_angle(t5_u))

        if return_both:
            # Order them by preference if prefer == 'top'
            if prefer == 'top':
                scored = [(self.approach_quality(sol_down, self.forward_kinematics), sol_down), (self.approach_quality(sol_up, self.forward_kinematics), sol_up)]
                scored.sort(key=lambda x: x[0])  # best (smallest) first
                return scored[0][1], scored[1][1]
            else:
                return sol_down, sol_up

        # If user requested a particular elbow branch return it (respect desired elbow)
        if elbow == 'down':
            chosen = sol_down
        elif elbow == 'up':
            chosen = sol_up
        else:
            # choose best by approach (top preference)
            if prefer == 'top':
                score_down = self.approach_quality(sol_down, self.forward_kinematics)
                score_up   = self.approach_quality(sol_up, self.forward_kinematics)
                chosen = sol_down if score_down <= score_up else sol_up
            else:
                chosen = sol_down  # default fallback

        return chosen
