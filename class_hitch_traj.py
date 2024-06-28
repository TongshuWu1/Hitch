import numpy as np

class hitch_traj:

    def _init_(self, pt, contact1, contact2, length_pt_R1, length_pt_R2):
        self.pt = pt
        self.v_c1_pt = pt - contact1
        self.v_c2_pt = pt - contact2
        self.l_c1_pt = np.linalg.norm(self.v_c1_pt)
        self.l_c2_pt = np.linalg.norm(self.v_c2_pt)

        self.length_pt_R1 = length_pt_R1
        self.length_pt_R2 = length_pt_R2

        self.r1, self.r2 = self.get_robot_position()

    

    def get_robot_position(self):

        #calculate unit vector
        unit_vector_c1_to_pt = self.v_c1_pt / self.l_c1_pt
        unit_vector_c2_to_pt = self.v_c2_pt / self.l_c2_pt

        r1 = self.pt + unit_vector_c1_to_pt * self.length_pt_R1
        r2 = self.pt + unit_vector_c2_to_pt * self.length_pt_R2



        return r1, r2

    def trajz_to_t(self, t, pt, r1, r2, a):
        # Calculate midpoint between r1 and r2
        midpoint = (r1 + r2) / 2

        # Axis of rotation is normalized vector from pt to midpoint
        axis = midpoint - pt
        axis = axis / np.linalg.norm(axis)

        # Calculate rotation angles in radians
        theta_r1 = 2 * np.pi * t
        theta_r2 = theta_r1 + 2*np.pi  # 180 degrees phase shift

        # Calculate rotation matrices for r1 and r2
        R_r1 = self.rotation_matrix(axis, a*theta_r1)
        R_r2 = self.rotation_matrix(axis, a*theta_r2)

        # Rotate points
        r1_rotated = np.dot(R_r1, r1 - midpoint) + midpoint
        r2_rotated = np.dot(R_r2, r2 - midpoint) + midpoint

        r1_rotated[2] = max(r1_rotated[2], 0.1)

        return r1_rotated, r2_rotated
    

    def rotation_matrix(self, axis, theta):
        """
        Return the rotation matrix associated with counterclockwise rotation about
        the given axis by theta radians using Rodrigues' formula.
        """
        axis = np.asarray(axis)
        axis = axis / np.sqrt(np.dot(axis, axis))
        a = np.cos(theta / 2.0)
        b, c, d = -axis * np.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                        [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                        [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])



