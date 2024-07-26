import numpy as np
from scipy import optimize


class CatenaryTrajectory:

    """
    Initialize the trajectory calculator for two robots at the ends of a catenary rope.
    :param dst_xCd: Array of float, the 3D center coordinates where the lowest point of the rope is located.
    :param yaw: Float, the rotation angle around the z-axis, affecting the orientation of the rope.
    :param rope_length: Float, the total length of the rope.
    """
    def __init__(self, dst_xCd, rope_length):
        self.dst_xCd = np.array(dst_xCd, dtype='f')  
        self.rope_length = rope_length

    # function catenary
    @staticmethod
    def f(c,l,x_bar):
        return (- l/2 + c * np.sinh(x_bar/(2*c)))  # only one real root at x = 1
    
    @staticmethod
    # Rotation matrix
    def Rotz(th):
        Rot = np.array([[np.cos(th), -np.sin(th), 0],
                    [np.sin(th), np.cos(th), 0],
                    [0, 0, 1]])
        return Rot
    
    # @staticmethod
    # # Hat_map
    # def hat_map(aux):
    #     A = np.array([[0, -aux[2], aux[1]], [aux[2], 0, -aux[0]], [-aux[1], aux[0], 0]])
    #     return A

    # function for compute the position of the robots
    # here the minimum point is fixed and we are rotating
    # and increasing and decreasing the span.


    def trajectory(self, t, dst_xCd, yaw, span):
        """
            Calculate the trajectory of the robots based on the span of the rope and orientation.
        :param t: Float, the time variable (currently unused but can be used for dynamic simulations).
        :param span: Float, the horizontal distance between the two robots.
        """

        self.dst_xCd = np.array(dst_xCd, dtype='f')  
        ## span desired
        dst_x_bar_d = span ############# --> this is s


        # minimum point fixed

        # dst_xCd = np.array([1., 0., 0.4], dtype='f') ############# --> this is Xc
        # dst_dxCd = np.array([0., 0., 0.], dtype='f')
        # dst_ddxCd = np.array([0., 0., 0.], dtype='f')

        # rotating with the time
        dst_yaw = yaw ############# --> this is yaw for the lowest point
        # dst_Omega = np.array([0, 0, 0], dtype='f')

        dst_R = self.Rotz(dst_yaw)
        # dst_dR = dst_R.dot(hat_map(dst_Omega))

        

        # dst_dx_bar_d = 0


        # solution using bisection
        sol = optimize.bisect(self.f, 0.01, 5, args=(self.rope_length, dst_x_bar_d))  ############# --> this is parameters in args l and s


        
        #(f, x0=[0.15], args=(1.5, dst_x_bar_d), method='hybr', tol=1e-6) 
        #print(sol)
        dst_c = np.abs(sol)
        # dst_dc = 0

        # equation for compute the slag
        dst_zABd = dst_c*(np.cosh(dst_x_bar_d/(2*dst_c)) - 1)

        # relative positions
        xAdv = np.array([-dst_x_bar_d/2, 0, dst_zABd], dtype='f')
        xBdv = np.array([dst_x_bar_d/2, 0, dst_zABd], dtype='f')

        ## Desired positions to vectors

        dst_xAd = self.dst_xCd + dst_R.dot(xAdv)
        dst_xBd = self.dst_xCd + dst_R.dot(xBdv)

    # dst_dxAd = dst_dxCd + dst_dR.dot(xAdv) + dst_R.dot(dxAdv)
    # dst_dxBd = dst_dxCd + dst_dR.dot(xBdv) + dst_R.dot(dxBdv)

        return dst_xAd, dst_xBd, dst_yaw
    
