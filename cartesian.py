import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
import numpy as np
import sys
import ikpy.chain
import os
from ament_index_python.packages import get_package_share_directory
from sympy import symbols, cos, sin, pi, simplify, pprint, tan, expand_trig, sqrt, trigsimp, atan2
from sympy.matrices import Matrix
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import time

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['6RJ1','6RJ2','6RJ3','6RJ4','6RJ5','6RJ6']

        self.robot_initialize()
        argv = sys.argv[1:]
        startxyz=(float(argv[0]),float(argv[1]),float(argv[2]))
        q1,q2,q3,q4,q5,q6 = self.get_angles(float(argv[0]),float(argv[1]),float(argv[2]),0,0,0)

        startjoint=[float(q1),float(q2),float(q3),float(q4),float(q5),float(q6)]
        print("first trajectory ",startjoint)
        self.goal_positions=list(startjoint)
        self.timer_callback()
        endxyz=(float(argv[3]),float(argv[4]),float(argv[5]))
        lq1,lq2,lq3,lq4,lq5,lq6=self.get_angles(float(argv[3]),float(argv[4]),float(argv[5]),0,0,0)
        print("last point ",lq1,lq2,lq3,lq4,lq5,lq6)
        timexyz=float(argv[6])
        shomarande=0
        while shomarande <= (timexyz):
            xpose=((float(argv[3])-float(argv[0]))/timexyz)*shomarande+float(argv[0])
            ypose=((float(argv[4])-float(argv[1]))/timexyz)*shomarande+float(argv[1])
            zpose=((float(argv[5])-float(argv[2]))/timexyz)*shomarande+float(argv[2])

            shomarande+=0.0015


            plq1,plq2,plq3,plq4,plq5,plq6=self.get_angles(float(xpose),float(ypose),float(zpose),0,0,0)
            nextpoint=[float(plq1),float(plq2),float(plq3),float(plq4),float(plq5),float(plq6)]
            print("next point trajectory  =", nextpoint)
            print ("Time =     ", shomarande)
            self.goal_positions=list(nextpoint)
            self.timer_callback()



     

    def timer_callback(self):
        
        bazu_trajectory_msg = JointTrajectory()
        bazu_trajectory_msg.joint_names = self.joints
        ## creating a point
        point = JointTrajectoryPoint()

        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=1)
        ## adding newly created point into trajectory message
        bazu_trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(bazu_trajectory_msg)
        print("\n Trajectory Sent !\n")
    
    def robot_initialize(self):
        urdf_file= ("/home/izan/my_ws/src/urdf_example/description/robotros2control.urdf.xacro")
        self.kuka_robot = ikpy.chain.Chain.from_urdf_file(urdf_file)

    def get_fk_solution(self):
        T=self.kuka_robot.forward_kinematics([0]*7)
        print("\n Transformation matrix \n" , T)
    







    def pose(self,theta, alpha, a, d):
        # returns the pose T of one joint frame i with respect to the previous joint frame (i - 1)
        # given the parameters:
        # theta: theta[i]
        # alpha: alpha[i-1]
        # a: a[i-1]
        # d: d[i]

        r11, r12 = cos(theta), -sin(theta)
        r23, r33 = -sin(alpha), cos(alpha)
        r21 = sin(theta) * cos(alpha)
        r22 = cos(theta) * cos(alpha)
        r31 = sin(theta) * sin(alpha)
        r32 = cos(theta) * sin(alpha)
        y = -d * sin(alpha)
        z = d * cos(alpha)
            
        T = Matrix([
            [r11, r12, 0.0, a],
            [r21, r22, r23, y],
            [r31, r32, r33, z],
            [0.0, 0.0, 0.0, 1]
        ])
        
        T = simplify(T)

        return T

    # under construction
    def forward_kin(self,q1,q2,q3,q4,q5,q6):
        X = []
        Y = []
        Z = []
        d90 = pi/2
        T01 = self.pose(q1, 0, 0, 0.75)
        T0g = T01
        px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
        X.append(px)
        Y.append(py)
        Z.append(pz)
        T12 = self.pose(q2 - d90, -d90, 0.35, 0)
        T0g = T0g* T12
        px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
        X.append(px)
        Y.append(py)
        Z.append(pz)
        T23 = self.pose(q3, 0, 1.25, 0)
        T0g = T0g* T23
        px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
        X.append(px)
        Y.append(py)
        Z.append(pz)
        T34 = self.pose(q4, -d90, -0.054, 1.5)
        T0g = T0g* T34
        px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
        X.append(px)
        Y.append(py)
        Z.append(pz)
        T45 = self.pose(q5, d90, 0, 0)
        T0g = T0g* T45
        px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
        X.append(px)
        Y.append(py)
        Z.append(pz)
        T56 = self.pose(q6, -d90, 0, 0)
        T0g = T0g* T56
        px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
        X.append(px)
        Y.append(py)
        Z.append(pz)
        T6g = self.pose(0, 0, 0, 0.303)
        #final position and rotation
        T0g = T0g* T6g
        px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
        X.append(px)
        Y.append(py)
        Z.append(pz)
        #fig = plt.figure()
        #ax = fig.add_subplot(111,projection = '3d')
        #ax.set_xlabel('x axis')
        #ax.set_ylabel('y axis')
        #ax.set_zlabel('z axis')
        
        X = np.reshape(X,(1,7))
        Y = np.reshape(Y,(1,7))
        Z = np.reshape(Z,(1,7))

        return X,Y,Z
        #ax.cla()
        #ax.plot_wireframe(X,Y,Z)
        #plt.draw()
        #plt.pause(3)
        #ax.cla()
        #ax.plot_wireframe(Z,Y,X,color='r')
        
        #plt.show()



    def create_plot(self):

        fig = plt.figure()
        ax = fig.add_subplot(111,projection = '3d')
        ax.set_xlabel('x axis')
        ax.set_ylabel('y axis')
        ax.set_zlabel('z axis')
        ax.set_xlim3d([0, 2])
        ax.set_ylim3d([0, 3])
        ax.set_zlim3d([0, 3])
        ax.set_autoscale_on(False)
        return fig,ax

    def update_plot(self,X,Y,Z,fig,ax):
        X = np.reshape(X,(1,7))
        Y = np.reshape(Y,(1,7))
        Z = np.reshape(Z,(1,7))
        ax.cla()
        ax.plot_wireframe(X,Y,Z)
        #plt.draw()
        ax.set_xlabel('x axis')
        ax.set_ylabel('y axis')
        ax.set_zlabel('z axis')
        ax.set_xlim3d([0, 2])
        ax.set_ylim3d([0, 3])
        ax.set_zlim3d([0, 3])
        ax.set_autoscale_on(False)
        fig.canvas.draw()
        fig.canvas.flush_events()
        #plt.pause(3)
        #ax.cla()
        #ax.plot_wireframe(Z,Y,X,color='r')


    #------------  Rotation Matrix and Euler Angles -----------
    # Calculates Rotation Matrix given euler angles.
    def eulerAnglesToRotationMatrix(self,theta) :
        
        R_x = np.array([[1,         0,                  0                   ],
                        [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                        [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                        ])
            
            
                        
        R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                        [0,                     1,      0                   ],
                        [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                        ])
                    
        R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                        [math.sin(theta[2]),    math.cos(theta[2]),     0],
                        [0,                     0,                      1]
                        ])
                        
                        
        R = np.dot(R_z, np.dot( R_y, R_x ))
    
        return R


    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self,R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(self,R) :
    
        assert(self.isRotationMatrix(R))
        
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        
        singular = sy < 1e-6
    
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
    
        return np.array([x, y, z])
#---------------------------------------


    def get_hypotenuse(self,a, b):
        # calculate the longest side given the two shorter sides 
        # of a right triangle using pythagorean theorem
        return sqrt(a*a + b*b)


    def get_cosine_law_angle(self,a, b, c):    
        # given all sides of a triangle a, b, c
        # calculate angle gamma between sides a and b using cosine law
        cos_gamma = (a*a + b*b - c*c) / (2*a*b)
        sin_gamma = sqrt(1 - cos_gamma * cos_gamma)
        gamma = atan2(sin_gamma, cos_gamma)

        return gamma


    def get_wrist_center(self,gripper_point, R0g, dg = 0.303):
        # get the coordinates of the wrist center wrt to the base frame (xw, yw, zw)
        # given the following info:
        # the coordinates of the gripper (end effector) (x, y, z)
        # the rotation of the gripper in gripper frame wrt to the base frame (R0u)
        # the distance between gripper and wrist center dg which is along common z axis
        # check WRITEUP.pdf for more info
        xu, yu, zu = gripper_point 
            
        nx, ny, nz = R0g[0, 2], R0g[1, 2], R0g[2, 2]
        xw = xu - dg * nx
        yw = yu - dg * ny
        zw = zu - dg * nz 

        return xw, yw, zw


    def get_first_three_angles(self,wrist_center):
        # given the wrist center which a tuple of 3 numbers x, y, z
        # (x, y, z) is the wrist center point wrt base frame
        # return the angles q1, q2, q3 for each respective joint
        # given geometry of the kuka kr210
        # check WRITEUP.pdf for more info
        x, y, z  = wrist_center
            
        a1, a2, a3 = 0.35, 1.25, -0.054
        d1, d4 = 0.75, 1.5
        l = 1.50097168527591 # get_hypotenuse(d4, abs(a3))
        phi = 1.53481186671284 # atan2(d4, abs(a3))
        
        x_prime = self.get_hypotenuse(x, y)
        mx = x_prime -  a1
        my = z - d1 
        m = self.get_hypotenuse(mx, my)
        alpha = atan2(my, mx)
        
        gamma = self.get_cosine_law_angle(l, a2, m)
        beta = self.get_cosine_law_angle(m, a2, l)
        
        q1 = atan2(y, x)
        q2 = pi/2 - beta - alpha 
        q3 = -(gamma - phi)
            
        return q1, q2, q3 


    def get_last_three_angles(self,R):
        #Recall that from our simplification, R36 (R) equals the following:
        #Matrix([
        #[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
        #[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
        #[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])
        #From trigonometry we can get q4, q5, q6 if we know numerical values of all cells of matrix R36 (R)
        #check WRITEUP.pdf for more info    
        sin_q4 = R[2, 2]
        cos_q4 =  -R[0, 2]
            
        sin_q5 = sqrt(R[0, 2]**2 + R[2, 2]**2) 
        cos_q5 = R[1, 2]
            
        sin_q6 = -R[1, 1]
        cos_q6 = R[1, 0] 
        
        q4 = atan2(sin_q4, cos_q4)
        q5 = atan2(sin_q5, cos_q5)
        q6 = atan2(sin_q6, cos_q6)
            
        return q4, q5, q6


    def get_angles(self,x, y, z, roll, pitch, yaw):
        # input: given position and orientation of the gripper_URDF wrt base frame
        # output: angles q1, q2, q3, q4, q5, q6
            
        gripper_point = x, y, z

        ################################################################################
        # All important symbolic transformations matrices are declared below 
        ################################################################################

        q1, q2, q3, q4, q5, q6 = symbols('q1:7')
        alpha, beta, gamma = symbols('alpha beta gamma', real = True)
        px, py, pz = symbols('px py pz', real = True)

        # Rotation of joint 3 wrt to the base frame interms the first three angles q1, q2, q3
        R03 = Matrix([
            [sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
            [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
            [        cos(q2 + q3),        -sin(q2 + q3),        0]])

        # Transpose of R03 
        R03T = Matrix([
            [sin(q2 + q3)*cos(q1), sin(q1)*sin(q2 + q3),  cos(q2 + q3)],
            [cos(q1)*cos(q2 + q3), sin(q1)*cos(q2 + q3), -sin(q2 + q3)],
            [            -sin(q1),              cos(q1),             0]])

        # Rotation of joint 6 wrt to frame of joint 3 interms of the last three angles q4, q5, q6
        R36 = Matrix([
            [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
            [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
            [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])

        # Rotation of urdf_gripper with respect to the base frame interms of alpha = yaw, beta = pitch, gamma = roll
        R0u = Matrix([
            [1.0*cos(alpha)*cos(beta), -1.0*sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha), 1.0*sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)],
            [1.0*sin(alpha)*cos(beta),  sin(alpha)*sin(beta)*sin(gamma) + 1.0*cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma) - 1.0*sin(gamma)*cos(alpha)],
            [          -1.0*sin(beta),                                     1.0*sin(gamma)*cos(beta),                                    1.0*cos(beta)*cos(gamma)]])

        # Total transform of gripper wrt to base frame given orientation yaw (alpha), pitch (beta), roll (beta) and position px, py, pz
        T0g_b = Matrix([
            [1.0*sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma),  1.0*sin(alpha)*cos(gamma) - 1.0*sin(beta)*sin(gamma)*cos(alpha), 1.0*cos(alpha)*cos(beta), px],
            [sin(alpha)*sin(beta)*cos(gamma) - 1.0*sin(gamma)*cos(alpha), -1.0*sin(alpha)*sin(beta)*sin(gamma) - 1.0*cos(alpha)*cos(gamma), 1.0*sin(alpha)*cos(beta), py],
            [                                   1.0*cos(beta)*cos(gamma),                                        -1.0*sin(gamma)*cos(beta),           -1.0*sin(beta), pz],
            [                                                          0,                                                                0,                        0,  1]])

        # Total transform of gripper wrt to base frame given angles q1, q2, q3, q4, q5, q6
        T0g_a = Matrix([
            [((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)],
            [ ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)],
            [                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                                  (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                                 -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
            [                                                                                                                                                            0,                                                                                                                                                             0,                                                                                        0,                                                                                                                                                                                                              1]])

        # Rotation of urdf_gripper wrt (DH) gripper frame from rotz(pi) * roty(-pi/2) and it's transpose
        Rgu_eval = Matrix([[0, 0, 1], [0, -1.00000000000000, 0], [1.00000000000000, 0, 0]])
        RguT_eval = Matrix([[0, 0, 1], [0, -1.00000000000000, 0], [1.00000000000000, 0, 0]])

        # Inverse kinematics transformations starts below

        R0u_eval = R0u.evalf(subs = {alpha: yaw, beta: pitch, gamma: roll})
        R0g_eval = R0u_eval * RguT_eval

        wrist_center = self.get_wrist_center(gripper_point, R0g_eval, dg = 0.303)

        j1, j2, j3 = self.get_first_three_angles(wrist_center)

        R03T_eval = R03T.evalf(subs = {q1: j1.evalf(), q2: j2.evalf(), q3: j3.evalf()})
        R36_eval = R03T_eval * R0g_eval

        j4, j5, j6 = self.get_last_three_angles(R36_eval)

        j1 = j1.evalf()
        j2 = j2.evalf()
        j3 = j3.evalf()
        j4 = j4.evalf()
        j5 = j5.evalf()
        j6 = j6.evalf()

        return j1, j2, j3, j4, j5, j6




















def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()