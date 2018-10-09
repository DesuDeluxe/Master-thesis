import pybullet as p
import time
import math
import pybullet_data

from multiprocessing import Pipe
from time import sleep
import sys
import platform
#import win32pipe, win32file
import logging

#logging.basicConfig(level=logging.DEBUG)
#logging.basicConfig(level=logging.INFO)

logger = logging.getLogger(__name__)


#MASS = 100
MASS = 110
#FRICTION = 9999990
FRICTION = 0.5
#GRAVITY = -90.8
GRAVITY = -20
GAIN = 0.02#0.009#0.01

STATE1 = [-45,90,90,  45,90,90,    -45,90,90,  45,90,90,          -45,90,90,  45,90,90]
STATE1 = []
class Simulation(object):
    def __init__(self, child_conn, gait_steps, gui = None):
#        super(Simulation,self).__init__()

        self.cubeStartPos = None
        self.cont_legs = [3, 7, 11, 15, 19, 23]
        self.robot = None
        self.joints, self.joints_seg = self.gen_jointLists()
        self.child_conn = child_conn

        self.gait_steps = gait_steps
        self.gui = gui

        self.child_conn.send(self.joints_seg)
        self.setup()
        self.run_sim()

    def reset(self):
        p.resetSimulation()
        #p.disconnect()

    def gen_jointLists(self):
        joints = []
        y=0
        for x in range(1,24):
            if y== 3:
                y=-1
            else:
                joints.append(x)
            y+=1
        #print(joints)
        joints_seg = []
        tmp = []
        y=1
        for x in joints:
            tmp.append(x)
            if y== 3:
                joints_seg.append(tmp)
                tmp = []
                y=0
            y+=1
        return joints, joints_seg
        #print(self.joints_seg)

    def setup(self):
        if self.gui == 'gui':
            physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        else:
            physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        p.resetDebugVisualizerCamera(cameraDistance = 2, cameraYaw = 50, cameraPitch = -30, cameraTargetPosition= [0.0,0.0,0.0])
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING,1)
        p.setPhysicsEngineParameter(enableConeFriction=0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

        planeId = p.loadURDF('plane.urdf')

        self.cubeStartPos = [0,0,0.15]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

        system = platform.system()
        if system == 'Windows':
            self.robot = p.loadURDF(r'E:\projekty\spider\pybullet\hexx\urdf\hexx.urdf',self.cubeStartPos, cubeStartOrientation)
        elif system == 'Linux':
            self.robot = p.loadURDF(r'/run/media/desu/n1/projekty/spider/pybullet/hexx/urdf/hexx.urdf',self.cubeStartPos, cubeStartOrientation)
        p.setRealTimeSimulation(0)
        #p.setTimeStep(0.0001)
        p.setGravity(0, 0, GRAVITY)
        #self.joints, self.joints_seg = self.gen_jointLists()
        #p.changeDynamics(self.robot, -1, mass=MASS*10, lateralFriction=FRICTION, spinningFriction=FRICTION, rollingFriction=FRICTION)
        p.changeDynamics(self.robot, -1, mass=MASS*10)
        for x in self.joints:
            #p.changeDynamics(self.robot, x, mass=MASS, lateralFriction=FRICTION, spinningFriction=FRICTION, rollingFriction=FRICTION, restitution=0.0000001, contactStiffness=1000, contactDamping=10000000)
            #p.changeDynamics(self.robot, x, mass=MASS, lateralFriction=FRICTION, spinningFriction=FRICTION, rollingFriction=FRICTION, restitution=0.0000001, contactStiffness=100, contactDamping=100,frictionAnchor=1)
            #p.changeDynamics(self.robot, x, mass=MASS, lateralFriction=FRICTION, spinningFriction=FRICTION, rollingFriction=FRICTION)
            p.changeDynamics(self.robot, x,  mass=MASS)
        #self.set_angl([0], 0)
        #self.set_angl([1,2], 90)
        #self.set_angles(STATE1)

    def stop(self):
        p.disconnect()

    def reset(self):
        p.resetBasePositionAndOrientation(self.robot,self.cubeStartPos, p.getQuaternionFromEuler([0,0,0]))
        self.set_angl([0], 0)
        self.set_angl([1,2], 90)
        for x in range(500):
            self.sim_step()

    def get_contactData(self):
        linkList = []
        contactList = p.getContactPoints(self.robot)
        for x in contactList:
            if x[4] == -1 and x[3] not in linkList:
                linkList.append(x[3])
        linkList2 = []
        for x in self.cont_legs:
            if x in linkList:
                linkList2.append(1)
            else:
                linkList2.append(0)
        return linkList2


    def get_baseData(self):
        angles = []
        pos = []
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robot)
        e = p.getEulerFromQuaternion(cubeOrn)
        for x in e:
            angles.append(int(math.degrees(round(x,2))))

        for x in cubePos:
            pos.append(round(x,3))

        return pos, angles
    #print(cubePos,cubeOrn)

    def get_requiredData(self):
        contact = self.get_contactData()
        base_pos, base_angle = self.get_baseData()
        #[contact, base_pos, base_angle] -> [[0, 0, 0, 0, 0, 0], [-0.0, 0.0, 0.2], [0, 0, 0]]
        return contact, base_pos, base_angle


    def get_anglesLists(self):
        angles = []
        angl = p.getJointStates(self.robot, self.joints)
        for x in angl:
            angles.append(round(x[0],2))
        '''
        angles_seg = []
        tmp = []
        y=1
        for x in angles:
            tmp.append(x)
            if y == 3:
                angles_seg.append(tmp)
                tmp = []
                y=0
            y+=1
        '''
        return angles#, angles_seg


    '''
    def set_angles(self, angles):
        y=1
        for x in range(0, len(angles)):
            print(angles)
            print(x)
            p.setJointMotorControl2(bodyUniqueId = self.robot, jointIndex = x, controlMode = p.POSITION_CONTROL, targetPosition = math.radians(-angles[x]),positionGain=GAIN)# force=100000)
            if y == 3:
                p.setJointMotorControl2(bodyUniqueId = self.robot, jointIndex = x, controlMode = p.POSITION_CONTROL, targetPosition = math.radians(angles[x]),positionGain=GAIN)# force=100000)
                y=0
            y+=1
    '''
    def set_angles(self, angles):
        x=-1
        for idx, leg in enumerate(self.joints_seg):
            for idy, part in enumerate(leg):
                x+=1
                try:
                    if angles[x] == 's':
                        continue
                    if idx % 2 == 0:
                        p.setJointMotorControl2(bodyUniqueId = self.robot, jointIndex = part, controlMode = p.POSITION_CONTROL, targetPosition = math.radians(-angles[x]),positionGain=GAIN)
                    else:
                        p.setJointMotorControl2(bodyUniqueId = self.robot, jointIndex = part, controlMode = p.POSITION_CONTROL, targetPosition = math.radians(angles[x]),positionGain=GAIN)
                except:
                    print('angles size:', len(angles))
                    print('x: ', x)
                    sys.exit(1)

    def set_angl(self, part, angle):
        for x in part:
            for idx, c in enumerate(self.joints_seg):
                if idx % 2 == 0:
                    p.setJointMotorControl2(bodyUniqueId = self.robot, jointIndex = c[x], controlMode = p.POSITION_CONTROL, targetPosition = math.radians(-angle))
                else:
                    p.setJointMotorControl2(bodyUniqueId = self.robot, jointIndex = c[x], controlMode = p.POSITION_CONTROL, targetPosition = math.radians(angle))


    def changeCam(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robot)
        a = p.getDebugVisualizerCamera()
        cyaw = a[8]
        cpitch = a[9]
        cdist =  a[10]
        cubePos = a[11]
        keys = p.getKeyboardEvents()
        #Keys to change camera
        if keys.get(100):  #D
            cyaw+=0.1
        if keys.get(97):   #A
            cyaw-=0.1
        if keys.get(99):   #C
            cpitch+=0.1
        if keys.get(102):  #F
            cpitch-=0.1
        if keys.get(122):  #Z
            cdist+=.01
        if keys.get(120):  #X
            cdist-=.01
        p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=cubePos)

    def sim_step(self):
        p.stepSimulation()
        #angles = self.get_anglesLists()
        #contact = self.get_contactData()
        #base_pos, base_angle = self.get_baseData()

        #out = [angles, cont, base_pos, base_angle]
        #out = [contact, base_pos, base_angle] #[[0, 0, 0, 0, 0, 0], [-0.0, 0.0, 0.2], [0, 0, 0]]

        #for x in b[0]:
        #    print(int(math.degrees(x)))
        #print(get_baseData())
        #print(get_contactData())
        #return angles, cont, base_pos, base_angle#18 angles(each joint), 6 legs contact with ground info, xyz base position, xyz euler angles = 30
        #return out
        #return contact, base_pos, base_angle
    def run_sim(self):
        x=0
        a=-1
        ang = []
        data = []
        data_s = []
        while True:
            p.stepSimulation()
            if self.gui == 'gui':
                self.changeCam()
            if x == 600:
                if ang:

                    logger.debug('S ang')
                    #sys.stdout.flush()


                    #print(ang[0])
                    self.set_angles(ang[0])
                    del ang[0]
                    a-=1
                    x=0
                    data_s.append(self.get_requiredData())
                    continue
                if a == 0:
                    a=-1
                    self.child_conn.send("simok")

                    logger.debug('S simok sent')
                    #sys.stdout.flush()

                    #sleep(0.1)
                    data_s.append(self.get_requiredData())
                    self.child_conn.send(data_s[1:])
                    data_s = []
                    while True:
                        try:
                            rec = self.child_conn.recv()
                            if rec == 'dataok':
                                break
                            else:
                                #child_conn.send(individual)
                                pass
                        except Exception as e:
                            a=0
                    x=0
                try:
                    data = self.child_conn.recv()
                except:
                    continue
                if len(data) == self.gait_steps:

                    logger.debug('S gait')
                    #sys.stdout.flush()

                    ang = data
                    a=self.gait_steps
                        #sleep(0.5)
                    #self.child_conn.send("simok")
                    #break
                    x=0
                elif data == 'reset':
                    self.reset()
                    self.child_conn.send("resok")

                    logger.debug('S resetok sent')
                    #sys.stdout.flush()

                    x=550
                    #print("next step")
                    #sys.stdout.flush()
                    #break
                else:
                    #self.child_conn.send("wrongdata")
                    x=0
                #sleep(0.1)
            x+=1
            sleep(0.001)
