#Luca Lotito 
#Class that handles the calculations for the simulation, holds everything together
import pygame
import numpy as np
from scipy.integrate import ode
import helpers

debugMessages = False

#Setup. Simulation only handles one plane at a time
class Plane(pygame.sprite.Sprite):
    def __init__(self, screen_height, imgfile):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load(imgfile)
        self.image = pygame.transform.smoothscale(self.image, (100, 36.8)) 
        self.rect = self.image.get_rect()
        self.screen_height = screen_height
        self.dt = 0.03
        self.g = -9.8
        self.p = 1.204
        self.A = 0.02
        self.scale = 5
        self.length = 5
        self.paused = True 
        
    def setup(self, pos, speed, initAOA):
        self.angle = initAOA
        self.mass = 0.005
        #Point masses for the plane
        self.m = [(0.005/12)*6, 0, (0.005/12)*3, (0.005/12)*1.5, (0.005/12)*1.5]
        #Locations of point masses 
        self.r  = [[1,1,-1,1,1],[5,0,0,0,0],[0,0,0,2,-2]]

        self.com = np.sum(self.r * np.tile(self.m, (3,1)), axis=1) / self.mass
        self.curCom = self.com #Used for debugging
        self.rp = self.r - np.tile(self.com , (5,1)).T 
        #Heavy Approximation with this value
        self.cop = 0.10
        #Sets up rotation
        rot = helpers.setup_rotation(initAOA)
        #Finds Inertia Tensor of the plane 
        eVal, eVec = helpers.findInertiaWorld(self.rp, self.mass)
        #rot  =eVec

        self.Ibody = np.diag(eVal)  # inertia tensor
        self.IbodyInv = np.linalg.inv(self.Ibody) 

        self.state = np.zeros(19)

        self.state[0:3] = pos 
        self.state[3:12] = rot.reshape([1,9])

        #Setting up inital Linear Momenteum
        self.state[12] = (np.cos(np.deg2rad(initAOA))*speed)*self.mass 
        self.state[13] = (np.sin(np.deg2rad(initAOA))*speed)*self.mass

        self.force = np.zeros(3)   

        self.torque = np.zeros(3)

        self.solver = ode(self.f)
        self.solver.set_integrator('dop853')
        self.solver.set_f_params(self.IbodyInv)
        self.solver.set_initial_value(self.state, 0)
        self.image_rot = self.image
        

    def step(self):
        #Gets the current rotation of the body
        self.r, self.curCom = helpers.getFrameOfRef(self.state[3:12].reshape([3,3]), self.rp, self.com, self.m, self.state[0:3].reshape([3]))
        self.angle, _l = self.get_angle_2d()
        #Finds the forces acting on the plane
        lifty = self.calculateLift(self.angle, self.state[12]/self.mass)
        dragx = self.calculateDrag(self.angle,self.state[12]/self.mass)
        #Aligns calculations on movement in the y axis
        liftx = self.calculateLift(-self.angle,self.state[13]/self.mass)
        dragy = self.calculateDrag(90-self.angle,self.state[13]/self.mass)
        self.cop = helpers.calculateCOP(self.angle)
        #Forces acting in each direction
        self.force[0] = liftx - dragx*(helpers.getSign(self.state[12]))*(1/40)
        self.force[1] = lifty - dragy*(helpers.getSign(self.state[13]))#Trying to get the direction of motion. negative, the drag should be applied in the opposite direction

        self.findTorque([self.force[0], lifty, 0])
        if self.solver.successful():
            self.solver.integrate(self.solver.t + self.dt)
        
        self.state = self.solver.y 
        self.t = self.solver.t 
        if debugMessages:
            self.debug()

    def f(self, t, state, IbodyInv):
        #Position
        rate = np.zeros(19)
        rate[0:3] = state[12:15]/self.mass
        
        #Rotation
        rate[3:12] = state[3:12] 
        _R = state[3:12].reshape([3,3])
        _R = helpers.orthonormalize(_R)
        Iinv = np.dot(_R, np.dot(IbodyInv, _R.T))
        _L = state[15:18]
        omega = np.dot(Iinv, _L)

        rate[3:12] = np.dot(helpers.star(omega), _R).reshape([1,9])
        #Linear Momentum
        rate[12] = self.force[0] 
        rate[13] = self.force[1] + self.g*self.mass
        rate[14] = self.force[2]
        rate[15:18] = self.torque

        return rate

    def calculateLift(self, angle, v):
        return helpers.liftCoefficent(angle)*(self.p*(v**2))/2 * self.A

    def calculateDrag(self, angle, v):
        return helpers.dragCoefficent(angle)*(self.p*(v**2))/2 * self.A
    #For informational purposes
    def getSpeed(self):
        return np.sqrt((self.state[12]/self.mass)**2+(self.state[13]/self.mass)**2)
    
    def findTorque(self, f):
        xpos = (self.r[0,0]-self.r[0,1])*self.cop
        ypos = (self.r[1,0]-self.r[1,1])*self.cop
        pos = [self.r[0,0]-xpos ,self.r[1,0] - ypos, 0]
        d = pos-((self.com-pos)*f)*f - self.com
        self.torque = np.cross(d, f)

    def rotate(self):
        self.image_rot = pygame.transform.rotate(self.image, self.angle)

    def move(self):
        new_x = self.state[0] * self.scale
        new_y = (self.state[1]-480) * self.scale +480
        self.pos = (new_x, new_y)

    def draw(self, surface):
        rect = self.image_rot.get_rect()
        rect.center = self.pos
        rect.centery = self.screen_height - rect.centery 
        surface.blit(self.image_rot, rect)

    def get_pos(self):
        return self.state[0:3]

    def get_angle_2d(self):
        v1 = [1,0,0]
        v2 = np.dot(self.state[3:12].reshape([3,3]), v1)
        cosang = np.dot(v1, v2)
        angle= np.degrees(np.arccos(cosang))
        axis = np.cross(v1, v2)
        #The y axis is used as the first check. If at 0, the z axis is used to check rotation
        #Helpful to avoid unintended rotational behaviours
        if axis[1] < 0:
            angle *= -1.
        elif axis[1] == 0:
            if axis[2] <0:
                angle *= -1
        return angle, axis
    

    def debug(self):
        print("Pos:", self.state[0:3])
        print("COM:", self.curCom)
        print("Force:", self.force)
        print("Lin Mon:", self.state[12:15]) 
        print("Angle:", self.get_angle_2d())
        print(f"COP: {self.cop} {[self.r[0,0]-(self.r[0,0]-self.r[0,1])*self.cop ,self.r[1,0] - (self.r[1,0]-self.r[1,1])*self.cop, 0]}")
        print("Torque:", self.torque)
        print("Linear:", self.state[15:18])
    
    def pause(self):
        self.paused = not self.paused


