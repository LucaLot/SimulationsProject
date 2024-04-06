#Luca Lotito
#Helper function to provide support to themain simulation
from scipy.spatial.transform import Rotation as R
import numpy as np


def setup_rotation(angle):
    return R.from_euler('y',angle, degrees=True).as_matrix()

def star(v):
    vs = np.zeros([3,3])
    vs[0][0] = 0
    vs[1][0] = v[2]
    vs[2][0] = -v[1]
    vs[0][1] = -v[2]
    vs[1][1] = 0
    vs[2][1] = v[0]
    vs[0][2] = v[1] 
    vs[1][2] = -v[0]
    vs[2][2] = 0
    return vs;   

def orthonormalize(m):
    mo = np.zeros([3,3])
    r0 = m[0,:]
    r1 = m[1,:]
    r2 = m[2,:]
        
    r0new = r0 / np.linalg.norm(r0)
        
    r2new = np.cross(r0new, r1)
    r2new = r2new / np.linalg.norm(r2new)

    r1new = np.cross(r2new, r0new)
    r1new = r1new / np.linalg.norm(r1new)

    mo[0,:] = r0new
    mo[1,:] = r1new
    mo[2,:] = r2new
    return mo

#Approximations of curves
#Based on experimental findings and NASA simulations 
def liftCoefficent(AOA):
    return 0.2*np.sin(2*np.deg2rad(AOA)) 

def dragCoefficent(AOA):
    return -np.cos(2*np.deg2rad(AOA)) + 1.007
#Very Heavy Approximation
def calculateCOP(AOA):
    return np.cos(2*np.deg2rad(AOA))*.45 + .55

#Calculates current rotation of the body
def getFrameOfRef(rot, rp, com, m, pos):
    r = np.dot(rot, rp) + np.tile(com, (5,1)).T 
    com = np.sum(r * np.tile(m, (3,1)), axis=1) / np.sum(m)
    com = com + pos
    return r, com

#Gets initial inertia
def findInertiaWorld(rp, mass):
    inertia = np.zeros((3,3))
    inertia[0][0] = np.sum(np.multiply(np.power(rp[1,:],2) + np.power(rp[2,:],2), mass))
    inertia[1][1] = np.sum(np.multiply(np.power(rp[2,:],2) + np.power(rp[0,:],2), mass))
    inertia[2][2] = np.sum(np.multiply(np.power(rp[0,:],2) + np.power(rp[1,:],2), mass))
    inertia[0][1] = inertia[1, 0] = np.sum(np.multiply(np.multiply(rp[0,:], rp[1,:]), mass))
    inertia[0,2] = inertia[2,0] = np.sum(np.multiply(np.multiply(rp[0,:], rp[2,:]), mass))
    inertia[1,2] = inertia[2,1] = np.sum(np.multiply(np.multiply(rp[1,:], rp[2,:]), mass))
    
    w, v = np.linalg.eig(inertia)
    return w, v
#Used for finding the direction of drag
def getSign(num):
    return 1 if num>=0 else -1
