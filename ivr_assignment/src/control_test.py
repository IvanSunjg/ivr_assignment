import numpy as np

def fk(joints):
    a,b,c,d = joints
    x = np.sin(a)*(np.sin(b)*np.cos(c)*(3*np.cos(d)+3.5)+3*np.cos(b)*np.sin(d)) + np.cos(a)*np.sin(c)*(3*np.cos(d)+3.5)
    y = np.cos(a)*(np.sin(b)*np.cos(c)*(-3*np.cos(d)-3.5) - 3*np.cos(b)*np.sin(d)) + np.sin(a)*np.sin(c)*(3*np.cos(d)+3.5)
    z = np.cos(b)*np.cos(c)*(3*np.cos(d)+3.5) - 3*np.sin(b)*np.sin(d) + 2.5
    end_effector = np.array([x,y,z])
    return end_effector

if __name__ == "__main__":
    test = [[0,0,0,0]
           ,[1,0,0,0]
           ,[0,np.pi/2,0,0]
           ,[0,0,np.pi/2,0]
           ,[0,0,0,np.pi/2]
           ,[np.pi/2,np.pi/2,0,0]
           ,[np.pi/4,0,np.pi/2,0]
           ,[0,np.pi/2,0,-np.pi/2]
           ,[0,np.pi/2,np.pi/2,-np.pi/2]
           ,[np.pi/4,np.pi/2,np.pi/2,-np.pi/2]]

    for j in test:
        print(j,fk(j))