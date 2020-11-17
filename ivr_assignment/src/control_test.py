import numpy as np

def fk(joints):
    a,b,c,d = joints
    x = np.sin(a)*(np.sin(b)*np.cos(c)*(3*np.cos(d)+3.5)+3*np.cos(b)*np.sin(d)) + np.cos(a)*np.sin(c)*(3*np.cos(d)+3.5)
    y = np.cos(a)*(np.sin(b)*np.cos(c)*(-3*np.cos(d)-3.5) - 3*np.cos(b)*np.sin(d)) + np.sin(a)*np.sin(c)*(3*np.cos(d)+3.5)
    z = np.cos(b)*np.cos(c)*(3*np.cos(d)+3.5) - 3*np.sin(b)*np.sin(d) + 2.5
    end_effector = np.array([x,y,z])
    return end_effector

if __name__ == "__main__":
    test = [[1.6,1.6,1.6,-1.6] 	#1
           ,[1.6,-1.6,1.6,1.6] 	#2
           ,[1.6,1.6,-1.6,-1.6]	#3
           ,[-1.6,1.6,1.6,-1.6]	#4
           ,[-1.6,1.6,-1.6,-1.6]	#5
           ,[0.8,1.6,1.6,-1.6]	#6
           ,[-0.8,-1.6,-1.6,1.6]	#7
           ,[1,1,1,-1]			#8
           ,[1,0.3,1.8,0.5]		#9
           ,[-1,-1,0.5,0.5]]		#10

    for j in test:
        print(j,fk(j))
