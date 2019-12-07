import pybullet as pb
import pybullet_data as pb_data


customPath = "/home/vahid/pybullet_test"


#pybullet setup
physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(customPath)
pb.setAdditionalSearchPath(pb_data.getDataPath())


#Loading Plane
pb.setGravity(0, 0, -9.8)
planeId = pb.loadURDF("plane.urdf")


#Loading target object
#box.sdf is in order to load load target object mesh
tarObjID = pb.loadSDF("box.sdf")
tarPos, tarOrn = pb.getBasePositionAndOrientation(tarObjID[0])
print("Target Object Pose:")
print(tarPos, tarOrn)


#Loading End_Effector
#endFStartPos = (tarPos[0] - 0.1, tarPos[1], tarPos[2])
endFStartPos = (0, 0, 0.5)
endFStartOrientation = pb.getQuaternionFromEuler([0, 0, 90])
endFId = pb.loadURDF("parallel_jaw.urdf", endFStartPos, endFStartOrientation)

print("Num of Joints in Current End_Effector:")
print(pb.getNumJoints(endFId))


#I need to spend time to have control over joints
#but for now just showing end_effector around target object is enough


pb.stepSimulation()


#Keep GUI Open
raw_input()
