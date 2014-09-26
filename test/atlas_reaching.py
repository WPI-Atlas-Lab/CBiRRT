from openravepy import *
import openravepy as rave
from numpy import *
from str2num import *
from rodrigues import *
from TransformMatrix import *
from TSR import *
import time
import sys


def getwholejoint(robot,jointvalue,activejoint):
    joint = numpy.zeros(robot.GetDOF())
    for x in xrange(len(activejoint)):
        joint[activejoint[x]]=jointvalue[x]
    # print len(joint.tolist())
    return joint.tolist()



if __name__ == "__main__":
      
    #load the environment if it is not already loaded
    try:
        orEnv
    except NameError:
        orEnv = Environment()
        orEnv.SetViewer('qtcoin')
    
    orEnv.Reset()
    orEnv.Load('./atlas_description/atlas.xml')
    orEnv.Load('data/table_atlas.xml')   

    robot = orEnv.GetRobots()[0]        
    init_transform = numpy.eye(4)
    init_transform[:3,3] = [.1, 0, .92712]
    robot.SetTransform(init_transform)
    
    activejoint = [robot.GetJoint("back_bkz").GetDOFIndex(),robot.GetJoint("back_bky").GetDOFIndex(),
                   robot.GetJoint("back_bkx").GetDOFIndex(),robot.GetJoint("neck_ay").GetDOFIndex(),
                   robot.GetJoint("l_leg_hpz").GetDOFIndex(),robot.GetJoint("l_leg_hpx").GetDOFIndex(),
                   robot.GetJoint("l_leg_hpy").GetDOFIndex(),robot.GetJoint("l_leg_kny").GetDOFIndex(),
                   robot.GetJoint("l_leg_aky").GetDOFIndex(),robot.GetJoint("l_leg_akx").GetDOFIndex(),
                   robot.GetJoint("r_leg_hpz").GetDOFIndex(),robot.GetJoint("r_leg_hpx").GetDOFIndex(),
                   robot.GetJoint("r_leg_hpy").GetDOFIndex(),robot.GetJoint("r_leg_kny").GetDOFIndex(),
                   robot.GetJoint("r_leg_aky").GetDOFIndex(),robot.GetJoint("r_leg_akx").GetDOFIndex(),
                   robot.GetJoint("l_arm_shy").GetDOFIndex(),robot.GetJoint("l_arm_shx").GetDOFIndex(),
                   robot.GetJoint("l_arm_ely").GetDOFIndex(),robot.GetJoint("l_arm_elx").GetDOFIndex(),
                   robot.GetJoint("l_arm_wry").GetDOFIndex(),robot.GetJoint("l_arm_wrx").GetDOFIndex(),
                   robot.GetJoint("r_arm_shy").GetDOFIndex(),robot.GetJoint("r_arm_shx").GetDOFIndex(),
                   robot.GetJoint("r_arm_ely").GetDOFIndex(),robot.GetJoint("r_arm_elx").GetDOFIndex(),
                   robot.GetJoint("r_arm_wry").GetDOFIndex(),robot.GetJoint("r_arm_wrx").GetDOFIndex()]

    start_point = [  0.0, 0.0, 0.0, 0.0,
                     2.57950881e-03, 6.72404021e-02, -2.78805792e-01, 6.19559944e-01,
                     -3.49206835e-01, -6.72566667e-02,

                      -1.57717208e-03, -5.83863221e-02,
                     -2.81613737e-01, 6.24628246e-01, -3.51236612e-01, 5.81570677e-02,


                     0.0, -1.3, 0.0, 0.0,
                     0.1, 0.0, 

                     0.5, 1.3,
                     0.1, -0.1, 0.1, 0.1]

    robot.SetDOFValues(getwholejoint(robot,start_point,activejoint))

    ##set printing, display options, and collision checker
    orEnv.SetDebugLevel(DebugLevel.Info)
    colchecker = RaveCreateCollisionChecker(orEnv,'ode')
    orEnv.SetCollisionChecker(colchecker)
    
    ##create problem instances
    probs_cbirrt = RaveCreateProblem(orEnv,'CBiRRT')
    orEnv.LoadProblem(probs_cbirrt,'atlas')

    time.sleep(0.5) #let the simulator draw the scene

    ##set the active dof
    activejoint = [robot.GetJoint("l_arm_shy").GetDOFIndex(),robot.GetJoint("l_arm_shx").GetDOFIndex(),
                   robot.GetJoint("l_arm_ely").GetDOFIndex(),robot.GetJoint("l_arm_elx").GetDOFIndex(),
                   robot.GetJoint("l_arm_wry").GetDOFIndex(),robot.GetJoint("l_arm_wrx").GetDOFIndex(),
                   robot.GetJoint("back_bkz").GetDOFIndex(),robot.GetJoint("back_bky").GetDOFIndex(),
                   robot.GetJoint("back_bkx").GetDOFIndex()]
    robot.SetActiveDOFs(activejoint)
    robot.SetActiveDOFValues([0.0, -1.3, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0])

    ################ IK solver #######################
    # #define targets for hand 
    # handtrans = mat([0.82, 0.21, 1.23373]).T
    # handrot = mat(rodrigues([pi/2, 0, pi/2]))
    # Tik = MakeTransform(handrot,handtrans);
    # print Tik
    # startik = probs_cbirrt.SendCommand('DoGeneralIK exec nummanips 0 maniptm 1 %s'%SerializeTransform(Tik))
    # print robot.GetActiveDOFValues()
    startik = "-2.56229520e-01 -9.15233016e-01 1.66518712e+00 1.59568906e+00 9.34448361e-01 6.38563752e-01 0.0 0.0 0.0"

    #plan a reaching motion to grab the box
    robot.SetActiveDOFValues([0.0, -1.3, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0])
    probs_cbirrt.SendCommand('RunCBiRRT jointgoals %d %s'%(len(str2num(startik)),Serialize1DMatrix(mat(str2num(startik)))))
   
    #execute the planned trajectory 
#    orEnv.LockPhysics(False)
    probs_cbirrt.SendCommand('traj cmovetraj.txt')
    robot.WaitForController(0)
#    orEnv.LockPhysics(True)
    

    # #let's define two TSR chains for this task, they differ only in the rotation of the hand

    # #first TSR chain

    # #place the first TSR's reference frame at the object's frame relative to world frame
    # T0_w = MakeTransform(mat(eye(3)),mat([0.85, 0.2, 1.25]).T)

    # #get the TSR's offset frame in w coordinates
    # Tw_e1 = MakeTransform(rodrigues([pi/2, 0, 0]),mat([0.4, 0.0, 0.1]).T)

    # #define bounds to only allow rotation of the hand about z axis and a small deviation in translation along the z axis
    # Bw = mat([0, 0,   0, 0,   -0.02, 0.02,   0, 0,   0, 0,   -pi, pi])

    # TSRstring1 = SerializeTSR(0,'NULL',T0_w,Tw_e1,Bw)
    # TSRChainString1 = SerializeTSRChain(0,1,0,1,TSRstring1,'NULL',[])


    # #now define the second TSR chain
    # #it is the same as the first TSR Chain except Tw_e is different (the hand is rotated by 180 degrees about its z axis)
    # Tw_e2 = MakeTransform(rodrigues([0, pi, 0])*rodrigues([pi/2, 0, 0]),mat([0.4, 0.0, 0.1]).T)
    # TSRstring2 = SerializeTSR(0,'NULL',T0_w,Tw_e2,Bw)
    # TSRChainString2 = SerializeTSRChain(0,1,0,1,TSRstring2,'NULL',[])

    # #call the cbirrt planner, it will generate a file with the trajectory called 'cmovetraj.txt'
    # resp = probs_cbirrt.SendCommand('RunCBiRRT psample 0.25 %s %s'%(TSRChainString1,TSRChainString2))
    # probs_cbirrt.SendCommand('traj cmovetraj1.txt')

    # #    traj=RaveCreateTrajectory(orEnv,'BarrettWAM')
    # #    traj.Read('cmovetraj.txt',robot)
    # #    robot.GetController().SetPath(traj)
    # robot.WaitForController(0)


    print "Press return to exit."
    sys.stdin.readline()


