# Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
#   Author: Dmitry Berenson <dberenso@cs.cmu.edu>
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor Carnegie Mellon University,
#       nor the names of their contributors, may be used to endorse or
#       promote products derived from this software without specific prior
#       written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
#   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-
'''An example of using TSRs for lifting a box with HERB2 (constraints on closed-chain kinematics and keeping the box upright'''

from openravepy import *
from numpy import *
from str2num import *
from rodrigues import *
from TransformMatrix import *
from TSR import *
import time
import sys


if __name__ == "__main__":
      
    #load the environment if it is not already loaded
    try:
        orEnv
    except NameError:
        orEnv = Environment()
        orEnv.SetViewer('qtcoin')
    
    #load the robot into the environment
    orEnv.Reset()
    orEnv.Load('../../ormodels/robots/herb2_padded.robot.xml')
    
    #load the box into the environment
    targobject = orEnv.ReadKinBodyXMLFile('../../ormodels/objects/misc/liftingbox.kinbody.xml')
    orEnv.AddKinBody(targobject)
    
#    orEnv.LockPhysics(True)
    #place the box in front of the robot
    T0_object = MakeTransform(mat(eye(3)),mat([0.6923,0.00,0.5689]).T)
    targobject.SetTransform(array(T0_object[0:3][:,0:4]))
    
    robot = orEnv.GetRobots()[0]   
    
    #set printing, display options, and collision checker
    orEnv.SetDebugLevel(DebugLevel.Info)
    colchecker = RaveCreateCollisionChecker(orEnv,'ode')
    orEnv.SetCollisionChecker(colchecker)

    #create problem instances
    probs_manip = RaveCreateProblem(orEnv,'Manipulation')
    orEnv.LoadProblem(probs_manip,'Herb2')

    probs_cbirrt = RaveCreateProblem(orEnv,'CBiRRT')
    orEnv.LoadProblem(probs_cbirrt,'Herb2')

    

    #set initial configuration
    arm0dofs = [0, 1, 2, 3, 4, 5, 6]
    arm1dofs = [11, 12, 13, 14, 15, 16, 17]
    activedofs = arm0dofs + arm1dofs
    initdofvals = r_[3.68,   -1.9,   -0.0000,    2.2022,   -0.0000,    0.0000,    0.0000, 2.6,  -1.9,   -0.0000,    2.2022,   -0.0001,   0.0000,    0.0000]
    robot.SetActiveDOFs(activedofs)
    robot.SetActiveDOFValues(initdofvals)    
    
 #   orEnv.LockPhysics(False)
    time.sleep(0.5) #let the simulator draw the scene
 #   orEnv.LockPhysics(True)
    
    
    #define targets for both hands relative to box in start pose
    transoffset0 = mat([0.0, -0.285, 0]).T
    transoffset1 = mat([0.0, 0.305, 0]).T
    handtrans0 = T0_object[0:3,3] + transoffset0;
    handrot0 = mat(rodrigues([-pi/2, 0, 0]))
    handtrans1 = T0_object[0:3,3] + transoffset1;
    handrot1 = mat(rodrigues([pi/2, 0, 0]))
    Tik0 = MakeTransform(handrot0,handtrans0);
    Tik1 = MakeTransform(handrot1,handtrans1);

   
    #get the ik solutions for both arms for the box in the start pose
    robot.SetActiveDOFs(arm0dofs)
    startik0 = probs_cbirrt.SendCommand('DoGeneralIK exec nummanips 1 maniptm 0 %s'%SerializeTransform(Tik0))
    robot.SetActiveDOFs(arm1dofs)
    startik1 = probs_cbirrt.SendCommand('DoGeneralIK exec nummanips 1 maniptm 1 %s'%SerializeTransform(Tik1))
    startik = '%s %s'%(startik0,startik1)

    robot.SetActiveDOFs(activedofs)
    robot.SetActiveDOFValues(str2num(startik))


    #define the target transform of the goal
    T0_object2 = MakeTransform(mat(eye(3)),mat([0.6923,0.00,1.3989]).T)
    targobject.SetTransform(array(T0_object2[0:3][:,0:4]))


    #define targets for both hands relative to box in goal pose
    handtrans0 = T0_object2[0:3,3] + transoffset0;
    handtrans1 = T0_object2[0:3,3] + transoffset1;
    Tik0 = MakeTransform(handrot0,handtrans0);
    Tik1 = MakeTransform(handrot1,handtrans1);



    #get the ik solutions for both arms for the box in the goal pose
    robot.SetActiveDOFs(arm0dofs)
    goalik0 = probs_cbirrt.SendCommand('DoGeneralIK exec nummanips 1 maniptm 0 %s'%SerializeTransform(Tik0))
    robot.SetActiveDOFs(arm1dofs)
    goalik1 = probs_cbirrt.SendCommand('DoGeneralIK exec nummanips 1 maniptm 1 %s'%SerializeTransform(Tik1))
    goalik = '%s %s'%(goalik0,goalik1)

    robot.SetActiveDOFs(activedofs)
    robot.SetActiveDOFValues(str2num(goalik))
    
    #reset the box to the start pose
    targobject.SetTransform(array(T0_object[0:3][:,0:4]))
    
    #plan a reaching motion to grab the box
    robot.SetActiveDOFValues(initdofvals)
    probs_cbirrt.SendCommand('RunCBiRRT jointgoals %d %s'%(len(str2num(startik)),Serialize1DMatrix(mat(str2num(startik)))))
   
    #execute the planned trajectory 
#    orEnv.LockPhysics(False)
    probs_cbirrt.SendCommand('traj cmovetraj.txt')
    robot.WaitForController(0)
#    orEnv.LockPhysics(True)
    
    #specify a TSR for manipulator 0 to keep the box from tilting during the motion
    TSRstring0 = SerializeTSR(0,'NULL',MakeTransform(handrot0,transoffset0),mat(eye(4)),mat([-1000, 1000,  -1000, 1000,  -1000, 1000,  0, 0,  -pi, pi,  0, 0]))

    #specify a TSR for manipulator 1 to keep its end-effector fixed relative to the box
    TSRstring1 = SerializeTSR(1,'box body',MakeTransform(handrot1,transoffset1),mat(eye(4)),mat([0, 0, 0, 0,  0, 0,  0, 0,  0, 0,  0, 0]))

    #pack the TSRs into a string
    TSRChainString = '%s %s'%(SerializeTSRChain(0,0,1,1,TSRstring0,'NULL',[]), SerializeTSRChain(0,0,1,1,TSRstring1,'NULL',[]))

    #grab the box with manipulator 0
    probs_manip.SendCommand('setactivemanip index 0')
    probs_manip.SendCommand('GrabBody name box')


    #plan the lifting motion
    goals_in = str2num(goalik);
    probs_cbirrt.SendCommand('RunCBiRRT timelimit 10 smoothingitrs 50 jointgoals %s %s %s'%(len(goals_in),Serialize1DMatrix(mat(goals_in)),TSRChainString))

    #execute the planned trajectory 
#    orEnv.LockPhysics(False)
    probs_cbirrt.SendCommand('traj cmovetraj.txt')
    robot.WaitForController(0)
#    orEnv.LockPhysics(True)

    print "Press return to exit."
    sys.stdin.readline()
#    orEnv.LockPhysics(False)
