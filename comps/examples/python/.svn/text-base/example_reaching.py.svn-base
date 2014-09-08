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
'''An example of TSR chaining for reaching for a bottle. This example uses two TSR Chains, each of length 1, to define the allowable goal end-effector poses'''

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
    
    orEnv.Reset()
    orEnv.Load('../../ormodels/environments/intelkitchen_robotized_herb2.env.xml')
    
    #load the bottle
    targobject = orEnv.ReadKinBodyXMLFile('../../ormodels/objects/household/juice_bottle_model.kinbody.xml')
    orEnv.AddKinBody(targobject)
    
    #put the bottle somewhere
    T0_object = MakeTransform(mat(eye(3)),mat([0.3602,  0.2226, 0.9214]).T)
    targobject.SetTransform(array(T0_object[0:3][:,0:4]))
    
    robot = orEnv.GetRobots()[0]   
    
    #set printing, display options, and collision checker
    orEnv.SetDebugLevel(DebugLevel.Info)
    colchecker = RaveCreateCollisionChecker(orEnv,'ode')
    orEnv.SetCollisionChecker(colchecker)

    #create problem instances
    probs_cbirrt = RaveCreateProblem(orEnv,'CBiRRT')
    orEnv.LoadProblem(probs_cbirrt,'BarrettWAM')

    #set up joint indices
    activedofs = [0, 1, 2, 3, 4, 5, 6]
    initdofvals = r_[3.68,   -1.9,   -0.0000,    2.2022,   -0.0000,    0.0000,    -2.1, 2.6,  -1.9,   -0.0000,    2.2022,   -3.14,   0.0000,    -1.0]
    
    #start the robot in a reasonable location and configuration
    Trobot = array([-0.0024,1,0,-1,-0.0024,0,0,0,1, 0.274, -0.6, 0.000]).reshape(4,3).T
    robot.SetTransform(array(Trobot))
    robot.SetActiveDOFs(r_[0, 1, 2, 3, 4, 5, 6, 11, 12, 13, 14, 15, 16, 17])
    robot.SetActiveDOFValues(initdofvals)

    #preshape the fingers
    handdof = r_[(0.5*ones([1,3]))[0]];
    robot.SetActiveDOFs([7, 8, 9])
    robot.SetActiveDOFValues(handdof)

    time.sleep(0.5) #let the simulator draw the scene

    #set the active dof
    robot.SetActiveDOFs(activedofs)

    #let's define two TSR chains for this task, they differ only in the rotation of the hand

    #first TSR chain

    #place the first TSR's reference frame at the object's frame relative to world frame
    T0_w = T0_object

    #get the TSR's offset frame in w coordinates
    Tw_e1 = MakeTransform(rodrigues([pi/2, 0, 0]),mat([0, 0.20, 0.1]).T)

    #define bounds to only allow rotation of the hand about z axis and a small deviation in translation along the z axis
    Bw = mat([0, 0,   0, 0,   -0.02, 0.02,   0, 0,   0, 0,   -pi, pi])

    TSRstring1 = SerializeTSR(0,'NULL',T0_w,Tw_e1,Bw)
    TSRChainString1 = SerializeTSRChain(0,1,0,1,TSRstring1,'NULL',[])


    #now define the second TSR chain
    #it is the same as the first TSR Chain except Tw_e is different (the hand is rotated by 180 degrees about its z axis)
    Tw_e2 = MakeTransform(rodrigues([0, pi, 0])*rodrigues([pi/2, 0, 0]),mat([0, 0.20, 0.1]).T)
    TSRstring2 = SerializeTSR(0,'NULL',T0_w,Tw_e2,Bw)
    TSRChainString2 = SerializeTSRChain(0,1,0,1,TSRstring2,'NULL',[])

    #call the cbirrt planner, it will generate a file with the trajectory called 'cmovetraj.txt'
    resp = probs_cbirrt.SendCommand('RunCBiRRT psample 0.25 %s %s'%(TSRChainString1,TSRChainString2))
    probs_cbirrt.SendCommand('traj cmovetraj.txt')
    
#    traj=RaveCreateTrajectory(orEnv,'BarrettWAM')
#    traj.Read('cmovetraj.txt',robot)
#    robot.GetController().SetPath(traj)
    robot.WaitForController(0)
    

    print "Press return to exit."
    sys.stdin.readline()


