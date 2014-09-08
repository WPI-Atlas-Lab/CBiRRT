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
'''An example of TSR chaining for door opening
This example uses a single TSR Chain to define constraints for opening a door. The chain has either one or two elements, depending on the 'chainlength' variable
'chainlength' defines whether to use a chain of length 1 or 2
length 1 only allows rotation about the door hinge
length 2 allows rotation about the door hinge and the door handle'''

from openravepy import *
from numpy import *
from str2num import *
from rodrigues import *
from TransformMatrix import *
from TSR import *
import time
import sys 

def WaitForController(robot):
    robot.WaitForController(0)

if __name__ == "__main__":
      
    chainlength = 2
    
    #load the environment if it is not already loaded
    try:
        orEnv
    except NameError:
        orEnv = Environment()
        orEnv.SetViewer('qtcoin')
    
    orEnv.Reset()
    orEnv.Load('../../ormodels/environments/intelkitchen_robotized_herb2.env.xml')
    robot = orEnv.GetRobots()[0]   
    kitchen = orEnv.GetRobots()[1]   

    #set printing and display options
    orEnv.SetDebugLevel(DebugLevel.Info)
    colchecker = RaveCreateCollisionChecker(orEnv,'ode')
    orEnv.SetCollisionChecker(colchecker)

    #create problem instances
    probs_cbirrt = RaveCreateProblem(orEnv,'CBiRRT')
    orEnv.LoadProblem(probs_cbirrt,'BarrettWAM')
    
    probs_cbirrt2 = RaveCreateProblem(orEnv,'CBiRRT')
    orEnv.LoadProblem(probs_cbirrt2,'kitchen')


    #set up joint indices
    activedofs = [0, 1, 2, 3, 4, 5, 6]
    initdofvals = r_[3.68,   -1.9,   -0.0000,    2.2022,   -0.0000,    0.0000,    -2.1, 2.6,  -1.9,   -0.0000,    2.2022,   -3.14,   0.0000,    -1.0]
    
    #start the robot in a reasonable location and configuration
    Trobot = array([-0.0024,1,0,-1,-0.0024,0,0,0,1,1.2996,-0.6217,0.000]).reshape(4,3).T
    robot.SetTransform(Trobot)
    robot.SetActiveDOFs(r_[0, 1, 2, 3, 4, 5, 6, 11, 12, 13, 14, 15, 16, 17])
    robot.SetActiveDOFValues(initdofvals)

    handdof = r_[(2*ones([1,3]))[0]];
    robot.SetActiveDOFs([7, 8, 9])
    robot.SetActiveDOFValues(handdof)
    
    #set the active dof
    robot.SetActiveDOFs(activedofs)

    #get info about the fridge hinge
    fridgejointind = 6

    jointtm = str2num(probs_cbirrt.SendCommand('GetJointTransform name kitchen jointind %d'%fridgejointind))

    #set up the grasp
    T0_RH1 = MakeTransform(rodrigues([0,0,pi])*(mat([-1, 0, 0, 0, 0, -1, 0, -1, 0]).T).reshape(3,3), mat([1.4351, -0.21, 1.1641]).T)

    

    startik = str2num(probs_cbirrt.SendCommand('DoGeneralIK exec nummanips 1 maniptm 0 %s'%SerializeTransform(T0_RH1)))
    robot.SetActiveDOFValues(startik)
    
    time.sleep(0.5) #let the simulator draw the scene
    
    #define TSR chains

    #place the first TSR's reference frame at the door's hinge with no rotation relative to world frame
    T0_w0 = MakeTransform(rodrigues([0,0,pi])*(mat([1, 0, 0, 0, 1, 0, 0, 0, 1]).T).reshape(3,3),mat(jointtm[9:12]).T)

    if chainlength == 1:
        #get the TSR's offset frame in w0 coordinates
        Tw0_e = linalg.inv(T0_w0)*T0_RH1
        #define bounds to only allow rotation of the door about z axis up to pi
        Bw0 = mat([0, 0,   0, 0,   0, 0,   0, 0,   0, 0,   0, pi])
        #serialize the TSR
        TSRstring1 = SerializeTSR(0,'NULL',T0_w0,Tw0_e,Bw0)
        #serialize the TSRChain
        TSRChainString = SerializeTSRChain(0,0,1,1,TSRstring1,'kitchen',mat(fridgejointind))
            
        #set the desired amount to open the door
        door_rot = pi/2.3;
        handle_rot = 0;
    else:
        #get the first TSR's offset from the hinge axis to the handle (but calculated using info from the hand location b/c it's easier)
        Tw0_e = linalg.inv(T0_w0)*MakeTransform(GetRot(T0_RH1).T,GetTrans(T0_RH1).T+mat([0, 0.155, 0]).T)
        Tw0_e[0:3][:,0:3] = eye(3)

        #define bounds to only allow rotation of the door about z axis up to pi     
        Bw0 = mat([0, 0,   0, 0,   0, 0,   0, 0,  0, 0,   0, pi])

        #get the second TSR's offset from the handle to the hand
        Tw1_e = linalg.inv(T0_w0*Tw0_e)*T0_RH1
            
        #define bounds to allow rotation of the hand about z axis of the handle from -pi/2 to 0
        Bw1 = mat([0, 0,   0, 0,   0, 0,   0, 0,   0, 0,   -pi/2, 0])

        #serialize the TSRs
        TSRstring1 = SerializeTSR(0,'NULL',T0_w0,Tw0_e,Bw0)
        TSRstring2 = SerializeTSR(0,'NULL',mat(eye(4)),Tw1_e,Bw1) #note the T0_w value here is ignored by the planner
        TSRChainString = SerializeTSRChain(0,0,1,2,TSRstring1 + ' ' + TSRstring2,'kitchen',mat(fridgejointind))
        
        #set the desired amount to open the door
        door_rot = pi/1.5
        #set the desired amount to rotate the hand about the handle at the goal
        handle_rot = -pi/2.1

    #keep track of how many DOF of the mimic body are being mimiced
    numTSRChainMimicDOF = 1;

    #get goal transform
    Tdoor_rot = MakeTransform(rodrigues([0, 0, door_rot]),mat([0, 0, 0]).T)
    Thandle_rot = MakeTransform(rodrigues([0, 0, handle_rot]),mat([0, 0, 0]).T)
    T0_doornew = T0_w0*Tdoor_rot;
    T0_RH2 = T0_doornew*Tw0_e*Thandle_rot*linalg.inv(T0_doornew*Tw0_e)* T0_doornew*linalg.inv(T0_w0)* T0_RH1;

    goalik = str2num(probs_cbirrt.SendCommand('DoGeneralIK exec nummanips 1 maniptm 0 %s'%SerializeTransform(T0_RH2)))
    robot.SetActiveDOFValues(goalik)

    #set robot to starting configuration
    robot.SetActiveDOFValues(startik)

    #call the cbirrt planner, it will generate a file with the trajectory called 'cmovetraj.txt'
    mimicvals = mat(zeros([1,numTSRChainMimicDOF]))
    goalik = mat(goalik)
    goaljoints = bmat('goalik  mimicvals')
    
    
    probs_cbirrt.SendCommand('RunCBiRRT smoothingitrs 50 jointgoals %d %s %s'%(size(goaljoints),Serialize1DMatrix(goaljoints), TSRChainString))
    probs_cbirrt.SendCommand('traj cmovetraj.txt')
    probs_cbirrt2.SendCommand('traj cmovetraj.txt')
    robot.WaitForController(0)
    kitchen.WaitForController(0)
    
    
    print "Press return to exit."
    sys.stdin.readline()

