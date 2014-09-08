% Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
%   Author: Dmitry Berenson <dberenso@cs.cmu.edu>
%
%   Redistribution and use in source and binary forms, with or without
%   modification, are permitted provided that the following conditions are met:
%
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of Intel Corporation nor Carnegie Mellon University,
%       nor the names of their contributors, may be used to endorse or
%       promote products derived from this software without specific prior
%       written permission.
%
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
%   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
%   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
%   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
%   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
%   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
%   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
%   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

%An example of TSR chaining for door opening
%This example uses a single TSR Chain to define constraints for opening a door. The chain has either one or two elements, depending on the 'chainlength' variable
clear all;
close all;

%'chainlength' defines whether to use a chain of length 1 or 2
%length 1 only allows rotation about the door hinge
%length 2 allows rotation about the door hinge and the door handle
chainlength = 2;

%load the environment
orEnvLoadScene('../../ormodels/environments/intelkitchen_robotized_herb2.env.xml',1);
robotid = orEnvGetBody('BarrettWAM');

%set printing and display options
orEnvSetOptions('debug 3')
orEnvSetOptions('collision ode')

%create problem instances 
probs.cbirrt = orEnvCreateProblem('CBiRRT','BarrettWAM');
probs.cbirrt2 = orEnvCreateProblem('CBiRRT','kitchen',0); %this is only for executing the trajectory for the fridge door

%get the descriptions of the robot's manipulators
manips = orRobotGetManipulators(robotid);

%define which joints are active
jointdofs = 0:orRobotGetActiveDOF(robotid);
activedofs = [manips{1}.armjoints];

%start the robot in a reasonable location and configuration
orBodySetTransform(robotid,[-0.0024,1,0,-1,-0.0024,0,0,0,1,1.2996,-0.6217,0.000]');
initdofvals = [3.68,   -1.9,   -0.0000,    2.2022,   -0.0000,    0.0000,    -2.1, 2.6,  -1.9,   -0.0000,    2.2022,   -3.14,   0.0000,    -1.0];
orRobotSetDOFValues(robotid,initdofvals,[manips{1}.armjoints manips{2}.armjoints]);

%set the active dof
orRobotSetActiveDOFs(robotid,activedofs);

%get info about the fridge hinge
fridgejointind = 6;
jointtm = str2num(orProblemSendCommand(['GetJointTransform name kitchen jointind ' num2str(fridgejointind)],probs.cbirrt));

%set up the grasp
T0_RH1 = MakeTransform(rodrigues([0 0 pi])*reshape([-1 0 0 0 0 -1 0 -1 0]',3,3),[1.4351 -0.21 1.1641]');

%call IK to find start configuration
startik = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 0 ' num2str([GetRot(T0_RH1),GetTrans(T0_RH1)]) ],probs.cbirrt);
orRobotSetDOFValues(robotid,str2num(startik));

%close the fingers
handdof = 2*ones(1,3);
orRobotSetDOFValues(robotid,handdof,manips{1}.handjoints);

%define TSR chains

%place the first TSR's reference frame at the door's hinge with no rotation relative to world frame
T0_w0 = MakeTransform(rodrigues([0 0 pi])*reshape([1 0 0 0 1 0 0 0 1]',3,3),jointtm(10:12));

if(chainlength == 1)
    %get the TSR's offset frame in w0 coordinates
    Tw0_e = inv(T0_w0)*T0_RH1;
    %define bounds to only allow rotation of the door about z axis up to pi/2
    Bw0 = [0 0   0 0   0 0   0 0   0 0   0 pi];
    %serialize the TSR
    TSRstring1 = SerializeTSR(0,'NULL',T0_w0,Tw0_e,Bw0);
    %serialize the TSRChain
    TSRChainString = SerializeTSRChain(0,0,1,1,TSRstring1,'kitchen',fridgejointind);
	
    %set the desired amount to open the door
    door_rot = pi/2.3;
    handle_rot = 0;
else
    %get the first TSR's offset from the hinge axis to the handle (but calculated using info from the hand location b/c it's easier)
    Tw0_e = inv(T0_w0)*MakeTransform(GetRot(T0_RH1)',GetTrans(T0_RH1)'+[0 0.155 0]');
    Tw0_e(1:3,1:3) = eye(3);
	
    %keep this here for plotting (useful for debugging)
    %orEnvPlot(GetTrans(T0_w0*Tw0_e),'size',15)

    %define bounds to only allow rotation of the door about z axis up to pi	
    Bw0 = [0 0   0 0   0 0   0 0   0 0   0 pi];

    %get the second TSR's offset from the handle to the hand
    T0_w1 = inv(T0_w0*Tw0_e)*T0_RH1;
	
    %define bounds to allow rotation of the hand about z axis of the handle from -pi/2 to 0
    Bw1 = [0 0   0 0   0 0   0 0   0 0   -pi/2 0];

    %serialize the TSRs
    TSRstring1 = SerializeTSR(0,'NULL',T0_w0,Tw0_e,Bw0);
    TSRstring2 = SerializeTSR(0,'NULL',eye(4),T0_w1,Bw1); %note the T0_w value here is ignored by the planner
    TSRChainString = SerializeTSRChain(0,0,1,2,[TSRstring1 ' ' TSRstring2],'kitchen',fridgejointind);
    
    %set the desired amount to open the door
    door_rot = pi/1.5;
    %set the desired amount to rotate the hand about the handle at the goal
    handle_rot = -pi/2.1; 
end

%keep track of how many DOF of the mimic body are being mimiced
numTSRChainMimicDOF = 1;

%get goal transform
Tdoor_rot = MakeTransform(rodrigues([0 0 door_rot]),[0 0 0]');
Thandle_rot = MakeTransform(rodrigues([0 0 handle_rot]),[0 0 0]');
T0_doornew = T0_w0*Tdoor_rot;
T0_RH2 = T0_doornew*Tw0_e*Thandle_rot*inv(T0_doornew*Tw0_e)* T0_doornew*inv(T0_w0)* T0_RH1;

%get goal IK solution
goalik = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 0 ' num2str([GetRot(T0_RH2),GetTrans(T0_RH2)])],probs.cbirrt);
%orRobotSetDOFValues(robotid,str2num(goalik));

%keep this here to debug where the door should be
%orRobotSetDOFValues(orEnvGetBody('kitchen'),door_rot,fridgejointind)

%set robot to starting configuration
orRobotSetDOFValues(robotid,str2num(startik));

%call the cbirrt planner, it will generate a file with the trajectory called 'cmovetraj.txt'
goaljoints = [str2num(goalik), zeros(1,numTSRChainMimicDOF)];
orProblemSendCommand(['RunCBiRRT smoothingitrs 50 jointgoals '  num2str(numel(goaljoints)) ' ' num2str(goaljoints) TSRChainString],probs.cbirrt);

%execute the trajectories generated by the planner
orProblemSendCommand(['traj cmovetraj.txt'],probs.cbirrt);
orProblemSendCommand(['traj cmovetraj.txt'],probs.cbirrt2);
orEnvWait(1);
