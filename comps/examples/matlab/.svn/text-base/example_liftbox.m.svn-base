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


%An example of using TSRs for lifting a box with HERB2 
%(constraints on closed-chain kinematics and keeping the box upright
clear all;
close all;

%load the robot into the environment
orEnvLoadScene('../../ormodels/robots/herb2_padded.robot.xml',1);
robotid = orEnvGetBody('Herb2');

%load the box into the environment
objid = orEnvCreateKinBody('box','../../ormodels/objects/misc/liftingbox.kinbody.xml');

%set printing and display options
orEnvSetOptions('debug 3')
orEnvSetOptions('collision ode')

%place the box in front of the robot
objtm = [1;0;0;0;1;0;0;0;1;0.6923;0.00;0.5689;];
orBodySetTransform(objid,objtm);

%create the problem instances we need
probs.manip = orEnvCreateProblem('Manipulation','Herb2');
probs.cbirrt = orEnvCreateProblem('CBiRRT','Herb2');

%get the descriptions of the robot's manipulators
manips = orRobotGetManipulators(robotid);

%define which joints are active
jointdofs = 0:orRobotGetActiveDOF(robotid);
activedofs = [manips{1}.armjoints,manips{2}.armjoints];

%set initial configuration
initdofvals = [3.68,   -1.9,   -0.0000,    2.2022,   -0.0000,    0.0000,    0.0000, 2.6,  -1.9,   -0.0000,    2.2022,   -0.0001,   0.0000,    0.0000];
orRobotSetDOFValues(robotid,initdofvals,activedofs);

%define targets for both hands relative to box in start pose
transoffset0 = [0.0 -0.285 0];
transoffset1 = [0.0 0.305 0];
handtrans0 = objtm(10:12)' + transoffset0;
handrot0 = rodrigues([-pi/2 0 0]);
handtrans1 = objtm(10:12)' + transoffset1;
handrot1 = rodrigues([pi/2 0 0]);
Tik0 = MakeTransform(handrot0,handtrans0');
Tik1 = MakeTransform(handrot1,handtrans1');


%get the ik solutions for both arms for the box in the start pose
orRobotSetActiveDOFs(robotid,manips{1}.armjoints);
startik0 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 0 ' num2str([GetRot(Tik0),GetTrans(Tik0)]) ],probs.cbirrt);
orRobotSetActiveDOFs(robotid,manips{2}.armjoints);
startik1 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 1 ' num2str([GetRot(Tik1),GetTrans(Tik1)]) ],probs.cbirrt);

orRobotSetActiveDOFs(robotid,activedofs);
startik = [startik0 ' ' startik1];
orRobotSetDOFValues(robotid,str2num(startik));

%define the target transform of the goal
objtm2 = [1;0;0;0;1;0;0;0;1;0.6923;0.00;1.3989;];
orBodySetTransform(objid,objtm2);

%define targets for both hands relative to box in goal pose
handtrans0 = objtm2(10:12)' + transoffset0;
handtrans1 = objtm2(10:12)' + transoffset1;
Tik0 = MakeTransform(handrot0,handtrans0');
Tik1 = MakeTransform(handrot1,handtrans1');

%get the ik solutions for both arms for the box in the goal pose
orRobotSetActiveDOFs(robotid,manips{1}.armjoints);
goalik0 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 0 ' num2str([GetRot(Tik0),GetTrans(Tik0)]) ],probs.cbirrt);
orRobotSetActiveDOFs(robotid,manips{2}.armjoints);
goalik1 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 1 ' num2str([GetRot(Tik1),GetTrans(Tik1)]) ],probs.cbirrt);

orRobotSetActiveDOFs(robotid,activedofs);
goalik = [goalik0 ' ' goalik1];
orRobotSetDOFValues(robotid,str2num(goalik));


%reset the box to the start pose
orBodySetTransform(objid,objtm);

%plan a reaching motion to grab the box
orRobotSetDOFValues(robotid,initdofvals,activedofs);
orProblemSendCommand(['RunCBiRRT jointgoals '  num2str(numel(str2num(startik))) ' ' num2str(startik) ],probs.cbirrt);

%execute the planned trajectory
orProblemSendCommand(['traj cmovetraj.txt'],probs.cbirrt);
orEnvWait(robotid);

%specify a TSR for manipulator 0 to keep the box from tilting during the motion
TSRstring0 = SerializeTSR(0,'NULL',MakeTransform(handrot0,transoffset0'),eye(4),[-1000 1000  -1000 1000  -1000 1000  0 0  -pi pi  0 0]);

%specify a TSR for manipulator 1 to keep its end-effector fixed relative to the box
TSRstring1 = SerializeTSR(1,'box body',MakeTransform(handrot1,transoffset1'),eye(4),zeros(1,12));

%pack the TSRs into a string
TSRChainString = [SerializeTSRChain(0,0,1,1,TSRstring0,'NULL',[]) ' ' SerializeTSRChain(0,0,1,1,TSRstring1,'NULL',[])];

%grab the box with manipulator 0
orProblemSendCommand(['setactivemanip index 0'],probs.manip);
orProblemSendCommand(['GrabBody name box'],probs.manip)

%plan the lifting motion
goals_in = [str2num(goalik)];
orProblemSendCommand(['RunCBiRRT timelimit 10 smoothingitrs 100 jointgoals '  num2str(numel(goals_in)) ' ' num2str(goals_in) ' ' TSRChainString],probs.cbirrt);

%execute the planned trajectory
orProblemSendCommand(['traj cmovetraj.txt'],probs.cbirrt);
orEnvWait(robotid);
