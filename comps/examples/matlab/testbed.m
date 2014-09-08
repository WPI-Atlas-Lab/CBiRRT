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

%testbed
orEnvLoadScene('/pr/data/ormodels/environments/intelkitchen_robotized.env.xml',1);
%obj.SetJointValues([0.5*pi])
objectid = orEnvCreateKinBody('pan',['/pr/data/ormodels/objects/household/pan_rotating.kinbody.xml']);
T0_object = MakeTransform(eye(3),[0.5, -1.5, 0.92]');
orBodySetTransform(objectid, [GetRot(T0_object) GetTrans(T0_object)]');
orBodySetJointValues(objectid,[0.5*pi]);
robotid = orEnvGetBody('BarrettWAM');

%set printing and display options
orEnvSetOptions('debug 3')
orEnvSetOptions('publishanytime 1');

%create problem instances 
probs.cbirrt = orEnvCreateProblem('CBiRRT','BarrettWAM');

%define joint indices
jointdofs = 0:10;
manips = orRobotGetManipulators(1);
activedofs = setdiff(jointdofs,[7 8 9 10]);

%start the robot in a reasonable location and configuration
orBodySetTransform(robotid,[-0.9970 0.0775         0   -0.0775   -0.9970         0         0         0    1.0000    0.8680   -0.5969    0.7390]');
initdofvals = [pi/2 0 0 pi/2 0 0 0 0 0 0 0];
orRobotSetDOFValues(robotid,initdofvals,jointdofs);

%set the active dof
orRobotSetActiveDOFs(robotid,activedofs);


robottm = [-0.49999845;-0.86602497;0;0.86602497;-0.49999845;0;0;0;0.73900002;0.28;-0.41999999;0.73900002];

orBodySetTransform(1,robottm)
%orRobotSetDOFValues(1,[1.9473    0.9221    2.0136   -0.8727   -0.0001   -1.2502   -0.0000]);

orProblemSendCommand('RunCBiRRT psample 0.25  TSRChain 1 0 1 0 NULL  -0.00000  1.00000  0.00000 -1.00000  -0.00000  0.00000  0.00000  0.00000  1.00000  0.50000  -1.50000 0.92000  0.00000  0.00000  1.00000  -1.00000  0.00000  0.00000  -0.00000 -1.00000  0.00000  0.23000  0.16800  0.05000  -0.02000  0.02000  0.000000.00000  0.00000  0.00000  -0.26180  0.26180  0.00000  0.00000  0.00000 0.00000 NULL');

orProblemSendCommand(['traj cmovetraj.txt'],probs.cbirrt);
orEnvWait(1);
