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

%function outstring = SerializeTSR(manipindex,bodyandlink,T0_w,Tw_e,Bw)
%
%Input:
%manipindex (int): the 0-indexed index of the robot's manipulator
%bodyandlink (str): body and link which is used as the 0 frame. Format 'body_name link_name'. To use world frame, specify 'NULL'
%T0_w (double 4x4): transform matrix of the TSR's reference frame relative to the 0 frame
%Tw_e (double 4x4): transform matrix of the TSR's offset frame relative the w frame
%Bw (double 6x2 or 1x12): bounds in x y z roll pitch yaw. Format: [x_min x_max; y_min y_max;...]
%
%Output:
%outstring (str): string to use for SerializeTSRChain function

function outstring = SerializeTSR(manipindex,bodyandlink,T0_w,Tw_e,Bw)

outstring = [num2str(manipindex) ' ' bodyandlink ' ' num2str([GetRot(T0_w),GetTrans(T0_w)]) ' ' num2str([GetRot(Tw_e),GetTrans(Tw_e)])];

if isequal(size(Bw),[6 2])
    Bw = Bw';
end

outstring = [outstring, ' ' num2str(Bw(:)')];
