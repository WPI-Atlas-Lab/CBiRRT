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

%function outstring = SerializeTSRChain(bSampleFromChain,bConstrainToChain,numTSRs,allTSRstring,mimicbodyname,mimicbodyjoints)
%
%Input:
%bSampleStartFromChain (0/1): 1: Use this chain for sampling start configurations 0:Ignore for sampling starts
%bSampleGoalFromChain (0/1): 1: Use this chain for sampling goal configurations   0:Ignore for sampling goals
%bConstrainToChain (0/1): 1: Use this chain for constraining configurations   0:Ignore for constraining
%numTSRs (int): Number of TSRs in this chain (must be > 0)
%allTSRstring (str): string of concetenated TSRs generated using SerializeTSR. Should be like [TSRstring 1 ' ' TSRstring2 ...]
%mimicbodyname (str): name of associated mimicbody for this chain (NULL if none associated)
%mimicbodyjoints (int [1xn]): 0-indexed indices of the mimicbody's joints that are mimiced (MUST BE INCREASING AND CONSECUTIVE [FOR NOW])
%
%Output:
%outstring (str): string to include in call to cbirrt planner

function outstring = SerializeTSRChain(bSampleStartFromChain,bSampleGoalFromChain,bConstrainToChain,numTSRs,allTSRstring,mimicbodyname,mimicbodyjoints)

outstring = [' TSRChain ' num2str(bSampleStartFromChain) ' ' num2str(bSampleGoalFromChain) ' ' num2str(bConstrainToChain) ' ' num2str(numTSRs) ' ' allTSRstring ' ' mimicbodyname];

if(numel(mimicbodyjoints) ~= 0)
    outstring = [outstring ' ' num2str(numel(mimicbodyjoints)) ' ' num2str(mimicbodyjoints)];
end


