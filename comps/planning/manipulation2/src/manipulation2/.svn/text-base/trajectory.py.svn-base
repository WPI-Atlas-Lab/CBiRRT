import openravepy
import types

def bind(mod):
   mod.blendtrajectory = types.MethodType(blendtrajectory,mod)
   mod.executeblendedtrajectory = types.MethodType(executeblendedtrajectory,mod)

def blendtrajectory(mod,
      traj=None, trajfile=None, maxsmoothiter=None, resolution=None,
      execute=None, ignore_collisions=None, blend_radius=None, blend_attempts=None,
      linearity_threshold=None, blend_step_size=None, trajfileout=None):
   cmd = 'BlendTrajectory'
   if trajfile is not None:
      cmd += ' trajfile %s' % trajfile
   if maxsmoothiter is not None: # number of smoothing iterations (int)
      cmd += ' maxsmoothiter %d' % maxsmoothiter
   if resolution is not None:  # per-joint collision checking step size for straight segments (list of floats)
      cmd += ' resolution %s' % (' '.join(['%.04f'%r for r in resolution]) )
   if execute is not None:
      cmd += ' execute %d' % (1 if execute else 0)
   if ignore_collisions is not None:
      cmd += ' ignore_collisions %d' % (1 if ignore_collisions else 0)
   if blend_radius is not None: # blend size (radians)
      cmd += ' blend_radius %0.04f' % blend_radius
   if blend_attempts is not None: # how many tries to make for a single point. blend radius will be cut in half each try. (int)
      cmd += ' blend_attempts %d' % blend_attempts
   if linearity_threshold is not None: # tolerance for linear segments (float)
      cmd += ' linearity_threshold %0.04f' % linearity_threshold
   if blend_step_size is not None: # path step size for checking blends (radians)
      cmd += ' blend_step_size %0.04f' % blend_step_size
   if trajfileout is not None:
      cmd += ' trajfileout %s' % trajfileout
   # this must be last, since all arguments after it are ignored by ExecuteBlendedTrajectory!
   if traj is not None:
      cmd += ' traj %s' % traj.serialize(0)
   resp = mod.SendCommand(cmd)
   if not execute and not trajfileout:
      traj = openravepy.RaveCreateTrajectory(mod.GetEnv(),'').deserialize(resp)
      return traj
   else:
      return resp


# This is actually just a copy of blendtrajectory underneath,
# with a different for the execute flag.
def executeblendedtrajectory(mod,
      traj=None, trajfile=None, maxsmoothiter=None, resolution=None,
      execute=None, ignore_collisions=None, blend_radius=None, blend_attempts=None,
      linearity_threshold=None, blend_step_size=None, trajfileout=None):
   cmd = 'ExecuteBlendedTrajectory'
   if trajfile is not None:
      cmd += ' trajfile %s' % trajfile
   if maxsmoothiter is not None: # number of smoothing iterations (int)
      cmd += ' maxsmoothiter %d' % maxsmoothiter
   if resolution is not None:  # per-joint collision checking step size for straight segments (list of floats)
      cmd += ' resolution %s' % (' '.join(['%.04f'%r for r in resolution]) )
   if execute is not None:
      cmd += ' execute %d' % (1 if execute else 0)
   if ignore_collisions is not None:
      cmd += ' ignore_collisions %d' % (1 if ignore_collisions else 0)
   if blend_radius is not None: # blend size (radians)
      cmd += ' blend_radius %0.04f' % blend_radius
   if blend_attempts is not None: # how many tries to make for a single point. blend radius will be cut in half each try. (int)
      cmd += ' blend_attempts %d' % blend_attempts
   if linearity_threshold is not None: # tolerance for linear segments (float)
      cmd += ' linearity_threshold %0.04f' % linearity_threshold
   if blend_step_size is not None: # path step size for checking blends (radians)
      cmd += ' blend_step_size %0.04f' % blend_step_size
   if trajfileout is not None:
      cmd += ' trajfileout %s' % trajfileout
   # this must be last, since all arguments after it are ignored by ExecuteBlendedTrajectory!
   if traj is not None:
      cmd += ' traj %s' % traj.serialize(0)
   resp = mod.SendCommand(cmd)
   if not execute and not trajfileout:
      traj = openravepy.RaveCreateTrajectory(mod.GetEnv(),'').deserialize(resp)
      return traj
   else:
      return resp

