import types

def bind(mod):
   mod.runcbirrt = types.MethodType(runcbirrt,mod)

class CbirrtException(Exception):
   pass

# Get the cbirrt command string
def runcbirrt(mod, argstr=None, jointgoals=None, smoothingitrs=None, filename=None, timelimit=None, allowlimadj=None):
   cmd = 'runcbirrt'
   if argstr is not None:
      cmd += ' ' + argstr
   if jointgoals is not None:
      cmd += ' jointgoals %d %s' % (len(jointgoals), ' '.join([str(v) for v in jointgoals]))
   if smoothingitrs is not None:
      cmd += ' smoothingitrs %d' % smoothingitrs
   if filename is not None:
      cmd += ' filename %s' % filename
   if timelimit is not None:
      cmd += ' timelimit %f' % timelimit
   if allowlimadj is not None and allowlimadj:
      cmd += ' allowlimadj 1'
   resp = mod.SendCommand(cmd)
   if int(resp) != 1:
      raise CbirrtException('runcbirrt failed!')
