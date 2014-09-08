import types

def bind(mod):
   mod.liftarmgeneralik = types.MethodType(liftarmgeneralik,mod)

def liftarmgeneralik(mod, exec_=None, minsteps=None, maxsteps=None, direction=None, filename=None):
   cmd = 'LiftArmGeneralIK'
   if exec_ is not None:
      if exec_:
         cmd += ' exec 1'
      else:
         cmd += ' exec 0'
   if minsteps is not None:
      cmd += ' minsteps %d' % minsteps
   if maxsteps is not None:
      cmd += ' maxsteps %d' % maxsteps
   if direction is not None:
      cmd += ' direction %s' % (' '.join([str(v) for v in direction]))
   if filename is not None:
      cmd += 'filename %s' % filename
   resp = mod.SendCommand(cmd)
   return resp
