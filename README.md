CBiRRT
======

Source code from: http://sourceforge.net/projects/comps/

Comps is developed by Dmitry Berenson from ARC Lab at WPI (http://arc.wpi.edu/)

##Installation:

Dependencies:

```
sudo apt-get install libqhull-dev libqhull5 libnewmat10-dev libnewmat10ldbl
```

Build plugins:

```
./buildallpulgins.sh


Edit .bashrc:

```
export OPENRAVE_PLUGINS=[COMPS_DIR]/plugins:$OPENRAVE_PLUGINS
```
