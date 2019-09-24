import maya.cmds as cmds

print("Startup")

#Can set up custom settings
#Use cmds.currentUnit to choose desired unit on startup

#Change the current time unit to ntsc
cmds.currentUnit(time = 'ntsc')

#Change the currentlinear unit to centimeters
cmds.currentUnit(linear = 'cm')

import py101ui.rigui as ui