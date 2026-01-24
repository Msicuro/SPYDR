import importlib
import maya.cmds as cmds

print('UI')

#The UI Class
class RDojo_UI(object):
	"""docstring for RDojo_UI"""
	def __init__(self, *args):
		print("In RDojo_UI")
		mi = cmds.window("MayaWindow", ma = True, q = True)
		for m in mi:
			if m == "RDojo_Menu":
				cmds.deleteUI('RDojo_Menu' ,m=True)

		mymenu = cmds.menu('RDojo_Menu', label = 'RD Menu', to = True, p = 'MayaWindow')
		cmds.menuItem(label = 'Rig_Tool', p = mymenu, command = self.ui)

		#Create a dictionary to store UI elements we can access later
		self.UIElements = {}


	def ui(self, *args):
		#Check to see if the UI exists
		windowName = "Window"
		if cmds.window(windowName, exists = True):
			cmds.deleteUI(windowName)

		#Define width and height for buttons and windows
		windowWidth = 480
		windowHeight = 80
		buttonWidth = 100
		buttonHeight = 30

		#Creating window and saving to self.UIElements
		self.UIElements["window"] = cmds.window(windowName, width = windowWidth, height = windowHeight, title = "RDojo_UI", sizeable=True)

		#Create layouts
		self.UIElements["mainColLayout"] = cmds.columnLayout(adjustableColumn = True)
		self.UIElements["guiFrameLayout1"] = cmds.frameLayout(label = "Layout", parent = self.UIElements["mainColLayout"])
		self.UIElements["guiFlowLayout1"] = cmds.flowLayout(vertical = False, width = windowWidth, height = windowHeight/2, wrap = True , backgroundColor = [0.2, 0.2, 0.2], parent = self.UIElements["guiFrameLayout1"])

		#Menu listing all the layout files
		cmds.separator(width = 10, horizontal = True, style = 'none', parent = self.UIElements["guiFlowLayout1"])

		#Create the Rig Arm button
		self.UIElements["rig_arm_button"] = cmds.button(label = "rig arm", width = buttonWidth, height = buttonHeight, backgroundColor = [0.2, 0.4, 0.2], parent = self.UIElements["guiFlowLayout1"], command = self.rigarm)
		self.UIElements["rig_leg_button"] = cmds.button(label = "rig leg", width = buttonWidth, height = buttonHeight, backgroundColor = [0.2, 0.4, 0.2], parent = self.UIElements["guiFlowLayout1"])

		#Show the window
		cmds.showWindow(windowName)

	def rigarm(*args):
		print("Rig Arm is cool")
		import First_auto_rig.rig_arm as rig_arm
		importlib.reload(rig_arm)
		print(rig_arm)
		rig_arm = rig_arm.Rig_Arm()
		print(rig_arm)
		rig_arm.rig_arm()


