import maya.cmds as cmds
import json
import tempfile

def writeJson(fileName,data):
	with open(fileName, 'w') as outfile:
		json.dump(data, outfile)
	file.close(outfile)

def readJson(fileName):
	with open(fileName, 'r') as outfile:
		data = (open(fileName, 'r').read())
	return data


def createJoint(name, position, instance):
	'''
	Takes in joint info as an argument and iterates through the name and position to create joint
	'''
	joint_list = [cmds.joint(n=name[i].replace('s_', instance), p=position[i]) for i in range(len(name))]
	cmds.select(cl=True)
	return(joint_list)


def createControl(ctrlInfo):
	'''
	Iterates through joint_names+positions to create control curves
	'''
	control_info = []
	for info in ctrlInfo:
		#Get WS position of joint
		pos = info[0]
		#Create an empty group
		ctrl_group = cmds.group(em=1, n="group_" + info[1])
		#Create circle control object
		ctrl = cmds.circle(n=info[1])
		#Parent the control under the group
		cmds.parent(ctrl,ctrl_group)
		#Move the group to the joint
		cmds.xform(ctrl_group, t = pos, ws = True)
		#Append control info to control_info List
		control_info.append([ctrl_group, ctrl])
	return(control_info)

def calculatePoleVectorPosition(joints):
	from maya import cmds , OpenMaya
	start = cmds.xform(joints[0], q = True, ws = True, t = True)
	mid = cmds.xform(joints[1], q = True, ws = True, t = True)
	end = cmds.xform(joints[2], q = True, ws = True, t = True)
	
	startVector = OpenMaya.MVector(start[0], start[1], start[2])
	midVector = OpenMaya.MVector(mid[0], mid[1], mid[2])
	endVector = OpenMaya.MVector(end[0], end[1], end[2])
	
	startEnd = endVector - startVector
	startMid = midVector - startVector
	dotP = startMid * startEnd
	proj = float(dotP) / float(startEnd.length())
	startEndN = startEnd.normal()
	projV = startEndN * proj
	arrowV = startMid - projV
	arrowV *= 5
	finalV = arrowV + midVector
	return([finalV.x, finalV.y, finalV.z])

def connectThroughBlendColors(parentsA, parentsB, children, instance, switchattr):
	constraints = []
	for i in range(len(children)):
		#Separate joint name with partition and store in a variable
		switch_Prefix = children[i].partition[2]
		#Create blend color nodes for Translate, Rotate and 
		#Scale and connect to the arm settings CTRL IK/FK attribute
		bcNode_Translate = cmds.shadingNode("blendColors", asUtility = True, name = "bcNode_Translate_Switch_" + switch_Prefix)
		cmds.connectAttr(switchattr, bcNode_Translate + ".blender")
		bcNode_Rotate = cmds.shadingNode("blendColors", asUtility = True, name = "bcNode_Rotate_Switch_" + switch_Prefix)
		cmds.connectAttr(switchattr, bcNode_Rotate + ".blender")
		bcNode_Scale = cmds.shadingNode("blendColors", asUtility = True, name = "bcNode_Scale_Switch_" + switch_Prefix)
		cmds.connectAttr(switchattr, bcNode_Scale + ".blender")
		constraints.append(bcNode_Translate, bcNode_Rotate, bcNode_Scale)

		#Input Parents
		cmds.connectAttr(parentsA[i] + ".translate", bcNode_Translate + ".color1")
		cmds.connectAttr(parentsA[i] + ".rotate", bcNode_Rotate + ".color1")
		cmds.connectAttr(parentsA[i] + ".scale", bcNode_Scale + ".color1")
		if parentsB != "None":
			cmds.connectAttr(parentsB[i] + ".translate", bcNode_Translate + ".color2")
			cmds.connectAttr(parentsB[i] + ".rotate", bcNode_Rotate + ".color2")
			cmds.connectAttr(parentsB[i] + ".scale", bcNode_Scale + ".color2")

		#Output to children
		cmds.connectAttr(bcNode_Translate + ".output", children[i] + ".translate")
		cmds.connectAttr(bcNode_Rotate + ".output", children[i] + ".rotate")
		cmds.connectAttr(bcNode_Scale + ".output", children[i] + ".scale")
	return(constraints)


def connectBlendColors(blend_attr, direct_conn_attrs, blend_conn_attrs, instance):
	'''
	Connects a blender attribute to two additional attributes using a blendColors node, intended for IK/FK switching
	Args:
		blend_attr: The attribute that will connect to the blendColors Blender attribute
		direct_conn_attrs: A list of attributes that the blend_attr will directly connect to
		blend_conn_attrs: A list of attributes that the outputR attribute of the blendColors will directly connect to
		instance: The instance the attribute is part of, typically left or right

	Returns:
		bcNode: The blendColors node
	'''
	# Create blend colors node
	bcNode = cmds.shadingNode("blendColors", asUtility = True, name = "{}_{}_blendColors".format(instance, blend_attr))

	# Set Color 1R, 1G & 1B to 0 and Color 2R, 2G, & 2B to 1
	for i in ["R", "G", "B"]:
		cmds.setAttr("{}.color1{}".format(bcNode, i), 0)
		cmds.setAttr("{}.color2{}".format(bcNode, i), 1)

	# Connect the blend attribute to the Blender attribute
	cmds.connectAttr("{}".format(blend_attr), "{}.blender".format(bcNode))

	# Connect the switch attribute to the direct connection attributes
	for i in direct_conn_attrs:
		cmds.connectAttr("{}".format(blend_attr), "{}".format(i))

	# Connect the output R attribute from the blend colors node to the blend connection attributes
	for i in blend_conn_attrs:
		cmds.connectAttr("{}.outputR".format(bcNode), "{}".format(i))

	return bcNode