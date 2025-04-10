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


def createControl(ctrlInfo, flip=True):
	'''
	Iterates through joint_names+positions to create control curves
	'''
	control_info = []
	for info in ctrlInfo:
		#Create an empty group
		ctrl_group = cmds.group(em=1, n="{}_GRP".format(info[1]) )
		#Create circle control object
		ctrl = cmds.circle(n=info[1])
		if flip:
			cmds.setAttr("{}.normalX".format(ctrl[1]), 90)
		#Parent the control under the group
		cmds.parent(ctrl,ctrl_group)
		#Move the group to the joint, if it's a list of translations; use those, else match the transforms
		if type(info[0]) == list:
			cmds.xform(ctrl_group, t = info[0], ws = True)
		else:
			cmds.matchTransform(ctrl_group, info[0])
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
		blend_attr(Attr): The attribute that will connect to the blendColors Blender attribute
		direct_conn_attrs(List:Attrs): A list of attributes that the blend_attr will directly connect to
		blend_conn_attrs(List:Attrs): A list of attributes that the outputR attribute of the blendColors will directly connect to
		instance(String): The instance the attribute is part of, typically left or right

	Returns:
		bcNode: The blendColors node
	'''
	# Create blend colors node
	bcNode = cmds.shadingNode("blendColors", asUtility = True, name = "{}_{}_blendColors".format(instance, blend_attr))

	# Set Color 1R, 1G & 1B to 0 and Color 2R, 2G, & 2B to 1
	for i in ["R", "G", "B"]:
		cmds.setAttr("{}.color1{}".format(bcNode, i), 0)
		cmds.setAttr("{}.color2{}".format(bcNode, i), 1)

	# Connect the blending attribute to the Blender attribute on the BC node
	cmds.connectAttr("{}".format(blend_attr), "{}.blender".format(bcNode))

	# Connect the blending attribute to the direct connection attributes
	for i in direct_conn_attrs:
		cmds.connectAttr("{}".format(blend_attr), "{}".format(i))

	# Connect the output R attribute from the blend colors node to the blend connection attributes
	for i in blend_conn_attrs:
		cmds.connectAttr("{}.outputR".format(bcNode), "{}".format(i))

	return bcNode


def re_orient_joints(joints=[], primary_orient="xzy", secondary_orient="zup", clean_zero=[]):
	'''
	Set the joint orientations for specified joints and zero out rotations and joint orientations on remaining floating
	joints
	Args:
		joints(List): Joints to re-orient
		orient(String): Preferred primary and secondary axes
		secondary_orient(String): Preferred seccondary axis orient/world direction
		clean_zero(List): Any joints that need to be zeroed out in rotations and joint orients
	Returns:
		None
	'''
	# Set the orientation and secondary axis orientation
	if joints:
		for i in joints:
			cmds.joint(i, edit=True, orientJoint=primary_orient,
					   secondaryAxisOrient=secondary_orient,
					   children=True,
					   zeroScaleOrient=True)

	# Zero out the rotations and joint orients on any specified joints
	if clean_zero:
		for i in clean_zero:
			cmds.setAttr("{}.jointOrient".format(i), 0, 0, 0)
			cmds.setAttr("{}.rotate".format(i), 0, 0, 0)


def text_to_hex(txt = ''):
	'''
	Converts unicode string to Hex encoding so the Maya type node can use it
	Args:
		txt(String): The text to encode

	Returns:
		A list of encoded characters
	'''
	out = []
	for c in txt:
		hx = c.encode('hex')
		out.append(hx)
	return ' '.join(out)

def make_curve_text(txt):
	'''
	Create curve text from Polygon TypeMesh for controls
	Args:
		txt(String): The text that will be turned into control curves

	Returns:
		TxtObjects(Mesh): The TypeMesh created from the text
	'''
	# A list to store all the newly created Geo.
	TxtObjects = []

	# Converting our text to Hex code so the Maya type node will read it.
	hx = text_to_hex(txt)

	# Creating the Polygon Type
	cmds.CreatePolygonType()
	# Getting the node name from selection because the CreatePolygonType() command doesn't actually return the node name.
	NewTextObj = cmds.ls(sl=True)[0]
	# Listing connections on the New Text object node to find the type node.
	TypeNode = cmds.listConnections(NewTextObj + ".message")

	# If we find it, Set attributes on it.
	if TypeNode:
		# Place any other preset settings in here.
		cmds.setAttr(TypeNode[0]+".textInput", hx, type="string")
	# Adding the New text object to the TxtObjects List
	TxtObjects.append(NewTextObj)

	return TxtObjects