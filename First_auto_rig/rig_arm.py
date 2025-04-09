import maya.cmds as cmds
import json
import os
import system.utils as utils
reload(utils)

#Create variables above the Class level that can be read on Class import
#This is also known as Attributes of a Class
class_name = 'Rig_Arm'
#layout_file = arm.json
num_joints = 3


class Rig_Arm:
	"""docstring for ClassName"""
	
	def __init__(self):
		#Get the joint list from the arm json file
		data_path = os.environ["RDOJO_DATA"] + "data/rig/arm.json"
		#Use the read json function
		data = utils.readJson(data_path)
		#Load the json into a dictionary
		self.module_info = json.loads(data)
		'''NOTE: If we want to build the arm from some set of joints
		in the scene, we could overwrite self.module_info['positions']'''
		
		#Make new Dictionary to store information about the arm rig
		self.rig_info = {}

		#Check if we have a selection of joints to get new positions from
		if len(cmds.ls(sl = True, type = 'joint',)) == num_joints:
			sel = cmds.ls(sl = True, type = 'joint')
			positions = []
			for i in sel:
				positions.append(cmds.xform(i, q = True, ws = True, t = True))
			self.rig_info['positions'] = positions
		else:
			self.rig_info['positions'] = self.module_info['positions']
		
		'''Instead of using Else, we could just return a message saying the selection
		doesn't meet the requirements for an arm'''

		#Set a temporary variable to override the name of the side to determine Left or Right
		self.instance = "Left_"

		#Run rig_arm function
		#self.rig_arm()


	def rig_arm(self):
		cmds.select(cl=True)
		#################
		##Create joints##
		#################

		#Create IK joints
		self.rig_info['ik_joints'] = utils.createJoint(self.module_info['ik_joints'], self.rig_info['positions'], self.instance)
		# Orient the joint with X down and Z facing positive in world Z
		utils.re_orient_joints(joints=[self.rig_info['ik_joints'][0]],
							   primary_orient=self.module_info["joint_orientation"],
							   secondary_orient=self.module_info["secondary_axis_orient"],
							   clean_zero=[self.rig_info['ik_joints'][-1]])
		cmds.select(cl=True)

		#Create FK joints
		self.rig_info['fk_joints'] =utils.createJoint(self.module_info['fk_joints'], self.rig_info['positions'], self.instance)
		# Orient the joint with X down and Z facing positive in world Z
		utils.re_orient_joints(joints=[self.rig_info['fk_joints'][0]],
							   primary_orient=self.module_info["joint_orientation"],
							   secondary_orient=self.module_info["secondary_axis_orient"],
							   clean_zero=[self.rig_info['fk_joints'][-1]])
		cmds.select(cl=True)

		#Create rig joints
		self.rig_info['rig_joints'] =utils.createJoint(self.module_info['rig_joints'], self.rig_info['positions'], self.instance)
		# Orient the joint with X down and Z facing positive in world Z
		utils.re_orient_joints(joints=[self.rig_info['rig_joints'][0]],
							   primary_orient=self.module_info["joint_orientation"],
							   secondary_orient=self.module_info["secondary_axis_orient"],
							   clean_zero=[self.rig_info['rig_joints'][-1]])
		cmds.select(cl=True)


		#################
		##Create IK Rig##
		#################


		#1st Step: Create IK Handle
		ikHandle_name = self.module_info['ik_controls'][1].replace('s_',self.instance)
		self.rig_info['ik_handle'] = cmds.ikHandle(n=ikHandle_name, sj=self.rig_info['ik_joints'][0], ee=self.rig_info['ik_joints'][2], sol='ikRPsolver',p = 2, w = 1)

		#2nd Step: Create IK control
		self.rig_info['ik_controls'] = utils.createControl([[self.rig_info['positions'][2], self.module_info['ik_controls'][0].replace("s_", self.instance)]])[0]
		# Match the controls position to the joint it's controlling
		cmds.matchTransform(self.rig_info['ik_controls'][0], self.rig_info['ik_joints'][-1])

		#3rd Step: Parent IK handle to the control
		cmds.parent(self.rig_info['ik_handle'][0], self.rig_info['ik_controls'][1])

		#Clear selection
		cmds.select(cl=True)

		
		#Create Pole vector for IK Handle

		#Store position for the pole vector
		pole_vector_position = utils.calculatePoleVectorPosition([self.rig_info['ik_joints'][0],self.rig_info['ik_joints'][1],self.rig_info['ik_joints'][2]])
		
		#create pole vector control
		self.rig_info['pole_vector_control'] = utils.createControl([[pole_vector_position, self.module_info['ik_controls'][2].replace("s_", self.instance)]], flip=False)[0]

		#Create Pole Vector Constraint
		cmds.poleVectorConstraint(self.rig_info['pole_vector_control'][1], self.rig_info['ik_handle'][0])

		#Orient constrain IK wrist joint to IK control
		cmds.orientConstraint(self.rig_info['ik_controls'][1], self.rig_info['ik_joints'][2], mo = True)

		#Make control arm settings to handle IK/FK switching
		self.rig_info['set_control'] = utils.createControl([[self.rig_info['positions'][2], '{}arm_settings_CTRL'.format(self.instance)]], flip=False)[0]
		cmds.addAttr(self.rig_info['set_control'][1], longName = 'IK_FK', attributeType = 'double', keyable = True,
					 															min = 0,
					 															max = 1,
					 															defaultValue = 0)


		#################
		##Create FK Rig##
		#################

		#Create FK controls
		self.rig_info['fk_controls'] = utils.createControl([[self.rig_info['fk_joints'][0], self.module_info['fk_controls'][0].replace('s_',self.instance)],
															[self.rig_info['fk_joints'][1], self.module_info['fk_controls'][1].replace('s_',self.instance)],
															[self.rig_info['fk_joints'][2], self.module_info['fk_controls'][2].replace('s_',self.instance)]])
		cmds.select(cl=True)

		#Parent FK controls
		cmds.parent(self.rig_info['fk_controls'][1][0],self.rig_info['fk_controls'][0][1][0])
		cmds.parent(self.rig_info['fk_controls'][2][0],self.rig_info['fk_controls'][1][1][0])

		# Constrain FK controls to the FK joint chain
		for index, ctrl in enumerate(self.rig_info["fk_controls"]):
			cmds.parentConstraint(ctrl[1][0], self.rig_info["fk_joints"][index])


		# Constrain IK and FK rigs to rig joints (IK chain first)
		ik_fk_constraints = []
		for i, jnt in enumerate(self.rig_info['ik_joints']):
			constraint = cmds.parentConstraint(jnt, self.rig_info['fk_joints'][i], self.rig_info["rig_joints"][i],
								  													maintainOffset = 1,
								  													weight = 1)
			ik_fk_constraints.append(constraint)
		# Save the constraints to a list
		self.rig_info["ik_fk_constraints"] = ik_fk_constraints

		# Save out the attributes to be used for the ik/fk switching
		ik_fk_switch_attr = "{}.IK_FK".format(self.rig_info['set_control'][1][0])

		fk_attrs = ["{}.{}".format(i[0], cmds.listAttr(i)[-1]) for i in ik_fk_constraints]
		fk_visibility_attrs = ["{}.visibility".format(i[1][0]) for i in self.rig_info["fk_controls"]]
		fk_attrs.extend(fk_visibility_attrs)

		ik_attrs = ["{}.{}".format(i[0], cmds.listAttr(i)[-2]) for i in ik_fk_constraints]
		ik_visibility_attrs = ["{}.visibility".format(self.rig_info["ik_controls"][1][0])]
		ik_visibility_attrs.append("{}.visibility".format(self.rig_info["pole_vector_control"][1][0]))
		ik_attrs.extend(ik_visibility_attrs)

		# Connect the relevant attributes for IK/FK switching
		utils.connectBlendColors(ik_fk_switch_attr, fk_attrs, ik_attrs, instance=self.instance)



print("IT'S ALIIIIIIVE")