import maya.cmds as cmds
import json
import os
import system.utils as utils

reload(utils)

# Create variables above the Class level that can be read on Class import
# This is also known as Attributes of a Class
class_name = 'Rig_Arm'
# layout_file = arm.json
num_joints = 3


class Rig_Arm:
    """docstring for ClassName"""

    def __init__(self):
        # Get the joint list from the arm json file
        data_path = os.environ["RDOJO_DATA"] + "data/rig/arm.json"
        # Use the read json function
        data = utils.readJson(data_path)
        # Load the json into a dictionary
        self.module_info = json.loads(data)
        '''NOTE: If we want to build the arm from some set of joints
        in the scene, we could overwrite self.module_info['positions']'''

        # Make new Dictionary to store information about the arm rig
        self.rig_info = {}

        # Check if we have a selection of joints to get new positions from
        if len(cmds.ls(sl=True, type='joint', )) == num_joints:
            sel = cmds.ls(sl=True, type='joint')
            positions = []
            for i in sel:
                positions.append(cmds.xform(i, q=True, ws=True, t=True))
            self.rig_info['positions'] = positions
        else:
            self.rig_info['positions'] = self.module_info['positions']

        '''Instead of using Else, we could just return a message saying the selection
        doesn't meet the requirements for an arm'''

        # Set a temporary variable to override the name of the side to determine Left or Right
        self.instance = "Left_"

    # Run rig_arm function
    # self.rig_arm()

    def rig_arm(self):
        cmds.select(cl=True)
        #################
        ##Create joints##
        #################

        # Create IK joints
        self.rig_info['ik_joints'] = utils.createJoint(self.module_info['ik_joints'], self.rig_info['positions'],
                                                       self.instance)
        cmds.select(cl=True)

        # Create FK joints
        self.rig_info['fk_joints'] = utils.createJoint(self.module_info['fk_joints'], self.rig_info['positions'],
                                                       self.instance)
        cmds.select(cl=True)

        # Create rig joints
        self.rig_info['rig_joints'] = utils.createJoint(self.module_info['rig_joints'], self.rig_info['positions'],
                                                        self.instance)
        cmds.select(cl=True)

        #################
        ##Create IK Rig##
        #################

        # 1st Step: Create IK Handle
        ikHandle_name = self.module_info['ik_controls'][1].replace('s_', self.instance)
        self.rig_info['ik_handle'] = cmds.ikHandle(n=ikHandle_name, sj=self.rig_info['ik_joints'][0],
                                                   ee=self.rig_info['ik_joints'][2], sol='ikRPsolver', p=2, w=1)

        # 2nd Step: Create IK control
        self.rig_info['ik_controls'] = \
        utils.createControl([[self.rig_info['positions'][2], self.module_info['ik_controls'][0]]])[0]

        # 3rd Step: Parent IK handle to the control
        cmds.parent(self.rig_info['ik_handle'][0], self.rig_info['ik_controls'][1])

        # Clear selection
        cmds.select(cl=True)

        # Create Pole vector for IK Handle

        # Store position for the pole vector
        pole_vector_position = utils.calculatePoleVectorPosition(
            [self.rig_info['ik_joints'][0], self.rig_info['ik_joints'][1], self.rig_info['ik_joints'][2]])

        # create pole vector control
        self.rig_info['pole_vector_control'] = \
        utils.createControl([[pole_vector_position, self.module_info['ik_controls'][2]]])[0]

        # Create Pole Vector Constraint
        cmds.poleVectorConstraint(self.rig_info['pole_vector_control'][1], self.rig_info['ik_handle'][0])

        # Orient constrain IK wrist joint to IK control
        cmds.orientConstraint(self.rig_info['ik_controls'][1], self.rig_info['ik_joints'][2], mo=True)

        # Make control arm settings to handled IK/FK switching
        self.rig_info['set_control'] = utils.createControl([[self.rig_info['positions'][2], 'control_settings']])[0]
        cmds.addAttr(self.rig_info['set_control'][1], longName='IK_FK', attributeType='enum', enumName='fk:ik',
                     keyable=True)

        #################
        ##Create FK Rig##
        #################

        # Create FK controls
        self.rig_info['fk_controls'] = utils.createControl(
            [[self.rig_info['positions'][0], self.module_info['fk_controls'][0].replace('s_', self.instance)],
             [self.rig_info['positions'][1], self.module_info['fk_controls'][1].replace('s_', self.instance)],
             [self.rig_info['positions'][2], self.module_info['fk_controls'][2].replace('s_', self.instance)]])
        cmds.select(cl=True)

        # Parent FK controls
        cmds.parent(self.rig_info['fk_controls'][0][0], self.rig_info['fk_controls'][1][1][0])
        cmds.parent(self.rig_info['fk_controls'][1][0], self.rig_info['fk_controls'][2][1][0])

        '''Connect IK and FK to rig joints'''
    # parent Constrain fk->ik->rig
    # cmds.parentConstraint('fk_shoulder_joint', 'ik_shoulder_joint', 'rig_shoulder_joint', maintainOffset=True, weight=1)
    # To switch between FK and IK, change values of the two attributes (FK shoulder joint and IK shoulder joint)
    # and set one or the other to zero


print("IT'S ALIIIIIIVE")