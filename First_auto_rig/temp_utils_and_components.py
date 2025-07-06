import pymel.core as pm

rig_chains = ["_BLEND", "_FK", "_IK", "_REVERSE"]
type = ["_JNT", "_CTRL", "_GRP"]
spine_chain = pm.selected()


def split_name(string, split_string, insert):
    '''
    Returns a new string with the desired characters inserted, meant for updating names of joints and control components
    Args:
        string: The string for the change to base from
        split_string: the string where the split will occur
        insert: the new value intended to be inserted
    '''
    split_index = string.index(split_string)
    new_name = "{}{}{}".format(string[:split_index], insert, string[split_index:])

    return new_name

def new_joint_chain(type_name, existing_chain=None):

    if existing_chain:
        pm.duplicate(existing_chain)

def list_joint_chain(base_joint):
    '''
    Returns a list of all joints in a chain from parent to child
    Args:
        base_joint: The top joint in a chain
    '''
    full_chain = base_joint.listRelatives(allDescendents=True, type="joint")
    full_chain.append(base_joint)
    full_chain.reverse()

    return full_chain

def fk_ik_ribbon(joint_chain):
    '''
    Needs:
        A duplicate of the spine chain for the FK
        A top and bottom control joint
        A lofted surface equal to the spine chain
        (Manual) A rebuilt surface with double the resolution of the spine chain/surface
        (Manual) Hair follicles on the rebuilt surface
            U count the same number of edges as the surface
            Edge bounded
            Static
    Returns:

    '''

    if joint_chain:
        # Duplicate the existing bind chain and save the full chain for the FK
        fk_joint_chain = pm.duplicate(joint_chain)
        fk_joint_chain = list_joint_chain(fk_joint_chain[0])
        # Rename joints in chain
        for i in fk_joint_chain:
            split_name(i, "_JNT", "_FK")

        # Create a chest control joint
        chest_ctrl_joint = pm.duplicate(fk_joint_chain)
        split_name(chest_ctrl_joint, "_JNT", "_CTRL")
        # Unparent the hip control joint from the chest control joint
        hip_ctrl_joint = list_joint_chain(chest_ctrl_joint[0])[-1]
        pm.parent(hip_ctrl_joint, world=True)
        # Delete remaining joints
        pm.delete(list_joint_chain(chest_ctrl_joint[0])[1])

def loft_surface(joint_chain):
    '''
    Create a lofted surface from a joint chain
    Args:
        joint_chain: The base joint of the chain for the curve to base its positions from

    Returns:

    '''
    # Get the current joint chain and save its positions to a list
    if joint_chain:
        full_joint_chain = list_joint_chain(joint_chain)

    positions = [i.getTranslation(world=True) for i in full_joint_chain]

    # Create the curve with CVs at the positions of the control joints
    curve_name = joint_chain.name()
    curve_name.replace("JNT", "CRV")
    curve = pm.curve(point=positions, n=curve_name)

    # Duplicate, move the curves to setup the loft
    curve_dupe_01 = curve[0].duplicate()
    curve_dupe_01[0].setTranslation([6, 0, 0])
    curve_dupe_02 = curve[0].duplicate()
    curve_dupe_02[0].setTranslation([-6, 0, 0])

    # Loft the curves to create the surface
    surface, loft_constructor = pm.loft(curve_dupe_02[0], curve_dupe_01[0])


def scale_ribbon_squash_and_stretch(curve, joints):
    '''
    Needs:
        Curve
        1 curveinfo and 3 multiply divide nodes
    Returns:

    '''
    curve_shape = curve.getShape()
    curve_info = pm.createNode("curveInfo")
    curve_shape.worldMatrix >> curve_info.input1X

    # Create the multiply divide nodes and make the connections from the curve and to the bind joints
    curve_diff = pm.createNode("multiplyDivide")
    curve_diff.operation.set(2)
    curve_diff.input2X.set(curve_info.arcLength)
    curve_info.arcLength >> curve_diff.input1X
    # Aim axis is assumed to be X
    for i in joints:
        curve_diff.outputX >> i.scaleX

    curve_sqrt_power = pm.createNode("multiplyDivide")
    curve_sqrt_power.operation.set(3)
    curve_sqrt_power.input2X.set(0.5)
    curve_diff.outputX >> curve_sqrt_power.input1X

    curve_sqrt_invert = pm.createNode("multiplyDivide")
    curve_sqrt_invert.operation.set(2)
    curve_sqrt_invert.input1X.set(1)
    curve_sqrt_power.output1X >> curve_sqrt_invert.input2X
    # Horizontal scale axes assumed to be Y and Z
    for i in joints:
        curve_sqrt_invert.outputX >> i.scaleY
        curve_sqrt_invert.outputX >> i.scaleZ


def create_fk_rig(base_joint, fk_ctrl_size=30):
    '''

    Args:
        base_joint:

    Returns:

    '''
    # Save IK chain names
    fk_chain_names = []
    for i in list_joint_chain(base_joint[0]):
        if "_JNT" in i.name():
            fk_chain_names.append(split_name(i, "_JNT", rig_chains[1]))
        else:
            fk_chain_names.append("{}{}{}".format(i, rig_chains[1], type[0]))

    # Create FK chain
    fk_chain = list_joint_chain(pm.duplicate(base_joint[0], renameChildren=True)[0])
    pm.parent(fk_chain[0], world=True)

    # Rename the FK chain
    print(fk_chain_names)
    for i, e in enumerate(fk_chain):
        e.rename(fk_chain_names[i])

    # Create controls for the FK chain and parent their zero transforms to each other
    fk_grps = []
    for i in fk_chain:
        new_trans = i.getTranslation(space="world")
        new_rot = i.getRotation(space="world")

        circ = pm.circle(name="{}".format(i.name().replace("_JNT", type[1])))
        circ[1].normalX.set(90)
        circ[1].radius.set(fk_ctrl_size)

        circ[0].setTranslation(new_trans, space="world")
        circ[0].setRotation(new_rot, space="world")
        # Create zero groups
        grp = pm.group(empty=True, name="{}{}".format(circ[0].name(), "_ZERO_GRP"))
        grp.setTranslation(new_trans, space="world")
        grp.setRotation(new_rot, space="world")
        fk_grps.append(grp)
        pm.parent(circ[0], grp)

        # Create the control hierarchy
        for i in range(len(fk_grps) - 1):
            pm.parent(fk_grps[i + 1], fk_grps[i].getChildren())

        # Constrain the FK chain to the controls
        for i, e in enumerate(fk_grps):
            pm.parentConstraint(e.getChildren(), fk_chain[i])

    return fk_grps, fk_chain

def create_ik_rig(base_joint):
    # Save IK chain names
    ik_chain_names = []
    for i in list_joint_chain(base_joint[0]):
        if "_JNT" in i.name():
            ik_chain_names.append(split_name(i, "_JNT", rig_chains[2]))
        else:
            ik_chain_names.append("{}{}{}".format(i, rig_chains[2], type[0]))

    # Create IK chain
    ik_chain = list_joint_chain(pm.duplicate(base_joint[0], renameChildren=True)[0])

    # Rename the IK chain
    for i, e in enumerate(ik_chain):
        e.rename(ik_chain_names[i])

    # Create IK control and parent it to the zero transform
    new_trans = ik_chain[-1].getTranslation(space="world")
    new_rot = ik_chain[-1].getRotation(space="world")

    ik_ctrl = pm.curve(d=1, p=[(0.5, 0.5, 0.5), (0.5, -0.5, 0.5), (-0.5, -0.5, 0.5), (-0.5, 0.5, 0.5), (0.5, 0.5, 0.5),
                               (0.5, 0.5, -0.5), (0.5, -0.5, -0.5), (0.5, -0.5, 0.5), (-0.5, -0.5, 0.5),
                               (-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (-0.5, 0.5, -0.5),
                               (-0.5, -0.5, -0.5), (-0.5, 0.5, -0.5), (-0.5, 0.5, 0.5)],
                       k=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])
    ik_ctrl.rename(ik_chain[-1].name().replace("_JNT", type[1]))

    ik_ctrl.scale.set([30, 30, 30])
    pm.makeIdentity(ik_ctrl, t=0, r=0, s=1, apply=True)

    ik_ctrl.setTranslation(new_trans, space="world")
    #ik_ctrl.setRotation(new_rot, space="world")
    # Create zero group
    ik_grp = pm.group(empty=True, name="{}{}".format(ik_ctrl.name(), "_ZERO_GRP"))
    ik_grp.setTranslation(new_trans, space="world")
    #ik_grp.setRotation(new_rot, space="world")
    pm.parent(ik_ctrl, ik_grp)
    # Create ik handle
    ik_handle = pm.ikHandle(sj=ik_chain[0], ee=ik_chain[-1], sol='ikRPsolver', p=2, w=1,
                            name="{}{}".format(ik_chain[-1].name(), "_ikHandle"))[0]

    # Create pole vector
    pv_ctrl = pm.curve(d=1, p=[(0, 3.21, 0), (0, 2.96, 1.23), (0, 2.27, 2.27), (0, 1.23, 2.96), (0, 0, 3.21),
                               (0, -1.23, 2.96), (0, -2.27, 2.27), (0, -2.97, 1.23), (0, -3.21, 0), (0, -2.96, -1.23),
                               (0, -2.27, -2.27), (0, -1.23, -2.96), (0, 0, -3.21), (0, 1.23, -2.96), (0, 2.27, -2.27),
                               (0, 2.96, -1.23), (0, 3.21, 0), (-0.87, 2.96, 0.97), (-1.60, 2.27, 1.60),
                               (-2.09, 1.23, 2.09), (-2.27, 0, 2.27), (-2.09, -1.23, 2.09), (-1.60, -2.27, 1.60),
                               (-0.87, -2.96, 0.87), (0, -3.21, 0), (0.87, -2.97, -0.87), (1.60, -2.27, -1.60),
                               (2.09, -1.23, -2.09), (2.27, 0, -2.27), (2.09, 1.23, -2.09), (1.60, 2.27, -1.60),
                               (0.87, 2.86, -0.87), (0, 3.21, 0), (-1.23, 2.97, 0), (-2.27, 2.27, 0), (-2.97, 1.23, 0),
                               (-3.21, 0, 0), (-2.97, -1.23, 0), (-2.27, -2.27, 0), (-1.23, -2.96, 0), (0, -3.21, 0),
                               (1.23, -2.97, 0), (2.27, -2.27, 0), (2.97, -1.23, 0), (3.21, 0, 0), (2.97, 1.23, 0),
                               (2.27, 2.27, 0), (1.23, 29.7, 0), (0, 3.21, 0), (-0.87, 2.97, -0.87),
                               (-1.60, 2.27, -1.60), (-2.09, 1.23, -2.09), (-2.27, 0, -2.27), (-2.09, -1.23, -2.09),
                               (-1.60, -2.27, -1.60), (-0.87, -2.96, -0.87), (0, -3.21, 0), (0.87, -2.97, 0.87),
                               (1.60, -2.27, 1.60), (2.09, -1.23, 2.09), (2.27, 0, 2.27), (2.09, 1.23, 2.09),
                               (1.60, 2.27, 1.60), (0.87, 2.97, 0.87), (0, 3.21, 0), (1.23, 2.97, 0), (2.27, 2.27, 0),
                               (2.97, 1.23, 0), (3.21, 0, 0), (2.27, 0, 2.27), (0, 0, 3.21), (-2.27, 0, 2.27),
                               (-3.21, 0, 0), (-2.27, 0, -2.27), (0, 0, -3.21), (2.27, 0, -2.27), (3.21, 0, 0),
                               (2.27, 0, 2.27), (0, 0, 3.21)], name=ik_ctrl.name().replace("_CTRL", "_PV"))
    pm.select(pv_ctrl + ".cv[47]")
    pm.delete()
    pm.select(clear=True)

    pv_ctrl.scale.set([7, 7, 7])
    pm.makeIdentity(pv_ctrl, t=0, r=0, s=1, apply=True)

    # Create zero group
    pv_grp = pm.group(empty=True, name="{}{}".format(pv_ctrl.name(), "_ZERO_GRP"))
    pm.parent(pv_ctrl, pv_grp)
    # Move the pole vector
    pv_position = calculatePoleVectorPosition(ik_chain, pv_distance=10)
    pv_grp.setTranslation(pv_position, space="world")
    # Constrain the pole vector and ik wrist to their controls
    pm.poleVectorConstraint(pv_ctrl, ik_handle)
    pm.orientConstraint(ik_ctrl, ik_chain[-1])

    # Parent the ik handle to the ik control
    pm.parent(ik_handle, ik_ctrl)

    # Create IK control group
    ik_ctrl_grp = pm.group(empty=True, name="{}{}".format(ik_ctrl.name().split(type[1])[0], type[2]))
    ik_ctrl_grp.setTranslation(ik_chain[0].getTranslation(space="world"), space="world")
    ik_ctrl_grp.setRotation(ik_chain[0].getRotation(space="world"), space="world")

    pm.parent(ik_grp, ik_ctrl_grp)
    pm.parent(pv_grp, ik_ctrl_grp)


    return ik_chain, ik_ctrl, pv_grp


def fk_ik_hinge(base_joint):
    '''

    Args:
        base_joint:

    Returns:
        FK Chain, IK Chain, IK Handle, Blend Chain
    '''

    # Save blend chain names
    blend_chain_names = []
    for i in list_joint_chain(base_joint[0]):
        if "_JNT" in i.name():
            blend_chain_names.append(split_name(i, "_JNT", rig_chains[0]))
        else:
            blend_chain_names.append("{}{}{}".format(i, rig_chains[0], type[0]))

    # Create blend chain
    blend_chain = list_joint_chain(pm.duplicate(base_joint[0], renameChildren=True)[0])

    # Rename the blend chain
    for i, e in enumerate(blend_chain):
        e.rename(blend_chain_names[i])

    blend_ctrl_points = [[1, 0, -1.0], [1, 0, 1], [0, 1.391788, 0], [1, 0, -1.0], [-1.0, 0, -1.0], [0, 1.391788, 0],
                         [1, 0, 1], [-1.0, 0, 1], [0, 1.391788, 0], [-1.0, 0, -1.0], [-1.0, 0, 1]]
    # Create blend control
    blend_ctrl = pm.curve(degree=1, point=blend_ctrl_points, name="{}{}{}".format(blend_chain[-1].name(),
                                                                                  rig_chains[0], type[1]))
    blend_ctrl.scale.set([15, 15, 15])
    pm.makeIdentity(blend_ctrl, t=0, r=0, s=1, apply=True)

    # Create zero group
    blend_grp = pm.group(empty=True, name="{}{}".format(blend_ctrl.name(), "_ZERO_GRP"))
    pm.parent(blend_ctrl, blend_grp)

    # Move the blend control
    new_trans = blend_chain[-1].getTranslation(space="world")
    if new_trans[0] > 0:
        new_trans[0] += 60
    else:
        new_trans[0] -= 60

    blend_grp.setTranslation(new_trans, space="world")
    pm.pointConstraint(blend_chain[-1], blend_grp, maintainOffset=True)

    # Add FKIK attribute to the blend control
    blend_ctrl.addAttr("FK_IK", attributeType="double", min=0, max=1, defaultValue=1)
    blend_ctrl.FK_IK.set(keyable=True)

    # Create FK chain
    fk_grps, fk_chain = create_fk_rig(base_joint)

    # Create IK chain
    ik_chain, ik_ctrl, pv_grp = create_ik_rig(base_joint)

    # Parent constrain the blend chain to the ik and fk chains
    blend_constraints = [pm.parentConstraint(fk_chain[i], ik_chain[i], e, maintainOffset=True) for i, e in enumerate(blend_chain)]
    # Create the reverse node
    reverse_node = pm.createNode("reverse", name = "{}_reverse".format(blend_ctrl))
    # Connect blend control FK IK attribute to reverse node
    blend_ctrl.FK_IK >> reverse_node.inputX
    # Connect blend control FK IK attribute to the IK attribute on the parent constraints
    # Connect blend control reverse node to the FK attribute on the parent constraints
    for i, e in enumerate(blend_constraints):
        blend_ctrl.FK_IK >> e.listAttr()[-1]
        reverse_node.outputX >> e.listAttr()[-2]
        reverse_node.outputX >> fk_grps[i].getChildren()[0].visibility

    blend_ctrl.FK_IK >> ik_ctrl.visibility
    blend_ctrl.FK_IK >> pv_grp.getChildren()[0].visibility

    # Create ctrl joint group
    ctrl_jnt_grp_trans = base_joint[0].getTranslation(space="world")
    ctrl_jnt_grp_rot = base_joint[0].getRotation(space="world")

    ctrl_jnt_grp = pm.group(empty=True, name="{}{}".format(base_joint[0].name(), type[2]))
    ctrl_jnt_grp.setTranslation(ctrl_jnt_grp_trans, space="world")
    ctrl_jnt_grp.setRotation(ctrl_jnt_grp_rot, space="world")

    pm.parent(blend_chain[0], ctrl_jnt_grp)
    pm.parent(fk_chain[0], ctrl_jnt_grp)
    pm.parent(ik_chain[0], ctrl_jnt_grp)

    return ik_chain, fk_chain, blend_chain


def calculatePoleVectorPosition(joints, pv_distance=5):

    if isinstance(joints, list) and len(joints) == 3:
        startVector = joints[0].getTranslation(space="world")
        midVector = joints[1].getTranslation(space="world")
        endVector = joints[2].getTranslation(space="world")

        startEnd = endVector - startVector
        startMid = midVector - startVector
        dotP = startMid.dot(startEnd)
        proj = float(dotP) / float(startEnd.length())
        startEndN = startEnd.normal()
        projV = startEndN * proj
        arrowV = startMid - projV
        arrowV *= pv_distance
        finalV = arrowV + midVector
    else:
        raise RuntimeError("Please select three joints in a chain")

    return (finalV)


def parse_curve_points(curve_string):
    # Split each point in the string
    p_split = curve_string.split("-p")
    # Remove the knot values
    p_split[-1] = p_split[-1].split("-k", 1)[0]
    # Remove the empty space at the front
    p_split.remove("")
    # Split each point into lists
    space_split = [i.split() for i in p_split]

    for i, e in enumerate(space_split):
        for each, point in enumerate(e):
            if len(point) == 1:
                space_split[i][each] = int(point)
            else:
                space_split[i][each] = float(point)

    return space_split


def addStretchyIK(base_joint):
    '''
    Create stretchy IK using the side lengths of the triangle made up of the IK joints
    Args:
        ctrl_joints: List of control joints
    '''
    ctrl_joints = list_joint_chain(base_joint)

    # Get the upperarm length
    upperarm_length = ctrl_joints[1].getTranslation()[0]
    # Get the lowerarm length
    lowerarm_length = ctrl_joints[2].getTranslation()[0]

    # Add the shoulder and elbow joint length for the arm length
    arm_length = upperarm_length+lowerarm_length

    # Create locators at the shoulder and wrist
    start_pos = ctrl_joints[0].getTranslation(space="world")
    end_pos = ctrl_joints[-1].getTranslation(space="world")

    start_loc_name = split_name(ctrl_joints[0].name(), type[0], "_stretch")
    end_loc_name = split_name(ctrl_joints[-1].name(), type[0], "_stretch")

    start_loc = pm.spaceLocator(name=start_loc_name)
    start_loc.setTranslation(start_pos)
    end_loc = pm.spaceLocator(name=end_loc_name)
    end_loc.setTranslation(end_pos)

    # Create distance node for arm distance (distance between the shoulder and wrist)
    arm_distance = pm.createNode("distanceBetween", name="{}_{}".format(ctrl_joints[-1].name(), "distanceBetween"),
                                 skipSelect=True)

    # Connect top and bottom locators to Distance node
    start_loc.getShape().worldPosition >> arm_distance.point1
    end_loc.getShape().worldPosition >> arm_distance.point2

    # Create a multiplyDivide node (set to divide) and name it distance_divider
    arm_divider = pm.createNode("multiplyDivide", name = "{}_{}".format(arm_distance.name(), "divide"))
    arm_divider.operation.set(2)
        # Set input2 of the divider to the arm length
    arm_divider.input2X.set(upperarm_length + lowerarm_length)
        # Plug the distance from the arm distance dimension node into the input1 of the divider
    arm_distance.distance >> arm_divider.input1X

    # Create two multDoubleLiner nodes, one for the upperarm and one for the lowerarm
    upperarm_multiplier = pm.createNode("multDoubleLinear", name="{}_{}".format(ctrl_joints[0], "multDoubleLinear"))
    lowerarm_multiplier = pm.createNode("multDoubleLinear", name="{}_{}".format(ctrl_joints[1], "multDoubleLinear"))
        # Plug the output from the divider into input1 of both multDoubleLinear nodes
    arm_divider.outputX >> upperarm_multiplier.input1
    arm_divider.outputX >> lowerarm_multiplier.input1
        # Set input2 of the upparm multipler to the length of the upperarm (elbow X value)
    upperarm_multiplier.input2.set(upperarm_length)
        # Set input2 of the lowerarm multipler to the length of the lowerarm (wrist X value)
    lowerarm_multiplier.input2.set(lowerarm_length)

    # Create two condition nodes, one for the upperarm and one for the lowerarm
    upperarm_condition = pm.createNode("condition", name=ctrl_joints[0].name())
    upperarm_condition.operation.set(2)
    lowerarm_condition = pm.createNode("condition", name=ctrl_joints[1].name())
    lowerarm_condition.operation.set(2)
        # Plug the distance from the arm distance into FirstTerm of both condition nodes
    arm_distance.distance >> upperarm_condition.firstTerm
    arm_distance.distance >> lowerarm_condition.firstTerm
        # Plug the output from the arm multipliers into ColorIfTrue of both condition nodes
    upperarm_multiplier.output >> upperarm_condition.colorIfTrueR
    lowerarm_multiplier.output >> lowerarm_condition.colorIfTrueR
        # Set the SecondTerm of the both condition nodes to the arm length
    upperarm_condition.secondTerm.set(upperarm_length + lowerarm_length)
    lowerarm_condition.secondTerm.set(upperarm_length + lowerarm_length)
        # Set the ColorIfFalse of the upperarm condition node to the length of the upperarm (elbow X)
    upperarm_condition.colorIfFalseR.set(upperarm_length)
        # Set the ColorIfFalse of the lowerarm condition node to the length of the lowerarm (wrist X)
    lowerarm_condition.colorIfFalseR.set(lowerarm_length)
    # Plug the OutColorR of the upperarm condition node into the translateX of the elbow joint
    #upperarm_condition.outColorR >> ctrl_joints[1].translateX
    # Plug the OutColorR of the lowerarm condition node into the translateX of the wrist joint
    #lowerarm_condition.outColorR >> ctrl_joints[2].translateX

    # Get the ik handle and the ik control before re-parenting
    ik_handle = ctrl_joints[0].listConnections(type="ikHandle")[0]
    ik_ctrl = ik_handle.getParent()
    # Parent the IK handle under the 2nd locator
    pm.parent(ik_handle, end_loc)
    # Parent the 2nd locator under the IK control
    pm.parent(end_loc, ik_ctrl)

    # Add stretchy IK attribute to ik control
    ik_ctrl.addAttr("Stretchy_IK", attributeType="double", min=0, max=1, defaultValue=0)
    ik_ctrl.Stretchy_IK.set(keyable=True)
    # Create blendColors node
    stretchy_blendColors = pm.createNode("blendColors", name="{}_{}".format(ik_ctrl.name(), "blendColors"))
        # Set the default joint lengths to the color2 attributes
    stretchy_blendColors.color2R.set(upperarm_length)
    stretchy_blendColors.color2G.set(lowerarm_length)
        # Connect the stretchy IK attribute and condition nodes to the blendColors node
    ik_ctrl.Stretchy_IK >> stretchy_blendColors.blender
    upperarm_condition.outColorR >> stretchy_blendColors.color1R
    lowerarm_condition.outColorR >> stretchy_blendColors.color1G
        # Connect the blendColors node outputs to the ik joints
    stretchy_blendColors.outputR >> ctrl_joints[1].translateX
    stretchy_blendColors.outputG >> ctrl_joints[2].translateX

def save_duplicate_chain_names(base_joint):
    pass
    chain_names = [i.name() for i in list_joint_chain(base_joint[0])]
    for i in chain_names:
        if "_JNT" in i:
            i = split_name(i, "_JNT", rig_chains[1])
        else:
            i = "{}{}{}".format(i.name(), rig_chains[1], type[0])


def create_reverse_foot(ankle_ik_joint, ankle_ik_ctrl, inner_offset=30, outer_offset=30, toetip_offset=1, heel_offset=64):
    '''
    Create a reverse foot setup on a three joint ik foot
    Args:
        ankle_ik_joint:
        ankle_ik_ctrl:
        inner_offset:
        outer_offset:
        toetip_offset:
        heel_offset:

    Returns:
        None
    '''
    # Save prefix name
    if "_" in ankle_ik_joint[0].name():
        prefix_name = ankle_ik_joint[0].name().split("_")[0]
    else:
        prefix_name = "_"

    # Save foot chain and joint positions
    ik_foot_chain = list_joint_chain(ankle_ik_joint[0])

    toe_pos = ik_foot_chain[-1].getTranslation(space="world")
    ball_pos = ik_foot_chain[1].getTranslation(space="world")
    ankle_pos = ik_foot_chain[0].getTranslation(space="world")

    in_bank_pos = [i for i in ball_pos]
    in_bank_pos[1] -= 10
    in_bank_pos[0] -= inner_offset

    out_bank_pos = [i for i in ball_pos]
    out_bank_pos[1] -= 10
    out_bank_pos[0] += outer_offset

    heel_pos = [i for i in ball_pos]
    heel_pos[1] -= 10
    heel_pos[-1] -= heel_offset

    toetip_pos = [i for i in toe_pos]
    toetip_pos[1] -= 10
    toetip_pos[-1] += toetip_offset

    in_bank_grp = pm.group(empty=True, name="{}_inner_bank{}".format(prefix_name, type[2]))
    in_bank_grp.setTranslation(in_bank_pos, space="world")

    out_bank_grp = pm.group(empty=True, name="{}_outer_bank{}".format(prefix_name, type[2]))
    out_bank_grp.setTranslation(out_bank_pos, space="world")

    toe_tap_grp = pm.group(empty=True, name="{}_toe_tap{}".format(prefix_name, type[2]))
    toe_tap_grp.setTranslation(ball_pos, space="world")

    pm.parent(out_bank_grp, in_bank_grp)

    # Save reverse foot chain positions
    reverse_foot_chain_pos = []
    reverse_foot_chain_pos.append(heel_pos)
    reverse_foot_chain_pos.append(toetip_pos)
    reverse_foot_chain_pos.append(ball_pos)
    reverse_foot_chain_pos.append(ankle_pos)

    # Save reverse foot chain names
    reverse_foot_chain_names = []
    for i in ik_foot_chain:
        if "_IK" in i.name():
            reverse_foot_chain_names.append(i.name().replace("_IK", rig_chains[3]))
        else:
            reverse_foot_chain_names.append("{}_{}{}{}".format(prefix_name, i, rig_chains[3], type[0]))
    reverse_foot_chain_names.append("{}_{}{}{}".format(prefix_name, "heel", rig_chains[3], type[0]))
    reverse_foot_chain_names.reverse()

    # Create reverse foot chain
    reverse_foot_chain = [pm.joint() for i in range(4)]
    # Move the reverse joints into place
    for i, e in enumerate(reverse_foot_chain):
        e.setTranslation(reverse_foot_chain_pos[i], space="world")

    # Rename the reverse joint chain
    for i, e in enumerate(reverse_foot_chain):
        e.rename(reverse_foot_chain_names[i])

    # Parent the reverse joint chain
    for i in range(len(reverse_foot_chain)-1):
        pm.parent(reverse_foot_chain[i+1], reverse_foot_chain[i])

    ankle_ik_handle = [i for i in ankle_ik_ctrl[0].listRelatives(children=True, allDescendents=True) if i.nodeType() == "ikHandle"][0]
    ball_ik_handle = pm.ikHandle(name="{}_ball_ikHandle".format(prefix_name), startJoint=ik_foot_chain[0],
                                 endEffector=ik_foot_chain[1], solver="ikSCsolver", priority=2, weight=1)[0]
    toe_ik_handle = pm.ikHandle(name="{}_toe_ikHandle".format(prefix_name), startJoint=ik_foot_chain[1],
                                endEffector=ik_foot_chain[2], solver="ikSCsolver", priority=2, weight=1)[0]

    pm.parent(ankle_ik_handle, reverse_foot_chain[-1])
    pm.parent(ball_ik_handle, reverse_foot_chain[-2])
    pm.parent(toe_ik_handle, toe_tap_grp)
    pm.parent(toe_tap_grp, reverse_foot_chain[-3])

    pm.parent(in_bank_grp, ankle_ik_ctrl[0])

    ankle_ik_ctrl[0].addAttr("Heel_Twist", attributeType="double", defaultValue=0)
    ankle_ik_ctrl[0].Heel_Twist.set(keyable=True)
    ankle_ik_ctrl[0].addAttr("Heel_Roll", attributeType="double", defaultValue=0)
    ankle_ik_ctrl[0].Heel_Roll.set(keyable=True)
    ankle_ik_ctrl[0].addAttr("Toe_Twist", attributeType="double", defaultValue=0)
    ankle_ik_ctrl[0].Toe_Twist.set(keyable=True)
    ankle_ik_ctrl[0].addAttr("Toe_Roll", attributeType="double", defaultValue=0)
    ankle_ik_ctrl[0].Toe_Roll.set(keyable=True)
    ankle_ik_ctrl[0].addAttr("Ball_Roll", attributeType="double", defaultValue=0)
    ankle_ik_ctrl[0].Ball_Roll.set(keyable=True)
    ankle_ik_ctrl[0].addAttr("Toe_Tap", attributeType="double", defaultValue=0)
    ankle_ik_ctrl[0].Toe_Tap.set(keyable=True)
    ankle_ik_ctrl[0].addAttr("Bank", attributeType="double", defaultValue=0)
    ankle_ik_ctrl[0].Bank.set(keyable=True)


    ankle_ik_ctrl[0].Heel_Twist >> reverse_foot_chain[0].rotateY
    ankle_ik_ctrl[0].Heel_Roll >> reverse_foot_chain[0].rotateX
    ankle_ik_ctrl[0].Toe_Twist >> reverse_foot_chain[1].rotateY
    ankle_ik_ctrl[0].Toe_Roll >> reverse_foot_chain[1].rotateX
    ankle_ik_ctrl[0].Ball_Roll >> reverse_foot_chain[2].rotateX
    ankle_ik_ctrl[0].Toe_Tap >> toe_tap_grp.rotateX

    bank_condition_node = pm.createNode("condition", name="{}_bank_condition".format(prefix_name))
    bank_condition_node.operation.set(2)
    bank_condition_node.colorIfFalseR.set(0)

    in_invert_mult = pm.createNode("multDoubleLinear", name="{}_inner_bank_invert_mult".format(prefix_name))
    in_invert_mult.input2.set(-1)
    out_invert_mult = pm.createNode("multDoubleLinear", name="{}_outer_bank_invert_mult".format(prefix_name))
    out_invert_mult.input2.set(-1)

    ankle_ik_ctrl[0].Bank >> bank_condition_node.firstTerm
    ankle_ik_ctrl[0].Bank >> bank_condition_node.colorIfTrueR
    ankle_ik_ctrl[0].Bank >> bank_condition_node.colorIfFalseG
    bank_condition_node.outColorR >> out_invert_mult.input1
    out_invert_mult.output >> out_bank_grp.rotateZ
    bank_condition_node.outColorG >> in_invert_mult.input1
    in_invert_mult.output >> in_bank_grp.rotateZ





# TODO: Add space switch on pole vectors
def add_space_switch(attr_ctrl, attr_name="Inherit_Transforms", parent_world=None, parent_local=None, target_grp=None):
    '''
    Add space switching to a control
    Args:
        attr_ctrl: The rig control that will hold the attribute manipulating the space switching
        attr_name: Name of the attribute on the control
        parent_world: The world or higher level parent space
        parent_local: The world or lower level parent space
        target_grp: The control group or object to move between parent spaces
    Returns:

    '''
    # Save the control and save parent objects if selected
    attr_ctrl = attr_ctrl[0]
    if not parent_world:
        parent_world = pm.selected()[0]
    if not parent_local:
        parent_local = pm.selected()[1]
    if not target_grp:
        target_grp = pm.selected()[2]

    parent_constraint = pm.parentConstraint(parent_world, parent_local, target_grp, maintainOffset=True)

    # Add the switching attribute to the control and create the needed reverse node
    attr_ctrl.addAttr(attr_name, attributeType="double", min=0, max=1, defaultValue=1)
    attr_ctrl.listAttr()[-1].set(keyable=True)
    reverse_node = pm.createNode("reverse", name="{}_{}_reverse".format(attr_ctrl.name(), attr_name))

    # Connect the attributes to the constraint
    attr_ctrl.listAttr()[-1] >> parent_constraint.listAttr()[-1]
    attr_ctrl.listAttr()[-1] >> reverse_node.inputX
    reverse_node.outputX >> parent_constraint.listAttr()[-2]

    def create_wrist_correctives():
        ## Add wrist corrective joint functionality ##

        # Pre-reqs:
        # Create an offset group at the base of the wrist
        # Create a control for the corrective joint
        # ensure the corrective joint is parented to the wrist joint
        wrist_joint = pm.selected()[0]
        wrist_corr_base = pm.selected()[0]
        wrist_corr_zero_grp = pm.selected()[0]

        # For rotation of the base:
        # Create a multDoubleLinear node and set input2 to -0.630
        rot_multDL = pm.createNode("multDoubleLinear", name="right_wrist_top_corr_rot_multDoubleLinear")
        rot_multDL.input2.set(-0.630)
        # Connect the rotate Z of the wrist joint to the input1 of the multDoubleLinear
        wrist_joint.rotateZ >> rot_multDL.input1
        # Connect the output of the multDoubleLinear to the rotate Z of the base corrective control offset group
        rot_multDL.output >> wrist_corr_base.rotateZ

        # For translation of the corrective joint:
        # Create a multDoubleLinear node, set input2 to .25 and connect the Rotate Z of the wrist joint to input1
        tran_multDL = pm.createNode("multDoubleLinear", name="right_wrist_top_corr_transl_multDoubleLinear")
        tran_multDL.input2.set(0.25)
        wrist_joint.rotateZ >> tran_multDL.input1
        # Create a multiplyDivide node, set operation to Divide, set input2X to -7.609
        tran_multDV = pm.createNode("multiplyDivide", name="right_wrist_top_corr_transl_scalefactor_multiplyDivide")
        tran_multDV.operation.set(2)
        tran_multDV.input2X.set(7.609)
        # Connect the Output from the multDoubleLinear node to the input1X of the multiplyDivide
        tran_multDL.output >> tran_multDV.input1X
        # Create a condition node and connect the output of the multiplyDivide to the colorIfTrueR and First Term
        tran_condition = pm.createNode("condition", name="right_wrist_top_corr_transl_condition")
        tran_multDV.outputX >> tran_condition.colorIfTrueR
        tran_multDV.outputX >> tran_condition.firstTerm
        # Set operation to Greater Than and Second Term to 1
        tran_condition.operation.set(2)
        tran_condition.secondTerm.set(1)
        # Create a scaleFactor multDoubleLinear node
        tran_scalefactor_multDL = pm.createNode("multDoubleLinear",
                                                name="right_wrist_top_corr_transl_scalefactor_multDoubleLinear")
        # Connect outColorX of the condition node to the
        tran_condition.outColorR >> tran_scalefactor_multDL.input1
        tran_scalefactor_multDL.input2.set(-7.609)
        # Connect the output of the scalefactor multDoubleLinear to the Translate Y of the corrective control zero group
        tran_scalefactor_multDL.output >> wrist_corr_zero_grp.translateY


def add_hand_settings(hand_settings_ctrl, finger_ctrls=None):
    '''
    Adds attributes for controlling finger rotation to a designated settings control curve
    Args:
        hand_settings_ctrl: The control curve that will hold the finger/hand attributes
        finger_ctrls: The individual FK control curves of the fingers

    '''

    ## Add hand settings ##
    hand_settings_ctrl = hand_settings_ctrl[0]
    # Pre-req: have a hand setting control
    print(hand_settings_ctrl)
    # Add attributes
    hand_settings_ctrl.addAttr("FINGERS", attributeType="enum", enumName="---")
    hand_settings_ctrl.FINGERS.set(keyable=True)
    attribute_names = ["Thumb_Curl", "Index_Curl", "Middle_Curl", "Ring_Curl", "Pinky_Curl"]
    for i in attribute_names:
        hand_settings_ctrl.addAttr(i, attributeType="double", defaultValue=0)
        # hand_settings_ctrl.i.set(keyable=True)
        pm.setAttr("{}.{}".format(hand_settings_ctrl.name(), i), keyable=True)

    # Create buffer groups above finger controls if not already there
    if not finger_ctrls:
        raise RuntimeError("No finger controls available")
    else:
        buff_groups = []
        for i in finger_ctrls:
            print(i)
            # Get parent
            zero_grp =  i.listRelatives(parent=True)[0]

            # Create buffer group
            buff_grp = pm.group(empty=True)
            pm.delete(pm.parentConstraint(zero_grp, buff_grp, maintainOffset=False))
            buff_grp.rename(zero_grp.name().replace("ZERO", "BUFF"))
            buff_groups.append(buff_grp)
            pm.parent(buff_grp, zero_grp)
            pm.parent(i, buff_grp)

    # Connect attributes to finger controls
    inc = 0
    for attr in attribute_names:
        for buffer in buff_groups[inc:inc+3]:
            # hand_settings_ctrl.attr >> buffer.rotateZ
            pm.connectAttr("{}.{}".format(hand_settings_ctrl, attr), "{}.rotateZ".format(buffer))

        inc += 3def duplicate_joint_chain(joint_chain):
    '''
    Duplicate a joint chain without additional children or leaf joints
    Args:
        joint_chain: The explicit joint chain to duplicate (will not return any descendants)

    Returns: The duplicate joint chain
    '''
    duplicate_chain = [pm.duplicate(i, parentOnly=True)[0] for i in joint_chain]
    for i in range(len(duplicate_chain)-1):
        pm.parent(duplicate_chain[i+1], duplicate_chain[i])

    return duplicate_chain