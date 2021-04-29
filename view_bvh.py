from parser import Bvh
import numpy as np
import matplotlib.pyplot as plt
import sys
from mpl_toolkits.mplot3d import Axes3D

def _separate_angles(frames, joints, joints_saved_channels):

    frame_i = 0
    joints_saved_angles = {}
    get_channels = []
    for joint in joints:
        _saved_channels = joints_saved_channels[joint]

        saved_rotations = []
        for chan in _saved_channels:
            if chan.lower().find('rotation') != -1:
                saved_rotations.append(chan)
                get_channels.append(frame_i)

            frame_i += 1
        joints_saved_angles[joint] = saved_rotations

    joints_rotations = frames[:,get_channels]

    return joints_rotations, joints_saved_angles

def _separate_positions(frames, joints, joints_saved_channels):

    frame_i = 0
    joints_saved_positions = {}
    get_channels = []
    for joint in joints:
        _saved_channels = joints_saved_channels[joint]

        saved_positions = []
        for chan in _saved_channels:
            if chan.lower().find('position') != -1:
                saved_positions.append(chan)
                get_channels.append(frame_i)

            frame_i += 1
        joints_saved_positions[joint] = saved_positions


    if len(get_channels) == 3*len(joints):
        #print('all joints have saved positions')
        return frames[:,get_channels], joints_saved_positions

    #no positions saved for the joints or only some are saved.
    else:
        return np.array([]), joints_saved_positions

    pass

def ProcessBVH(filename):

    with open(filename) as f:
        mocap = Bvh(f.read())

    #get the names of the joints
    joints = mocap.get_joints_names()

    #this contains all of the frames data.
    frames = np.array(mocap.frames).astype('float32')

    #determine the structure of the skeleton and how the data was saved
    joints_offsets = {}
    joints_hierarchy = {}
    joints_saved_channels = {}
    for joint in joints:
        #get offsets. This is the length of skeleton body parts
        joints_offsets[joint] = np.array(mocap.joint_offset(joint))

        #Some bvh files save only rotation channels while others also save positions.
        #the order of rotation is important
        joints_saved_channels[joint] = mocap.joint_channels(joint)

        #determine the hierarcy of each joint.
        joint_hierarchy = []
        parent_joint = joint
        while True:
            parent_name = mocap.joint_parent(parent_joint)
            if parent_name == None:break

            joint_hierarchy.append(parent_name.name)
            parent_joint = parent_name.name

        joints_hierarchy[joint] = joint_hierarchy

    #seprate the rotation angles and the positions of joints
    joints_rotations, joints_saved_angles = _separate_angles(frames, joints, joints_saved_channels)
    joints_positions, joints_saved_positions = _separate_positions(frames, joints, joints_saved_channels)

    #root positions are always saved
    root_positions = frames[:, 0:3]

    return [joints, joints_offsets, joints_hierarchy, root_positions, joints_rotations, joints_saved_angles, joints_positions, joints_saved_positions]

#rotation matrices
def Rx(ang, in_radians = False):
    if in_radians == False:
        ang = np.radians(ang)

    Rot_Mat = np.array([
        [1, 0, 0],
        [0, np.cos(ang), -1*np.sin(ang)],
        [0, np.sin(ang),    np.cos(ang)]
    ])
    return Rot_Mat

def Ry(ang, in_radians = False):
    if in_radians == False:
        ang = np.radians(ang)

    Rot_Mat = np.array([
        [np.cos(ang), 0, np.sin(ang)],
        [0, 1, 0],
        [-1*np.sin(ang), 0, np.cos(ang)]
    ])
    return Rot_Mat

def Rz(ang, in_radians = False):
    if in_radians == False:
        ang = np.radians(ang)

    Rot_Mat = np.array([
        [np.cos(ang), -1*np.sin(ang), 0],
        [np.sin(ang), np.cos(ang), 0],
        [0, 0, 1]
    ])
    return Rot_Mat

#the rotation matrices need to be chained according to the order in the file
def _get_rotation_chain(joint_channels, joint_rotations):

    #the rotation matrices are constructed in the order given in the file
    Rot_Mat =  np.array([[1,0,0],[0,1,0],[0,0,1]])#identity matrix 3x3
    order = ''
    index = 0
    for chan in joint_channels: #if file saves xyz ordered rotations, then rotation matrix must be chained as R_x @ R_y @ R_z
        if chan[0].lower() == 'x':
            Rot_Mat = Rot_Mat @ Rx(joint_rotations[index])
            order += 'x'

        elif chan[0].lower() == 'y':
            Rot_Mat = Rot_Mat @ Ry(joint_rotations[index])
            order += 'y'

        elif chan[0].lower() == 'z':
            Rot_Mat = Rot_Mat @ Rz(joint_rotations[index])
            order += 'z'
        index += 1
    #print(order)
    return Rot_Mat

#Here root position is used as local coordinate origin.
def _calculate_frame_joint_positions_in_local_space(joints, joints_offsets, frame_joints_rotations, joints_saved_angles, joints_hierarchy):

    local_positions = {}

    for joint in joints:

        #ignore root joint and set local coordinate to (0,0,0)
        if joint == joints[0]:
            local_positions[joint] = [0,0,0]
            continue

        connected_joints = joints_hierarchy[joint]
        connected_joints = connected_joints[::-1]
        connected_joints.append(joint) #this contains the chain of joints that finally end with the current joint that we want the coordinate of.
        Rot = np.eye(3)
        pos = [0,0,0]
        for i, con_joint in enumerate(connected_joints):
            if i == 0:
                pass
            else:
                parent_joint = connected_joints[i - 1]
                Rot = Rot @ _get_rotation_chain(joints_saved_angles[parent_joint], frame_joints_rotations[parent_joint])
            joint_pos = joints_offsets[con_joint]
            joint_pos = Rot @ joint_pos
            pos = pos + joint_pos

        local_positions[joint] = pos

    return local_positions

def _calculate_frame_joint_positions_in_world_space(local_positions, root_position, root_rotation, saved_angles):

    world_pos = {}
    for joint in local_positions:
        pos = local_positions[joint]

        Rot = _get_rotation_chain(saved_angles, root_rotation)
        pos = Rot @ pos

        pos = np.array(root_position) + pos
        world_pos[joint] = pos

    return world_pos


def Draw_bvh(joints, joints_offsets, joints_hierarchy, root_positions, joints_rotations, joints_saved_angles):

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    frame_joints_rotations = {en:[] for en in joints}

    """
    Number of frames skipped is controlled with this variable below. If you want all frames, set to 1.
    """
    frame_skips = 5

    figure_limit = None #used to set figure axis limits

    for i in range(0,len(joints_rotations), frame_skips):

        frame_data = joints_rotations[i]

        #fill in the rotations dict
        joint_index = 0
        for joint in joints:
            frame_joints_rotations[joint] = frame_data[joint_index:joint_index+3]
            joint_index += 3

        #this returns a dictionary of joint positions in local space. This can be saved to file to get the joint positions.
        local_pos = _calculate_frame_joint_positions_in_local_space(joints, joints_offsets, frame_joints_rotations, joints_saved_angles, joints_hierarchy)

        #calculate world positions
        world_pos = _calculate_frame_joint_positions_in_world_space(local_pos, root_positions[i], frame_joints_rotations[joints[0]], joints_saved_angles[joints[0]])

        #calculate the limits of the figure. Usually the last joint in the dictionary is one of the feet.
        if figure_limit == None:
            lim_min = np.abs(np.min(local_pos[list(local_pos)[-1]]))
            lim_max = np.abs(np.max(local_pos[list(local_pos)[-1]]))
            lim = lim_min if lim_min > lim_max else lim_max
            figure_limit = lim

        for joint in joints:
            if joint == joints[0]: continue #skip root joint
            parent_joint = joints_hierarchy[joint][0]
            plt.plot(xs = [local_pos[parent_joint][0], local_pos[joint][0]],
                     zs = [local_pos[parent_joint][1], local_pos[joint][1]],
                     ys = [local_pos[parent_joint][2], local_pos[joint][2]], c = 'blue', lw = 2.5)

            #uncomment here if you want to see the world coords. If nothing appears on screen, change the axis limits below!
            # plt.plot(xs = [world_pos[parent_joint][0], world_pos[joint][0]],
            #          zs = [world_pos[parent_joint][1], world_pos[joint][1]],
            #          ys = [world_pos[parent_joint][2], world_pos[joint][2]], c = 'red', lw = 2.5)

        #Depending on the file, the axis limits might be too small or too big. Change accordingly.
        ax.set_axis_off()
        ax.set_xlim(-0.6*figure_limit, 0.6*figure_limit)
        ax.set_ylim(-0.6*figure_limit, 0.6*figure_limit)
        ax.set_zlim(-0.2*figure_limit, 1.*figure_limit)
        plt.title('frame: ' + str(i))
        plt.pause(0.001)
        ax.cla()

    pass


if __name__ == "__main__":

    if len(sys.argv) != 2:
        print('Call the function with the BVH file')
        quit()

    filename = sys.argv[1]
    skeleton_data = ProcessBVH(filename)

    joints = skeleton_data[0]
    joints_offsets = skeleton_data[1]
    joints_hierarchy = skeleton_data[2]
    root_positions = skeleton_data[3]
    joints_rotations = skeleton_data[4] #this contains the angles in degrees
    joints_saved_angles = skeleton_data[5] #this contains channel information. E.g ['Xrotation', 'Yrotation', 'Zrotation']
    joints_positions = skeleton_data[6]
    joints_saved_positions = skeleton_data[7]

    Draw_bvh(joints, joints_offsets, joints_hierarchy, root_positions, joints_rotations, joints_saved_angles)
