import re

from lxml import etree

import rospy

re_limb_and_pos = re.compile('([a-zA-Z]+)_([a-zA-Z]+)([0-9]+)_link')
def get_limb_and_pos_from_str(limb_pos_str):
    """ parse link name to get limb and position """
    m = re_limb_and_pos.match(limb_pos_str)
    if m is None:
        rospy.logwarn('ant_motion_controller: unknown limb and pos of link {}'
                      .format(limb_pos_str))
        return (None, None, None)
    else:
        return (m.group(1), m.group(2), int(m.group(3)))

def get_floats_from_string(input_str):
    try:
        return [float(v) for v in input_str.split(' ')]
    except ValueError:
        rospy.logwarn('ant_motion_controller: wrong format of float numbers: "{}"'
                      .format(input_str))
        return []

def load_robot_geometry(descr_str, last_parts_fixed, joints_num=3):
    """ describe robot geometry (limbs and joints) by URDF model;
    descr_str: URDF model of the robot;
    last_parts_fixed: True if length of last parts should be added to their parents'"""
    # load data
    descr_xml = etree.fromstring(descr_str)
    # find all joints and links to gather geometry data
    j_data = []
    l_data = {}
    for el in descr_xml:
        if el.tag == 'joint':
            parent = None
            child = None
            orig_xyz = '0. 0. 0.'
            orig_rpy = '0. 0. 0.'
            for j_el in el:
                if j_el.tag == 'parent':
                    parent = j_el.attrib['link']
                elif j_el.tag == 'child':
                    child = j_el.attrib['link']
                elif j_el.tag == 'origin':
                    orig_xyz = j_el.attrib['xyz']
                    orig_rpy = j_el.attrib['rpy']
            if parent is None or child is None:
                rospy.logwarn('ant_motion_controller: wrong link description:'
                    'parent = {}, child = {}'.format(parent, child))
                exit(-1)
            j_data.append((parent, child, orig_xyz, orig_rpy))
        # gather data about sizes of limbs
        elif el.tag == 'link':
            name = None
            if 'name' in el.attrib.keys():
                name = el.attrib['name']
            if name is None:
                continue # skip processing limbs without names
            size = None
            for el2 in el:
                if el2.tag == 'collision':
                    for el3 in el2:
                        if el3.tag == 'geometry':
                            for el4 in el3:
                                if el4.tag == 'box':
                                    # box of the link found; save size
                                    if 'size' in el4.attrib.keys():
                                        size = el4.attrib['size']
                                        break
                        if size is not None: # skip remaining parts
                            break
                if size is not None:
                    break
            # save link size
            if size is not None:
                l_data[name] = size
    
    #print('l_data: ', l_data)
    #print('j_data: ', j_data)
    
    # calculate parameters for each limb:
    # get first parts for each limb
    first_parts_pos = {}
    limbs_first_part = []
    for parent, child, xyz, rpy in j_data:
        if parent == 'body_link':
            side,pos,order = get_limb_and_pos_from_str(child)
            if side is None:
                continue
            else:
                # check correctness of xyz and rpy
                xyz_num = get_floats_from_string(xyz)
                if len(xyz_num) < 2:
                    rospy.logwarn('ant_motion_controller: processing of'
                        ' limb {}_{} skipped due to wrong coordinates'
                        ' of first link'.format(side, pos))
                    exit(-1)
                rpy_num = get_floats_from_string(rpy)
                if len(rpy_num) < 2:
                    rospy.logwarn('ant_motion_controller: processing of'
                        ' limb {}_{} skipped due to wrong orientation'
                        ' of first link'.format(side, pos))
                    exit(-1)
                first_parts_pos[(side, pos)] = (xyz_num[0], xyz_num[1], 
                                                xyz_num[2], rpy_num[2])
                limbs_first_part.append(child)
    
    # get full data about each limb
    limb_parts_poses = {(sd,ps) : []
        for (sd, ps) in first_parts_pos.keys()}
    for fp in limbs_first_part:
        cfp = fp
        side,pos,order = get_limb_and_pos_from_str(fp)
        found = True
        while found:
            found = False
            for parent, child, xyz, rpy2 in j_data:
                if parent == cfp:
                    cfp = child
                    # calculate length of link
                    szs = get_floats_from_string(xyz)
                    if len(szs) < 1:
                        rospy.logwarn('ant_motion_controller: processing of'
                            ' limb {}_{} skipped due to wrong length'
                            ' of link {}'.format(side, pos, parent))
                        exit(-1)
                    limb_parts_poses[(side,pos)].append(szs[0])
                    found = True
                    break
        # if last_parts_fixed, process final part separately
        if last_parts_fixed:
            # find size of last part in URDF
            if cfp in l_data.keys():
                sz = get_floats_from_string(l_data[cfp])
                if len(sz) < 1:
                    rospy.logwarn('ant_motion_controller: size of link {}'
                                  'has wrong format: {}'
                                  .format(cfp, l_data[cfp]))
                    exit(-1)
                limb_parts_poses[(side,pos)][-1] += sz[0] # add x to previous length
    # drop all parts of limbs except the first joints_num
    for k,v in limb_parts_poses.items():
        lengths = v[0:(joints_num - 1)] + [sum(v[joints_num - 1:])]
        limb_parts_poses[k] = lengths
    print(first_parts_pos)
    print(limb_parts_poses)
    return (first_parts_pos, limb_parts_poses)
    
