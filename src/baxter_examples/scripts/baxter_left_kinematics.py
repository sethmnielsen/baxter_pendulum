from math import sin, cos
from offset_util import offset_and_reshape
import numpy as np
pi = np.pi



def joint_fk00(q):
#
    pose = [0]*16
#
    x0 = q[0] + (1/4)*math.pi
    x1 = math.cos(x0)
    x2 = math.sin(x0)
#
    pose[0] = x1
    pose[1] = 0
    pose[2] = -x2
    pose[3] = 0.069*x1
    pose[4] = x2
    pose[5] = 0
    pose[6] = x1
    pose[7] = 0.069*x2
    pose[8] = 0
    pose[9] = -1
    pose[10] = 0
    pose[11] = 0.270350000000000
    pose[12] = 0
    pose[13] = 0
    pose[14] = 0
    pose[15] = 1
#
    pose = offset_and_reshape(pose,0.0640273121782,0.259027312178,0.129626)
    return pose



def joint_fk01(q):
#
    pose = [0]*16
#
    x0 = math.sin(q[1])
    x1 = q[0] + (1/4)*math.pi
    x2 = math.cos(x1)
    x3 = math.sin(x1)
    x4 = math.cos(q[1])
#
    pose[0] = -x0*x2
    pose[1] = -x3
    pose[2] = x2*x4
    pose[3] = 0.069*x2
    pose[4] = -x0*x3
    pose[5] = x2
    pose[6] = x3*x4
    pose[7] = 0.069*x3
    pose[8] = -x4
    pose[9] = 0
    pose[10] = -x0
    pose[11] = 0.270350000000000
    pose[12] = 0
    pose[13] = 0
    pose[14] = 0
    pose[15] = 1
#
    pose = offset_and_reshape(pose,0.0640273121782,0.259027312178,0.129626)
    return pose



def joint_fk02(q):
#
    pose = [0]*16
#
    x0 = math.sin(q[2])
    x1 = q[0] + (1/4)*math.pi
    x2 = math.sin(x1)
    x3 = x0*x2
    x4 = math.sin(q[1])
    x5 = math.cos(q[2])
    x6 = math.cos(x1)
    x7 = x5*x6
    x8 = math.cos(q[1])
    x9 = x6*x8
    x10 = x2*x5
    x11 = x0*x6
    x12 = 0.069*x6
    x13 = x10*x4
    x14 = x2*x8
    x15 = x5*x8
#
    pose[0] = -x3 - x4*x7
    pose[1] = -x9
    pose[2] = -x10 + x11*x4
    pose[3] = -x12*x4*x5 + x12 - 0.069*x3 + 0.36435*x9
    pose[4] = x11 - x13
    pose[5] = -x14
    pose[6] = x3*x4 + x7
    pose[7] = x0*x12 - 0.069*x13 + 0.36435*x14 + 0.069*x2
    pose[8] = -x15
    pose[9] = x4
    pose[10] = x0*x8
    pose[11] = -0.069*x15 - 0.36435*x4 + 0.27035
    pose[12] = 0
    pose[13] = 0
    pose[14] = 0
    pose[15] = 1
#
    pose = offset_and_reshape(pose,0.0640273121782,0.259027312178,0.129626)
    return pose



def joint_fk03(q):
#
    pose = [0]*16
#
    x0 = math.sin(q[3])
    x1 = q[0] + (1/4)*math.pi
    x2 = math.cos(x1)
    x3 = math.cos(q[1])
    x4 = x2*x3
    x5 = math.cos(q[3])
    x6 = math.sin(q[2])
    x7 = math.sin(x1)
    x8 = x6*x7
    x9 = math.sin(q[1])
    x10 = math.cos(q[2])
    x11 = x10*x2
    x12 = -x11*x9 - x8
    x13 = x10*x7
    x14 = x2*x6
    x15 = 0.069*x2
    x16 = x3*x7
    x17 = x13*x9
    x18 = x14 - x17
    x19 = x10*x3
#
    pose[0] = -x0*x4 + x12*x5
    pose[1] = -x13 + x14*x9
    pose[2] = x0*x12 + x4*x5
    pose[3] = -x10*x15*x9 + x15 + 0.36435*x4 - 0.069*x8
    pose[4] = -x0*x16 + x18*x5
    pose[5] = x11 + x8*x9
    pose[6] = x0*x18 + x16*x5
    pose[7] = x15*x6 + 0.36435*x16 - 0.069*x17 + 0.069*x7
    pose[8] = x0*x9 - x19*x5
    pose[9] = x3*x6
    pose[10] = -x0*x19 - x5*x9
    pose[11] = -0.069*x19 - 0.36435*x9 + 0.27035
    pose[12] = 0
    pose[13] = 0
    pose[14] = 0
    pose[15] = 1
#
    pose = offset_and_reshape(pose,0.0640273121782,0.259027312178,0.129626)
    return pose



def joint_fk04(q):
#
    pose = [0]*16
#
    x0 = math.sin(q[4])
    x1 = math.cos(q[2])
    x2 = q[0] + (1/4)*math.pi
    x3 = math.sin(x2)
    x4 = x1*x3
    x5 = math.sin(q[1])
    x6 = math.sin(q[2])
    x7 = math.cos(x2)
    x8 = x6*x7
    x9 = -x4 + x5*x8
    x10 = x0*x9
    x11 = math.cos(q[4])
    x12 = math.sin(q[3])
    x13 = math.cos(q[1])
    x14 = x13*x7
    x15 = math.cos(q[3])
    x16 = x3*x6
    x17 = x1*x7
    x18 = -x16 - x17*x5
    x19 = -x12*x14 + x15*x18
    x20 = x11*x19
    x21 = x14*x15
    x22 = x12*x18
    x23 = 0.069*x7
    x24 = x16*x5 + x17
    x25 = x0*x24
    x26 = x13*x3
    x27 = x4*x5
    x28 = -x27 + x8
    x29 = -x12*x26 + x15*x28
    x30 = x11*x29
    x31 = x15*x26
    x32 = x12*x28
    x33 = x13*x6
    x34 = x0*x33
    x35 = x1*x13
    x36 = x12*x5 - x15*x35
    x37 = x11*x36
    x38 = x15*x5
    x39 = x12*x35
#
    pose[0] = x10 + x20
    pose[1] = -x21 - x22
    pose[2] = -x0*x19 + x11*x9
    pose[3] = -x1*x23*x5 + 0.01*x10 + 0.36435*x14 - 0.069*x16 + 0.01*x20 + 0.37429*x21 + 0.37429*x22 + x23
    pose[4] = x25 + x30
    pose[5] = -x31 - x32
    pose[6] = -x0*x29 + x11*x24
    pose[7] = x23*x6 + 0.01*x25 + 0.36435*x26 - 0.069*x27 + 0.069*x3 + 0.01*x30 + 0.37429*x31 + 0.37429*x32
    pose[8] = x34 + x37
    pose[9] = x38 + x39
    pose[10] = -x0*x36 + x11*x33
    pose[11] = 0.01*x34 - 0.069*x35 + 0.01*x37 - 0.37429*x38 - 0.37429*x39 - 0.36435*x5 + 0.27035
    pose[12] = 0
    pose[13] = 0
    pose[14] = 0
    pose[15] = 1
#
    pose = offset_and_reshape(pose,0.0640273121782,0.259027312178,0.129626)
    return pose



def joint_fk05(q):
#
    pose = [0]*16
#
    x0 = math.sin(q[5])
    x1 = math.cos(q[3])
    x2 = q[0] + (1/4)*math.pi
    x3 = math.cos(x2)
    x4 = math.cos(q[1])
    x5 = x3*x4
    x6 = x1*x5
    x7 = math.sin(q[3])
    x8 = math.sin(q[2])
    x9 = math.sin(x2)
    x10 = x8*x9
    x11 = math.sin(q[1])
    x12 = math.cos(q[2])
    x13 = x12*x3
    x14 = -x10 - x11*x13
    x15 = x14*x7
    x16 = -x15 - x6
    x17 = math.cos(q[5])
    x18 = math.sin(q[4])
    x19 = x12*x9
    x20 = x3*x8
    x21 = x11*x20 - x19
    x22 = x18*x21
    x23 = math.cos(q[4])
    x24 = x1*x14 - x5*x7
    x25 = x23*x24
    x26 = x22 + x25
    x27 = 0.069*x3
    x28 = x4*x9
    x29 = x1*x28
    x30 = x11*x19
    x31 = x20 - x30
    x32 = x31*x7
    x33 = -x29 - x32
    x34 = x10*x11 + x13
    x35 = x18*x34
    x36 = x1*x31 - x28*x7
    x37 = x23*x36
    x38 = x35 + x37
    x39 = x1*x11
    x40 = x12*x4
    x41 = x40*x7
    x42 = x39 + x41
    x43 = x4*x8
    x44 = x18*x43
    x45 = -x1*x40 + x11*x7
    x46 = x23*x45
    x47 = x44 + x46
#
    pose[0] = x0*x16 + x17*x26
    pose[1] = -x18*x24 + x21*x23
    pose[2] = x0*x26 - x16*x17
    pose[3] = -0.069*x10 - x11*x12*x27 + 0.37429*x15 + 0.01*x22 + 0.01*x25 + x27 + 0.36435*x5 + 0.37429*x6
    pose[4] = x0*x33 + x17*x38
    pose[5] = -x18*x36 + x23*x34
    pose[6] = x0*x38 - x17*x33
    pose[7] = x27*x8 + 0.36435*x28 + 0.37429*x29 - 0.069*x30 + 0.37429*x32 + 0.01*x35 + 0.01*x37 + 0.069*x9
    pose[8] = x0*x42 + x17*x47
    pose[9] = -x18*x45 + x23*x43
    pose[10] = x0*x47 - x17*x42
    pose[11] = -0.36435*x11 - 0.37429*x39 - 0.069*x40 - 0.37429*x41 + 0.01*x44 + 0.01*x46 + 0.27035
    pose[12] = 0
    pose[13] = 0
    pose[14] = 0
    pose[15] = 1
#
    pose = offset_and_reshape(pose,0.0640273121782,0.259027312178,0.129626)
    return pose



def joint_fk06(q):
#
    pose = [0]*16
#
    x0 = math.sin(q[6])
    x1 = math.cos(q[4])
    x2 = math.cos(q[2])
    x3 = q[0] + (1/4)*math.pi
    x4 = math.sin(x3)
    x5 = x2*x4
    x6 = math.sin(q[1])
    x7 = math.sin(q[2])
    x8 = math.cos(x3)
    x9 = x7*x8
    x10 = -x5 + x6*x9
    x11 = math.sin(q[4])
    x12 = math.sin(q[3])
    x13 = math.cos(q[1])
    x14 = x13*x8
    x15 = math.cos(q[3])
    x16 = x4*x7
    x17 = x2*x8
    x18 = -x16 - x17*x6
    x19 = -x12*x14 + x15*x18
    x20 = x1*x10 - x11*x19
    x21 = math.cos(q[6])
    x22 = math.sin(q[5])
    x23 = x14*x15
    x24 = x12*x18
    x25 = -x23 - x24
    x26 = math.cos(q[5])
    x27 = x10*x11
    x28 = x1*x19
    x29 = x27 + x28
    x30 = x22*x25 + x26*x29
    x31 = x25*x26
    x32 = x22*x29
    x33 = 0.069*x8
    x34 = x16*x6 + x17
    x35 = x13*x4
    x36 = x5*x6
    x37 = -x36 + x9
    x38 = -x12*x35 + x15*x37
    x39 = x1*x34 - x11*x38
    x40 = x15*x35
    x41 = x12*x37
    x42 = -x40 - x41
    x43 = x11*x34
    x44 = x1*x38
    x45 = x43 + x44
    x46 = x22*x42 + x26*x45
    x47 = x26*x42
    x48 = x22*x45
    x49 = x13*x7
    x50 = x13*x2
    x51 = x12*x6 - x15*x50
    x52 = x1*x49 - x11*x51
    x53 = x15*x6
    x54 = x12*x50
    x55 = x53 + x54
    x56 = x11*x49
    x57 = x1*x51
    x58 = x56 + x57
    x59 = x22*x55 + x26*x58
    x60 = x26*x55
    x61 = x22*x58
#
    pose[0] = x0*x20 + x21*x30
    pose[1] = -x0*x30 + x20*x21
    pose[2] = -x31 + x32
    pose[3] = 0.36435*x14 - 0.069*x16 - x2*x33*x6 + 0.37429*x23 + 0.37429*x24 + 0.01*x27 + 0.01*x28 - 0.229525*x31 + 0.229525*x32 + x33
    pose[4] = x0*x39 + x21*x46
    pose[5] = -x0*x46 + x21*x39
    pose[6] = -x47 + x48
    pose[7] = x33*x7 + 0.36435*x35 - 0.069*x36 + 0.069*x4 + 0.37429*x40 + 0.37429*x41 + 0.01*x43 + 0.01*x44 - 0.229525*x47 + 0.229525*x48
    pose[8] = x0*x52 + x21*x59
    pose[9] = -x0*x59 + x21*x52
    pose[10] = -x60 + x61
    pose[11] = -0.069*x50 - 0.37429*x53 - 0.37429*x54 + 0.01*x56 + 0.01*x57 - 0.36435*x6 - 0.229525*x60 + 0.229525*x61 + 0.27035
    pose[12] = 0
    pose[13] = 0
    pose[14] = 0
    pose[15] = 1
#
    pose = offset_and_reshape(pose,0.0640273121782,0.259027312178,0.129626)
    return pose



FK = {0:joint_fk00, 1:joint_fk01, 2:joint_fk02, 3:joint_fk03, 4:joint_fk04, 5:joint_fk05, 6:joint_fk06, }



def jacobian00(q):
#
    jacobian = [0]*42
#
    x0 = q[0] + (1/4)*math.pi
#
    jacobian[0] = -0.069*math.sin(x0)
    jacobian[1] = 0
    jacobian[2] = 0
    jacobian[3] = 0
    jacobian[4] = 0
    jacobian[5] = 0
    jacobian[6] = 0
    jacobian[7] = 0.069*math.cos(x0)
    jacobian[8] = 0
    jacobian[9] = 0
    jacobian[10] = 0
    jacobian[11] = 0
    jacobian[12] = 0
    jacobian[13] = 0
    jacobian[14] = 0
    jacobian[15] = 0
    jacobian[16] = 0
    jacobian[17] = 0
    jacobian[18] = 0
    jacobian[19] = 0
    jacobian[20] = 0
    jacobian[21] = 0
    jacobian[22] = 0
    jacobian[23] = 0
    jacobian[24] = 0
    jacobian[25] = 0
    jacobian[26] = 0
    jacobian[27] = 0
    jacobian[28] = 0
    jacobian[29] = 0
    jacobian[30] = 0
    jacobian[31] = 0
    jacobian[32] = 0
    jacobian[33] = 0
    jacobian[34] = 0
    jacobian[35] = 1
    jacobian[36] = 0
    jacobian[37] = 0
    jacobian[38] = 0
    jacobian[39] = 0
    jacobian[40] = 0
    jacobian[41] = 0
#
    jacobian = np.array(jacobian).reshape(6,7)
    return jacobian



def jacobian01(q):
#
    jacobian = [0]*42
#
    x0 = q[0] + (1/4)*math.pi
    x1 = math.sin(x0)
    x2 = math.cos(x0)
#
    jacobian[0] = -0.069*x1
    jacobian[1] = 0
    jacobian[2] = 0
    jacobian[3] = 0
    jacobian[4] = 0
    jacobian[5] = 0
    jacobian[6] = 0
    jacobian[7] = 0.069*x2
    jacobian[8] = 0
    jacobian[9] = 0
    jacobian[10] = 0
    jacobian[11] = 0
    jacobian[12] = 0
    jacobian[13] = 0
    jacobian[14] = 0
    jacobian[15] = 0
    jacobian[16] = 0
    jacobian[17] = 0
    jacobian[18] = 0
    jacobian[19] = 0
    jacobian[20] = 0
    jacobian[21] = 0
    jacobian[22] = -x1
    jacobian[23] = 0
    jacobian[24] = 0
    jacobian[25] = 0
    jacobian[26] = 0
    jacobian[27] = 0
    jacobian[28] = 0
    jacobian[29] = x2
    jacobian[30] = 0
    jacobian[31] = 0
    jacobian[32] = 0
    jacobian[33] = 0
    jacobian[34] = 0
    jacobian[35] = 1
    jacobian[36] = 0
    jacobian[37] = 0
    jacobian[38] = 0
    jacobian[39] = 0
    jacobian[40] = 0
    jacobian[41] = 0
#
    jacobian = np.array(jacobian).reshape(6,7)
    return jacobian



def jacobian02(q):
#
    jacobian = [0]*42
#
    x0 = q[0] + (1/4)*math.pi
    x1 = math.sin(x0)
    x2 = 0.069*x1
    x3 = math.sin(q[2])
    x4 = math.cos(x0)
    x5 = 0.069*x4
    x6 = x3*x5
    x7 = math.cos(q[1])
    x8 = x1*x7
    x9 = 0.36435*x8
    x10 = math.sin(q[1])
    x11 = math.cos(q[2])
    x12 = x10*x11
    x13 = x12*x2
    x14 = -0.36435*x10 - 0.069*x11*x7
    x15 = x14*x4
    x16 = x1*x14
    x17 = -x13 + x6 + x9
    x18 = x4*x7
    x19 = -x12*x5 + 0.36435*x18 - x2*x3
#
    jacobian[0] = x13 - x2 - x6 - x9
    jacobian[1] = x15
    jacobian[2] = x10*x17 + x16*x7
    jacobian[3] = 0
    jacobian[4] = 0
    jacobian[5] = 0
    jacobian[6] = 0
    jacobian[7] = x19 + x5
    jacobian[8] = x16
    jacobian[9] = -x10*x19 - x15*x7
    jacobian[10] = 0
    jacobian[11] = 0
    jacobian[12] = 0
    jacobian[13] = 0
    jacobian[14] = 0
    jacobian[15] = -x1*x17 - x19*x4
    jacobian[16] = x17*x18 - x19*x8
    jacobian[17] = 0
    jacobian[18] = 0
    jacobian[19] = 0
    jacobian[20] = 0
    jacobian[21] = 0
    jacobian[22] = -x1
    jacobian[23] = x18
    jacobian[24] = 0
    jacobian[25] = 0
    jacobian[26] = 0
    jacobian[27] = 0
    jacobian[28] = 0
    jacobian[29] = x4
    jacobian[30] = x8
    jacobian[31] = 0
    jacobian[32] = 0
    jacobian[33] = 0
    jacobian[34] = 0
    jacobian[35] = 1
    jacobian[36] = 0
    jacobian[37] = -x10
    jacobian[38] = 0
    jacobian[39] = 0
    jacobian[40] = 0
    jacobian[41] = 0
#
    jacobian = np.array(jacobian).reshape(6,7)
    return jacobian



def jacobian03(q):
#
    jacobian = [0]*42
#
    x0 = q[0] + (1/4)*math.pi
    x1 = math.sin(x0)
    x2 = 0.069*x1
    x3 = math.sin(q[2])
    x4 = math.cos(x0)
    x5 = 0.069*x4
    x6 = x3*x5
    x7 = math.cos(q[1])
    x8 = x1*x7
    x9 = 0.36435*x8
    x10 = math.sin(q[1])
    x11 = math.cos(q[2])
    x12 = x10*x11
    x13 = x12*x2
    x14 = -0.36435*x10 - 0.069*x11*x7
    x15 = x14*x4
    x16 = x1*x14
    x17 = -x13 + x6 + x9
    x18 = x4*x7
    x19 = -x12*x5 + 0.36435*x18 - x2*x3
    x20 = x10*x3
#
    jacobian[0] = x13 - x2 - x6 - x9
    jacobian[1] = x15
    jacobian[2] = x10*x17 + x16*x7
    jacobian[3] = 0
    jacobian[4] = 0
    jacobian[5] = 0
    jacobian[6] = 0
    jacobian[7] = x19 + x5
    jacobian[8] = x16
    jacobian[9] = -x10*x19 - x15*x7
    jacobian[10] = 0
    jacobian[11] = 0
    jacobian[12] = 0
    jacobian[13] = 0
    jacobian[14] = 0
    jacobian[15] = -x1*x17 - x19*x4
    jacobian[16] = x17*x18 - x19*x8
    jacobian[17] = 0
    jacobian[18] = 0
    jacobian[19] = 0
    jacobian[20] = 0
    jacobian[21] = 0
    jacobian[22] = -x1
    jacobian[23] = x18
    jacobian[24] = -x1*x11 + x20*x4
    jacobian[25] = 0
    jacobian[26] = 0
    jacobian[27] = 0
    jacobian[28] = 0
    jacobian[29] = x4
    jacobian[30] = x8
    jacobian[31] = x1*x20 + x11*x4
    jacobian[32] = 0
    jacobian[33] = 0
    jacobian[34] = 0
    jacobian[35] = 1
    jacobian[36] = 0
    jacobian[37] = -x10
    jacobian[38] = x3*x7
    jacobian[39] = 0
    jacobian[40] = 0
    jacobian[41] = 0
#
    jacobian = np.array(jacobian).reshape(6,7)
    return jacobian



def jacobian04(q):
#
    jacobian = [0]*42
#
    x0 = q[0] + (1/4)*math.pi
    x1 = math.sin(x0)
    x2 = 0.069*x1
    x3 = math.sin(q[2])
    x4 = math.cos(x0)
    x5 = x3*x4
    x6 = 0.069*x5
    x7 = math.cos(q[1])
    x8 = x1*x7
    x9 = 0.36435*x8
    x10 = math.cos(q[2])
    x11 = math.sin(q[1])
    x12 = x10*x11*x2
    x13 = math.cos(q[3])
    x14 = x13*x8
    x15 = 0.37429*x14
    x16 = x10*x4
    x17 = x1*x3
    x18 = x11*x17 + x16
    x19 = 0.01*math.sin(q[4])
    x20 = x18*x19
    x21 = math.sin(q[3])
    x22 = x1*x10
    x23 = -x11*x22 + x5
    x24 = x21*x23
    x25 = 0.37429*x24
    x26 = 0.01*math.cos(q[4])
    x27 = x26*(x13*x23 - x21*x8)
    x28 = x10*x7
    x29 = x11*x13
    x30 = x3*x7
    x31 = x21*x28
    x32 = x19*x30 + x26*(x11*x21 - x13*x28) - 0.37429*x29 - 0.37429*x31
    x33 = -0.36435*x11 - 0.069*x28 + x32
    x34 = x33*x4
    x35 = x1*x33
    x36 = x15 + x20 + x25 + x27
    x37 = -x12 + x36 + x6 + x9
    x38 = x14 + x24
    x39 = -x29 - x31
    x40 = x4*x7
    x41 = x11*x16
    x42 = x13*x40
    x43 = x11*x5 - x22
    x44 = -x17 - x41
    x45 = x21*x44
    x46 = x19*x43 + x26*(x13*x44 - x21*x40) + 0.37429*x42 + 0.37429*x45
    x47 = -x2*x3 + 0.36435*x40 - 0.069*x41 + x46
    x48 = x42 + x45
#
    jacobian[0] = x12 - x15 - x2 - x20 - x25 - x27 - x6 - x9
    jacobian[1] = x34
    jacobian[2] = x11*x37 + x35*x7
    jacobian[3] = x18*x32 - x30*x36
    jacobian[4] = x32*x38 - x36*x39
    jacobian[5] = 0
    jacobian[6] = 0
    jacobian[7] = 0.069*x4 + x47
    jacobian[8] = x35
    jacobian[9] = -x11*x47 - x34*x7
    jacobian[10] = x30*x46 - x32*x43
    jacobian[11] = -x32*x48 + x39*x46
    jacobian[12] = 0
    jacobian[13] = 0
    jacobian[14] = 0
    jacobian[15] = -x1*x37 - x4*x47
    jacobian[16] = x37*x40 - x47*x8
    jacobian[17] = -x18*x46 + x36*x43
    jacobian[18] = x36*x48 - x38*x46
    jacobian[19] = 0
    jacobian[20] = 0
    jacobian[21] = 0
    jacobian[22] = -x1
    jacobian[23] = x40
    jacobian[24] = x43
    jacobian[25] = x48
    jacobian[26] = 0
    jacobian[27] = 0
    jacobian[28] = 0
    jacobian[29] = x4
    jacobian[30] = x8
    jacobian[31] = x18
    jacobian[32] = x38
    jacobian[33] = 0
    jacobian[34] = 0
    jacobian[35] = 1
    jacobian[36] = 0
    jacobian[37] = -x11
    jacobian[38] = x30
    jacobian[39] = x39
    jacobian[40] = 0
    jacobian[41] = 0
#
    jacobian = np.array(jacobian).reshape(6,7)
    return jacobian



def jacobian05(q):
#
    jacobian = [0]*42
#
    x0 = q[0] + (1/4)*math.pi
    x1 = math.sin(x0)
    x2 = 0.069*x1
    x3 = math.sin(q[2])
    x4 = math.cos(x0)
    x5 = x3*x4
    x6 = 0.069*x5
    x7 = math.cos(q[1])
    x8 = x1*x7
    x9 = 0.36435*x8
    x10 = math.cos(q[2])
    x11 = math.sin(q[1])
    x12 = x10*x11*x2
    x13 = math.cos(q[3])
    x14 = x13*x8
    x15 = 0.37429*x14
    x16 = x10*x4
    x17 = x1*x3
    x18 = x11*x17 + x16
    x19 = math.sin(q[4])
    x20 = 0.01*x19
    x21 = x18*x20
    x22 = math.sin(q[3])
    x23 = x1*x10
    x24 = -x11*x23 + x5
    x25 = x22*x24
    x26 = 0.37429*x25
    x27 = x13*x24 - x22*x8
    x28 = math.cos(q[4])
    x29 = 0.01*x28
    x30 = x27*x29
    x31 = x10*x7
    x32 = x11*x13
    x33 = x3*x7
    x34 = x22*x31
    x35 = x11*x22 - x13*x31
    x36 = x20*x33 + x29*x35 - 0.37429*x32 - 0.37429*x34
    x37 = -0.36435*x11 - 0.069*x31 + x36
    x38 = x37*x4
    x39 = x1*x37
    x40 = x15 + x21 + x26 + x30
    x41 = -x12 + x40 + x6 + x9
    x42 = x14 + x25
    x43 = -x32 - x34
    x44 = x4*x7
    x45 = x11*x16
    x46 = x13*x44
    x47 = x11*x5 - x23
    x48 = -x17 - x45
    x49 = x22*x48
    x50 = x13*x48 - x22*x44
    x51 = x20*x47 + x29*x50 + 0.37429*x46 + 0.37429*x49
    x52 = -x2*x3 + 0.36435*x44 - 0.069*x45 + x51
    x53 = x46 + x49
#
    jacobian[0] = x12 - x15 - x2 - x21 - x26 - x30 - x6 - x9
    jacobian[1] = x38
    jacobian[2] = x11*x41 + x39*x7
    jacobian[3] = x18*x36 - x33*x40
    jacobian[4] = x36*x42 - x40*x43
    jacobian[5] = 0
    jacobian[6] = 0
    jacobian[7] = 0.069*x4 + x52
    jacobian[8] = x39
    jacobian[9] = -x11*x52 - x38*x7
    jacobian[10] = x33*x51 - x36*x47
    jacobian[11] = -x36*x53 + x43*x51
    jacobian[12] = 0
    jacobian[13] = 0
    jacobian[14] = 0
    jacobian[15] = -x1*x41 - x4*x52
    jacobian[16] = x41*x44 - x52*x8
    jacobian[17] = -x18*x51 + x40*x47
    jacobian[18] = x40*x53 - x42*x51
    jacobian[19] = 0
    jacobian[20] = 0
    jacobian[21] = 0
    jacobian[22] = -x1
    jacobian[23] = x44
    jacobian[24] = x47
    jacobian[25] = x53
    jacobian[26] = -x19*x50 + x28*x47
    jacobian[27] = 0
    jacobian[28] = 0
    jacobian[29] = x4
    jacobian[30] = x8
    jacobian[31] = x18
    jacobian[32] = x42
    jacobian[33] = x18*x28 - x19*x27
    jacobian[34] = 0
    jacobian[35] = 1
    jacobian[36] = 0
    jacobian[37] = -x11
    jacobian[38] = x33
    jacobian[39] = x43
    jacobian[40] = -x19*x35 + x28*x33
    jacobian[41] = 0
#
    jacobian = np.array(jacobian).reshape(6,7)
    return jacobian



def jacobian06(q):
#
    jacobian = [0]*42
#
    x0 = q[0] + (1/4)*math.pi
    x1 = math.sin(x0)
    x2 = 0.069*x1
    x3 = math.sin(q[2])
    x4 = math.cos(x0)
    x5 = x3*x4
    x6 = 0.069*x5
    x7 = math.cos(q[1])
    x8 = x1*x7
    x9 = 0.36435*x8
    x10 = math.cos(q[2])
    x11 = math.sin(q[1])
    x12 = x10*x11*x2
    x13 = math.cos(q[3])
    x14 = x13*x8
    x15 = 0.37429*x14
    x16 = math.sin(q[4])
    x17 = x10*x4
    x18 = x1*x3
    x19 = x11*x18 + x17
    x20 = x16*x19
    x21 = 0.01*x20
    x22 = math.sin(q[3])
    x23 = x1*x10
    x24 = -x11*x23 + x5
    x25 = x22*x24
    x26 = 0.37429*x25
    x27 = math.cos(q[4])
    x28 = x13*x24 - x22*x8
    x29 = x27*x28
    x30 = 0.01*x29
    x31 = math.cos(q[5])
    x32 = x31*(-x14 - x25)
    x33 = 0.229525*x32
    x34 = math.sin(q[5])
    x35 = x34*(x20 + x29)
    x36 = 0.229525*x35
    x37 = x10*x7
    x38 = x11*x22 - x13*x37
    x39 = x27*x38
    x40 = x11*x13
    x41 = x3*x7
    x42 = x16*x41
    x43 = x22*x37
    x44 = x31*(x40 + x43)
    x45 = x34*(x39 + x42)
    x46 = -0.229525*x44 + 0.229525*x45
    x47 = 0.01*x39 - 0.37429*x40 + 0.01*x42 - 0.37429*x43 + x46
    x48 = -0.36435*x11 - 0.069*x37 + x47
    x49 = x4*x48
    x50 = x1*x48
    x51 = -x33 + x36
    x52 = x15 + x21 + x26 + x30 + x51
    x53 = -x12 + x52 + x6 + x9
    x54 = x14 + x25
    x55 = -x40 - x43
    x56 = -x16*x28 + x19*x27
    x57 = -x16*x38 + x27*x41
    x58 = -x32 + x35
    x59 = -x44 + x45
    x60 = x4*x7
    x61 = x11*x17
    x62 = -x18 - x61
    x63 = x22*x62
    x64 = x13*x62 - x22*x60
    x65 = x27*x64
    x66 = x11*x5 - x23
    x67 = x16*x66
    x68 = x13*x60
    x69 = x31*(-x63 - x68)
    x70 = x34*(x65 + x67)
    x71 = -0.229525*x69 + 0.229525*x70
    x72 = 0.37429*x63 + 0.01*x65 + 0.01*x67 + 0.37429*x68 + x71
    x73 = -x2*x3 + 0.36435*x60 - 0.069*x61 + x72
    x74 = x63 + x68
    x75 = -x16*x64 + x27*x66
    x76 = -x69 + x70
#
    jacobian[0] = x12 - x15 - x2 - x21 - x26 - x30 + x33 - x36 - x6 - x9
    jacobian[1] = x49
    jacobian[2] = x11*x53 + x50*x7
    jacobian[3] = x19*x47 - x41*x52
    jacobian[4] = x47*x54 - x52*x55
    jacobian[5] = x46*x56 - x51*x57
    jacobian[6] = x46*x58 - x51*x59
    jacobian[7] = 0.069*x4 + x73
    jacobian[8] = x50
    jacobian[9] = -x11*x73 - x49*x7
    jacobian[10] = x41*x72 - x47*x66
    jacobian[11] = -x47*x74 + x55*x72
    jacobian[12] = -x46*x75 + x57*x71
    jacobian[13] = -x46*x76 + x59*x71
    jacobian[14] = 0
    jacobian[15] = -x1*x53 - x4*x73
    jacobian[16] = x53*x60 - x73*x8
    jacobian[17] = -x19*x72 + x52*x66
    jacobian[18] = x52*x74 - x54*x72
    jacobian[19] = x51*x75 - x56*x71
    jacobian[20] = x51*x76 - x58*x71
    jacobian[21] = 0
    jacobian[22] = -x1
    jacobian[23] = x60
    jacobian[24] = x66
    jacobian[25] = x74
    jacobian[26] = x75
    jacobian[27] = x76
    jacobian[28] = 0
    jacobian[29] = x4
    jacobian[30] = x8
    jacobian[31] = x19
    jacobian[32] = x54
    jacobian[33] = x56
    jacobian[34] = x58
    jacobian[35] = 1
    jacobian[36] = 0
    jacobian[37] = -x11
    jacobian[38] = x41
    jacobian[39] = x55
    jacobian[40] = x57
    jacobian[41] = x59
#
    jacobian = np.array(jacobian).reshape(6,7)
    return jacobian



J = {0:jacobian00, 1:jacobian01, 2:jacobian02, 3:jacobian03, 4:jacobian04, 5:jacobian05, 6:jacobian06, }
