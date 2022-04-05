# from numpy.linalg import norm
# import numpy as np
from numpy import array, zeros, ones
from scipy.spatial import ConvexHull
from scipy.optimize import linprog
from hpp.gepetto import ViewerFactory
from hpp.corbaserver.rbprm import rbprmstate, state_alg
from hpp.corbaserver.rbprm.anymal import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.rbprm.tools.display_tools import hull_to_obj, plt, plot_hull

# from plot_polytopes import *
# from pinocchio import Quaternion

NUM_SAMPLES = 18000
IT_DISPLAY_PROGRESS = NUM_SAMPLES / 10
MIN_DIST_BETWEEN_FEET_Y = 0.10
MIN_DIST_BETWEEN_FEET_X = 0.10
MAX_DIST_BETWEEN_FEET_X = 0.35
MAX_DIST_BETWEEN_FEET_Z = 0.35
MIN_HEIGHT_COM = 0.3
# margin used to constrain the com y position : if it's on the left of the left foot
# or on the right of the right foot
# for more than this margin, we reject this sample:
MARGIN_FEET_SIDE = 0.05

fullBody = Robot()

fullBody.setConstrainedJointsBounds()
fullBody.setJointBounds("LF_KFE", [-1.4, 0.0])
fullBody.setJointBounds("RF_KFE", [-1.4, 0.0])
fullBody.setJointBounds("LH_KFE", [0.0, 1.4])
fullBody.setJointBounds("RH_KFE", [0.0, 1.4])
fullBody.setJointBounds("root_joint", [-20, 20, -20, 20, -20, 20])
dict_heuristic = {
    fullBody.rLegId: "static",
    fullBody.lLegId: "static",
    fullBody.rArmId: "fixedStep04",
    fullBody.lArmId: "fixedStep04",
}
fullBody.loadAllLimbs(dict_heuristic, "ReferenceConfiguration", nbSamples=12)

nbSamples = 1

ps = ProblemSolver(fullBody)
vf = ViewerFactory(ps)
v = vf.createViewer()
rootName = "root_joint"

zero = [0.0, 0.0, 0.0]
rLegId = fullBody.rLegId
rLeg = fullBody.rleg
rfoot = fullBody.rfoot
rLegOffset = fullBody.offset[:]
lLegOffset = fullBody.offset[:]
rArmOffset = fullBody.offset[:]
lArmOffset = fullBody.offset[:]

lLegId = fullBody.lLegId
lLeg = fullBody.lleg
lfoot = fullBody.lfoot

# make sure this is 0
q_0 = fullBody.getCurrentConfig()
zeroConf = [0, 0, 0, 0, 0, 0, 1.0]
q_0[0:7] = zeroConf
fullBody.setCurrentConfig(q_0)

effectors = [
    fullBody.rfoot,
    fullBody.lfoot,
    fullBody.rhand,
    fullBody.lhand,
]
limbIds = [fullBody.rLegId, fullBody.lLegId, fullBody.rArmId, fullBody.lArmId]
offsets = [array(rLegOffset), array(lLegOffset), array(rArmOffset), array(lArmOffset)]

compoints = [[] for _ in effectors]
# compoints = [[[0.012471792486262121, 0.0015769611415203033, 0.8127583093263778]],
# [[0.012471792486262121, 0.0015769611415203033, 0.8127583093263778]]]
points = [{} for _ in effectors]
for i, eff in enumerate(effectors):
    for j, otherEff in enumerate(effectors):
        if i != j:
            points[i][otherEff] = []

success = 0
fails = 0


# static eq is com is convex combination of pos (projected)
def staticEq(positions, com):
    sizeX = len(positions)
    E = zeros((3, sizeX))
    for i, pos in enumerate(positions):
        E[:2, i] = pos[:2]
    e = array([com[0], com[1], 1.0])
    E[2, :] = ones(sizeX)
    res = linprog(
        ones(sizeX),
        A_ub=None,
        b_ub=None,
        A_eq=E,
        b_eq=e,
        bounds=[(0.0, 1.0) for _ in range(sizeX)],
        method="interior-point",
        callback=None,
        options={"presolve": True},
    )
    return res["success"]


def pointInsideHull(positions):
    """
    returns true of one of the point is inside the convex hulls of the others.
    We do not want that
    """
    for i, pos in enumerate(positions):
        others = positions[:i] + positions[i + 1 :]
        if staticEq(others, pos):
            return True
    return False


def genFlat(init=False):
    q = fullBody.shootRandomConfig()
    if init:
        q = fullBody.referenceConfig[::]
    q[0:7] = zeroConf
    fullBody.setCurrentConfig(q)
    # v(q)

    positions = [fullBody.getJointPosition(foot)[:3] for foot in effectors]

    s = rbprmstate.State(fullBody, q=q, limbsIncontact=limbIds)
    succ = True
    for effId, pos in zip(limbIds, positions):
        s, succ = state_alg.addNewContact(
            s, effId, pos, [0.0, 0.0, 1.0], num_max_sample=0
        )
        if not succ:
            break

    # posrf = fullBody.getJointPosition(rfoot)[:3]
    # poslf = fullBody.getJointPosition(lfoot)[:3]
    # print ("limbsIds ", limbIds)
    # s = rbprmstate.State(fullBody, q = q, limbsIncontact = limbIds)
    # s, succ = state_alg.addNewContact(s, rLegId, posrf, [0.,0.,1.], num_max_sample= 0)
    # if succ:
    # s, succ = state_alg.addNewContact(s, lLegId, poslf, [0.,0.,1.], num_max_sample= 0)
    if succ:
        # ~ succ = fullBody.isConfigValid(q)[0]
        #          and norm (array(posrf[:2]) - array(poslf[:2]) ) >= 0.3
        succ = fullBody.isConfigValid(q)[0]

    # assert that in static equilibrium
    if succ:
        fullBody.setCurrentConfig(q)
        succ = staticEq(positions, fullBody.getCenterOfMass())
        if not succ:
            v(q)
    if succ:
        succ = not pointInsideHull(positions)
        if not succ:
            print("************* contacts crossing", not succ)
            v(q)
            # if succ and norm (array(posrf[:2]) - array(poslf[:2]) ) <= 0.1:
            # if succ and norm (array(posrf) - array(poslf) ) <= 0.1:
            v(s.q())
    return s.q(), succ, s, positions


def printFootPositionRelativeToOther(nbConfigs):
    for i in range(0, nbConfigs):
        if i > 0 and not i % IT_DISPLAY_PROGRESS:
            print(int((i * 100) / nbConfigs), " % done")
        q, succ, s, pos = genFlat(i == 0)
        if succ:
            global success
            success += 1
            addCom = True
            for j, effectorName in enumerate(effectors):
                for otheridx, (oeffectorName, limbId) in enumerate(
                    zip(effectors, limbIds)
                ):
                    if otheridx != j:
                        fullBody.setCurrentConfig(q)
                        pos_other = fullBody.getJointPosition(oeffectorName)
                        pos = fullBody.getJointPosition(effectorName)
                        p = array(pos_other[:3]) - array(pos[:3]).tolist()
                        # ~ qtr = q[:]
                        # ~ qtr[:3] = [qtr[0] - pos_other[0], qtr[1] - pos_other[1],
                        # ~             qtr[2] - pos_other[2]]
                        # ~ fullBody.setCurrentConfig(qtr)
                        # ~ qEffector = fullBody.getJointPosition(effectorName)

                        # check current joint pos is now zero
                        # ~ q0 = Quaternion(qEffector[6], qEffector[3], qEffector[4],
                        #                   qEffector[5])
                        # ~ rot = q0.matrix()  # compute rotation matrix world -> local
                        # (0,0,0) coordinate expressed in effector fram
                        # ~ p = qEffector[0:3]
                        # ~ rm = np.zeros((4, 4))
                        # ~ for k in range(0, 3):
                        # ~ for l in range(0, 3):
                        # ~ rm[k, l] = rot[k, l]
                        # ~ for m in range(0, 3):
                        # ~ rm[m, 3] = qEffector[m]
                        # ~ rm[3, 3] = 1
                        # ~ invrm = np.linalg.inv(rm)
                        # ~ p = invrm.dot([0, 0, 0., 1])
                        if MAX_DIST_BETWEEN_FEET_Z > abs(p[2]):
                            if MIN_DIST_BETWEEN_FEET_Y <= abs(p[1]):
                                if MIN_DIST_BETWEEN_FEET_X <= abs(p[0]):
                                    # this is not what we want to do in theory
                                    # but it works well in fact
                                    points[j][oeffectorName].append(p[:3])
                                else:
                                    addCom = False
                            else:
                                addCom = False
                        else:
                            print(
                                "rejecting ",
                                effectorName,
                                " ",
                                oeffectorName,
                                p,
                                abs(p[2]),
                            )
                            # ~ print ('pos_other', pos_other)
                            # ~ print ('old_pos', old_pos)
                            addCom = False
                            v(q)
                        # ~ if (j == 0 and p[1] > MIN_DIST_BETWEEN_FEET_Y
                        #       and abs(p[0]) < MAX_DIST_BETWEEN_FEET_X):
                        # ~ points[j].append(p[:3])
                        # ~ elif (j == 1 and p[1] < -MIN_DIST_BETWEEN_FEET_Y
                        #         and abs(p[0]) < MAX_DIST_BETWEEN_FEET_X):
                        # ~ points[j].append(p[:3])
                        # ~ else:
                        # ~ addCom =
            # now compute coms

            fullBody.setCurrentConfig(q)
            com = array(fullBody.getCenterOfMass())
            print("com ", com)
            # ~ for x in range(0, 3):
            # ~ q[x] = -com[x]
            for j, effectorName in enumerate(effectors):
                pos = fullBody.getJointPosition(effectorName)
                rp = array(com) - array(pos[:3]).tolist()
                # ~ qEffector = fullBody.getJointPosition(effectorName)
                # ~ q0 = Quaternion(qEffector[6], qEffector[3], qEffector[4],
                #                   qEffector[5])
                # ~ rot = q0.matrix()  # compute rotation matrix world -> local
                # ~ p = qEffector[0:3]  # (0,0,0) coordinate expressed in effector fram
                # ~ rm = np.zeros((4, 4))
                # ~ for k in range(0, 3):
                # ~ for l in range(0, 3):
                # ~ rm[k, l] = rot[k, l]
                # ~ for m in range(0, 3):
                # ~ rm[m, 3] = qEffector[m]
                # ~ rm[3, 3] = 1
                # ~ invrm = np.linalg.inv(rm)
                # ~ p = invrm.dot([0, 0, 0, 1])
                # ~ # add offset
                # ~ rp = array(p[:3] - offsets[j]).tolist()

                if rp[2] < MIN_HEIGHT_COM:
                    addCom = False
                    print("reject min heught")
                if addCom:
                    compoints[j].append(rp)
                    # ~ if j == 1:
                    # ~ if rp[1] < MARGIN_FEET_SIDE:
                    # ~ compoints[j].append(rp)
                    # ~ else:
                    # ~ if rp[1] > -MARGIN_FEET_SIDE:
                    # ~ compoints[j].append(rp)

        else:
            global fails
            fails += 1
            # print(fullBody.isConfigValid(q)[1])
    # for j in range(0,len(limbIds)):
    # f1=open('./'+str(limbIds[j])+'_com.erom', 'w+')
    # for p in points[j]:
    # f1.write(str(p[0]) + "," + str(p[1]) + "," + str(p[2]) + "\n")
    # f1.close()


s = rbprmstate.State(
    fullBody, q=fullBody.getCurrentConfig(), limbsIncontact=[fullBody.limbs_names[0]]
)

# printRootPosition(rLegId, rfoot, nbSamples)
# printRootPosition(lLegId, lfoot, nbSamples)
# printRootPosition(rarmId, rHand, nbSamples)
# printRootPosition(larmId, lHand, nbSamples)
printFootPositionRelativeToOther(6000)
print("successes ", success)
print("fails  ", fails)

# ~ for effector, comData, pointsData in zip(effectors, compoints, points):
# ~ for effector, limbId, comData, pointsData in zip(effectors[:1],limbIds[1:],
#                                                    compoints[:1], points[:1]):
for effector, limbId, comData, pointsData in zip(effectors, limbIds, compoints, points):
    hcom = ConvexHull(comData)
    hull_to_obj(
        hcom,
        comData,
        "anymal_COM_constraints_in_" + str(limbId) + "_effector_frame_quasi_static.obj",
    )
    fig = plt.figure()
    fig.suptitle(
        "anymal_COM_constraints_in_" + str(limbId) + "_effector_frame_quasi_static.obj",
        fontsize=16,
    )
    plot_hull(hcom, comData, array(comData), color="r", plot=False, fig=fig, ax=None)

    fig = plt.figure()
    fig.suptitle(str(limbId), fontsize=16)
    # ~ axes = [221,222,223,224]
    ax = None
    # ~ for (oEffector, pts), axId in zip(pointsData.items(), axes):
    for (oEffector, pts) in pointsData.items():
        # ~ ax = fig.add_subplot(axId, projection="3d")
        hpts = ConvexHull(pts)
        hull_to_obj(
            hpts,
            pts,
            "anymal_" + str(oEffector) + "_constraints_in_" + str(limbId) + ".obj",
        )
        print("ax ", ax)
        ax = plot_hull(hpts, pts, array(pts), color="b", plot=False, fig=fig, ax=ax)
        print(
            "effector ",
            limbId,
        )
        print(
            "oEffector ",
            oEffector,
        )
    plt.show(block=False)

# ~ hcomRF = ConvexHull(compoints[0])
# ~ hcomLF = ConvexHull(compoints[1])
# ~ hull_to_obj(hcomRF,compoints[0],"anymal_COM_constraints_in_RF_effector_frame.obj")
# ~ hull_to_obj(hcomLF,compoints[1],"anymal_COM_constraints_in_LF_effector_frame.obj")

# ~ hptsRF = ConvexHull(points[0])
# ~ hptsLF = ConvexHull(points[1])
# ~ hull_to_obj(hptsRF,points[0],"anymal_LF_constraints_in_RF.obj")
# ~ hull_to_obj(hptsLF,points[1],"anymal_RF_constraints_in_LF.obj")

# ~ for k in range(2):
# ~ hcom = ConvexHull(compoints[k])
# ~ plot_hull(hcom, compoints[k], array(compoints[k]))

# ~ hpts = ConvexHull(points[k])
# ~ plot_hull(hpts, points[k], array(points[k]), color = "b", plot = k == 1 and True)
