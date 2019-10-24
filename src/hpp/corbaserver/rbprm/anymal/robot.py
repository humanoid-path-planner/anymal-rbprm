#!/usr/bin/env python
# Copyright (c) 2019 CNRS
# Author: Pierre Fernbach
#
# This file is part of hpp-rbprm-robot-data.
# hpp_tutorial is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp_tutorial is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp_tutorial.  If not, see
# <http://www.gnu.org/licenses/>.

from hpp.corbaserver.rbprm.rbprmfullbody import FullBody as Parent
from pinocchio import SE3, Quaternion
import numpy as np

class Robot (Parent):
    ##
    #  Information to retrieve urdf and srdf files.

    packageName = "anymal_data"
    meshPackageName = "anymal_data"
    rootJointType = "freeflyer"
    urdfName = "anymal"
    urdfSuffix = ""
    srdfSuffix = ""

    ## Information about the names of thes joints defining the limbs of the robot
    rLegId = 'RFleg'
    rleg = 'RF_HAA'
    rfoot = 'RF_ADAPTER_TO_FOOT'
    lLegId = 'LFleg'
    lleg = 'LF_HAA'
    lfoot = 'LF_ADAPTER_TO_FOOT'
    lArmId = 'LHleg'
    larm = 'LH_HAA'
    lhand = 'LH_ADAPTER_TO_FOOT'
    rArmId = 'RHleg'
    rarm = 'RH_HAA'
    rhand = 'RH_ADAPTER_TO_FOOT'

    
    referenceConfig_asymetric =[0.,0.,0.461, 0.,0.,0.,1., # FF
        0.0, 0.611, -1.0452,
        0.0, -0.853, 1.0847,
        -0.0, 0.74, -1.08,
        -0.0, -0.74, 1.08,
        ]
    
    """
    referenceConfig=[0,0,0.448,0,0,0,1,
        0.079,0.78,-1.1,
        0.079,-0.78,1.1,
        -0.079,0.78,-1.1,
        -0.079,-0.78,1.1
       ]
    """    
    """
    referenceConfig=[0,0,0.448,0,0,0,1,
        0.095,0.76,-1.074,
        0.095,-0.76,1.074,
        -0.095,0.76,-1.074,
        -0.095,-0.76,1.074
       ]
    """
    referenceConfig=[0,0,0.47,0,0,0,1,
        -0.12,0.724,-1.082,
        -0.12,-0.724,1.082,
        0.12,0.724,-1.082,
        0.12,-0.724,1.082
       ]
    postureWeights=[0,0,0,0,0,0, #FF
    100.,1.,1.,
    100.,1.,1.,
    100.,1.,1.,
    100.,1.,1.,]

    # informations required to generate the limbs databases the limbs : 
    nbSamples = 50000
    octreeSize = 0.002
    cType = "_3_DOF"
    offset = [0.,0.,-0.005] # was 0.005

    rLegLimbOffset = [0.373, 0.264, 0.]
    lLegLimbOffset = [0.373, -0.264,0.]
    rArmLimbOffset = [-0.373, 0.264, 0.]
    lArmLimbOffset = [-0.373, -0.264, 0.]
    normal = [0,0,1]
    legx = 0.02; legy = 0.02
    kinematicConstraintsPath="package://anymal-rbprm/com_inequalities/"

    minDist = 0.2

    # data used by scripts :,,,
    #limbs_names = [rArmId,lLegId,lArmId,rLegId] # reverse default order to try to remove contacts at the beginning of the contact plan
    #limbs_names = [lLegId,rArmId,rLegId,lArmId] # default order to try to remove contacts at the beginning of the contact plan
    limbs_names = [rArmId,rLegId,lArmId,lLegId]
    dict_limb_rootJoint = {rLegId:rleg, lLegId:lleg, rArmId:rarm, lArmId:larm}
    dict_limb_joint = {rLegId:rfoot, lLegId:lfoot, rArmId:rhand, lArmId:lhand}
    dict_limb_color_traj = {rfoot:[0,1,0,1], lfoot:[1,0,0,1],rhand:[0,0,1,1],lhand:[0.9,0.5,0,1]}
    FOOT_SAFETY_SIZE = 0.01
    # size of the contact surface (x,y)
    dict_size={rfoot:[0.01 , 0.01], lfoot:[0.01 , 0.01],rhand:[0.01 , 0.01],lhand:[0.01 , 0.01]}
    #dict_size={rfoot:[0.01 , 0.01], lfoot:[0.01 , 0.01],rhand:[0.01 , 0.01],lhand:[0.01 , 0.01]}
    #various offset used by scripts
    MRsole_offset = SE3.Identity()
    MRsole_offset.translation = np.matrix(offset).T
    MLsole_offset = MRsole_offset.copy()
    MRhand_offset = MRsole_offset.copy()
    MLhand_offset = MRsole_offset.copy()
    dict_offset = {rfoot:MRsole_offset, lfoot:MLsole_offset, rhand:MRhand_offset, lhand:MLhand_offset}
    dict_limb_offset= {rLegId:rLegLimbOffset, lLegId:lLegLimbOffset, rArmId:rArmLimbOffset, lArmId:lArmLimbOffset}
    dict_normal = {rfoot:normal, lfoot:normal, rhand:normal, lhand:normal}
    # display transform :
    MRsole_display = SE3.Identity()
    MLsole_display = SE3.Identity()
    MRhand_display = SE3.Identity()
    MLhand_display = SE3.Identity()
    dict_display_offset = {rfoot:MRsole_display, lfoot:MLsole_display, rhand:MRhand_display, lhand:MLhand_display}

    kneeIds = {"LF":9,"LH":12,"RF":15,"RH":18}

    def __init__ (self, name = None,load = True):
        Parent.__init__ (self,load)
        if load:
            self.loadFullBodyModel(self.urdfName, self.rootJointType, self.meshPackageName, self.packageName, self.urdfSuffix, self.srdfSuffix)
        if name != None:
            self.name = name
        # save original bounds of the urdf for futur reset
        self.LF_HAA_bounds = self.getJointBounds('LF_HAA')
        self.LF_HFE_bounds = self.getJointBounds('LF_HFE')
        self.LF_HAA_bounds = self.getJointBounds('LF_KFE')

        self.RF_HAA_bounds = self.getJointBounds('RF_HAA')
        self.RF_HFE_bounds = self.getJointBounds('RF_HFE')
        self.RF_HAA_bounds = self.getJointBounds('RF_KFE')

        self.LH_HAA_bounds = self.getJointBounds('LH_HAA')
        self.LH_HFE_bounds = self.getJointBounds('LH_HFE')
        self.LH_HAA_bounds = self.getJointBounds('LH_KFE')

        self.RH_HAA_bounds = self.getJointBounds('RH_HAA')
        self.RH_HFE_bounds = self.getJointBounds('RH_HFE')
        self.RH_HAA_bounds = self.getJointBounds('RH_KFE')

    def loadAllLimbs(self,heuristic, analysis = None, nbSamples = nbSamples, octreeSize = octreeSize,disableEffectorCollision = False):
        if isinstance(heuristic,basestring):#only one heuristic name given assign it to all the limbs
            dict_heuristic = {}
            for id in self.limbs_names:
                dict_heuristic.update({id:heuristic})
        elif isinstance(heuristic,dict):
            dict_heuristic=heuristic
        else : 
            raise Exception("heuristic should be either a string or a map limbId:string")
        #dict_heuristic = {self.rLegId:"static", self.lLegId:"static", self.rArmId:"fixedStep04", self.lArmId:"fixedStep04"}
        for id in self.limbs_names:
            print "add limb : ",id
            eff = self.dict_limb_joint[id]
            print "effector name = ",eff
            self.addLimb(id,self.dict_limb_rootJoint[id],eff,self.dict_offset[eff].translation.T.tolist()[0],self.dict_normal[eff],self.dict_size[eff][0]/2.,self.dict_size[eff][1]/2.,nbSamples,dict_heuristic[id],octreeSize,self.cType,disableEffectorCollision = disableEffectorCollision,kinematicConstraintsPath=self.kinematicConstraintsPath+self.dict_limb_rootJoint[id]+"_06_com_constraints.obj",limbOffset=self.dict_limb_offset[id],kinematicConstraintsMin=self.minDist)
            if analysis :
                self.runLimbSampleAnalysis(id, analysis, True)

    def setConstrainedJointsBounds(self):
        self.setJointBounds('LF_HAA',[-1.,1.])
        self.setJointBounds('LF_HFE',[-0.25,2.35])
        self.setJointBounds('LF_KFE',[-2.35,0.])

        self.setJointBounds('RF_HAA',[-1.,1.])
        self.setJointBounds('RF_HFE',[-0.4,2.35])
        self.setJointBounds('RF_KFE',[-2.35,0.])

        self.setJointBounds('LH_HAA',[-1.,1.])
        self.setJointBounds('LH_HFE',[-2.35,0.4])
        self.setJointBounds('LH_KFE',[0.,2.35])

        self.setJointBounds('RH_HAA',[-1.,1.])
        self.setJointBounds('RH_HFE',[-2.35,0.25])
        self.setJointBounds('RH_KFE',[0.,2.35])


    def setVeryConstrainedJointsBounds(self):
        self.setJointBounds('LF_HAA',[-0.4,0.4])
        self.setJointBounds('LF_HFE',[0.2,0.95])
        self.setJointBounds('LF_KFE',[-2.35,0.])

        self.setJointBounds('RF_HAA',[-0.4,0.4])
        self.setJointBounds('RF_HFE',[0.2,0.95])
        self.setJointBounds('RF_KFE',[-2.35,0.])

        self.setJointBounds('LH_HAA',[-0.4,0.4])
        self.setJointBounds('LH_HFE',[-1.,-0.5])
        self.setJointBounds('LH_KFE',[0.,2.35])

        self.setJointBounds('RH_HAA',[-0.4,0.4])
        self.setJointBounds('RH_HFE',[-1.,-0.5])
        self.setJointBounds('RH_KFE',[0.,2.35])


    def resetJointsBounds(self):
        self.setJointBounds('LF_HAA',self.LF_HAA_bounds)
        self.setJointBounds('LF_HFE',self.LF_HFE_bounds)
        self.setJointBounds('LF_KFE',self.LF_HAA_bounds)

        self.setJointBounds('RF_HAA',self.RF_HAA_bounds)
        self.setJointBounds('RF_HFE',self.RF_HFE_bounds)
        self.setJointBounds('RF_KFE',self.RF_HAA_bounds)

        self.setJointBounds('LH_HAA',self.LH_HAA_bounds)
        self.setJointBounds('LH_HFE',self.LH_HFE_bounds)
        self.setJointBounds('LH_KFE',self.LH_HAA_bounds)

        self.setJointBounds('RH_HAA',self.RH_HAA_bounds)
        self.setJointBounds('RH_HFE',self.RH_HFE_bounds)
        self.setJointBounds('RH_KFE',self.RH_HAA_bounds)


        
