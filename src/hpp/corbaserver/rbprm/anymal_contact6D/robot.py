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

    packageName = "anymal_description"
    meshPackageName = "anymal_description"
    rootJointType = "freeflyer"
    urdfName = "anymal_reachability"
    urdfSuffix = "_boxFeet"
    srdfSuffix = ""

    ## Information about the names of thes joints defining the limbs of the robot
    rLegId = 'RFleg'
    rleg = 'RF_HAA'
    rfoot = 'RF_CONTACT_3'
    lLegId = 'LFleg'
    lleg = 'LF_HAA'
    lfoot = 'LF_CONTACT_3'
    lArmId = 'LHleg'
    larm = 'LH_HAA'
    lhand = 'LH_CONTACT_3'
    rArmId = 'RHleg'
    rarm = 'RH_HAA'
    rhand = 'RH_CONTACT_3'


    referenceConfig =[0.,0.,0.444, 0.,0.,0.,1., # FF
        0.04, 0.74, -1.08,0.34,-0.04,0.,
        0.04, -0.74, 1.08,-0.34,-0.04,0.,
        -0.04, 0.74, -1.08,0.34,0.04, 0.,
        -0.04, -0.74, 1.08,-0.34,0.04,0.
        ]
    
    reference_weights=[100.,1.,1.,0.,0.,0.]

    # informations required to generate the limbs databases the limbs : 
    nbSamples = 50000
    octreeSize = 0.01
    cType = "_6_DOF"
    offset = [0.,0.,0.]

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
    limbs_names = [rArmId,lArmId,lLegId,rLegId]
    dict_limb_rootJoint = {rLegId:rleg, lLegId:lleg, rArmId:rarm, lArmId:larm}
    dict_limb_joint = {rLegId:rfoot, lLegId:lfoot, rArmId:rhand, lArmId:lhand}
    dict_limb_color_traj = {rfoot:[0,1,0,1], lfoot:[1,0,0,1],rhand:[0,0,1,1],lhand:[0.9,0.5,0,1]}
    FOOT_SAFETY_SIZE = 0.01
    # size of the contact surface (x,y)
    dict_size={rfoot:[0.031 , 0.031], lfoot:[0.031 , 0.031],rhand:[0.031 , 0.031],lhand:[0.031 , 0.031]}
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

    def __init__ (self, name = None,load = True):
        Parent.__init__ (self,load)
        if load:
            self.loadFullBodyModel(self.urdfName, self.rootJointType, self.meshPackageName, self.packageName, self.urdfSuffix, self.srdfSuffix)
        if name != None:
            self.name = name

    def loadAllLimbs(self,heuristic, analysis = None, nbSamples = nbSamples, octreeSize = octreeSize):
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
            print("add limb : ",id)
            eff = self.dict_limb_joint[id]
            print("effector name = ",eff)
            self.addLimb(id,self.dict_limb_rootJoint[id],eff,self.dict_offset[eff].translation.tolist(),self.dict_normal[eff],self.dict_size[eff][0]/2.,self.dict_size[eff][1]/2.,nbSamples,dict_heuristic[id],octreeSize,self.cType,kinematicConstraintsPath=self.kinematicConstraintsPath+self.dict_limb_rootJoint[id]+"06_com_constraints.obj",limbOffset=self.dict_limb_offset[id],kinematicConstraintsMin=self.minDist)
            if analysis :
                self.runLimbSampleAnalysis(id, analysis, True)
        
