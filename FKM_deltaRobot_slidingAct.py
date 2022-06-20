from logging import root
from logging.config import listen
import math
from pyexpat import model
from re import template
from unittest import case
import Sofa
import numpy
from splib3.constants import Key
from splib3.numerics import vec3
from splib3.numerics.quat import Quat
from splib3.numerics import to_radians
from math import sin, cos, sqrt
from stlib3.physics.constraints import *
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.mixedmaterial import Rigidify
from splib3.objectmodel import SofaPrefab, SofaObject, setData
from stlib3.components import addOrientedBoxRoi
from BenActuatedArm import ActuatedArm
from elasticbody import ElasticBody
from blueprint import Blueprint
from splib3.animation import AnimationManager, animate
from stlib3.scene.contactheader import ContactHeader
from stlib3.physics.collision import CollisionMesh

PI = 3.14159265359

meshRobot='Soft_delta_bloc_v1.vtk'


def createScene(rootNode):
    # Root node
    rootNode.dt = 0.01
    rootNode.gravity = [0, 0, 0.0] #[0, 0, -9.81]
    rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels showCollision showVisualModels showForceFields showInteractionForceFields showCollisionModels hideBoundingCollisionModels hideWireframe')
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    #Required plugin
    rootNode.addObject('RequiredPlugin', pluginName='SofaGeneralRigid SofaLoader SofaRigid SofaMiscMapping SofaConstraint SofaBoundaryCondition SofaDeformable SofaEngine SoftRobots SofaPython3 SofaPreconditioner SofaSparseSolver SofaValidation SofaOpenglVisual SofaGeneralAnimationLoop SofaMiscCollision SofaMeshCollision'+
    ' Sofa.Component.IO.Mesh Sofa.Component.Mass Sofa.Component.SceneUtility Sofa.Component.SolidMechanics.FEM.Elastic Sofa.Component Sofa.Component.StateContainer Sofa.Component.Topology.Container.Dynamic Sofa.Component.Visual SofaGeneralEngine Sofa.GL.Component.Rendering3D '+
    'Sofa.Component.LinearSolver.Iterative Sofa.Component.Mapping.MappedMatrix Sofa.Component.ODESolver.Backward' )
    rootNode.addObject('DefaultVisualManagerLoop')
    # rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('FreeMotionMasterSolver')
    rootNode.addObject('QPInverseProblemSolver', name="QP", printLog='0')
    AnimationManager(rootNode)
    
    # rootNode.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
###################################################################################################################################
### Mechanical structure ####
    modelling = rootNode.addChild("Modelling")
    elasticobject = ElasticMaterialObject(volumeMeshFileName=meshRobot, translation=[0.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.0],
                                          youngModulus=1.80*1e3, poissonRatio=0.45, totalMass=0.50, surfaceColor=[0.0,0.8,0.7], solverName="") #definition of the elastic component using silicon rubber
    modelling.addChild(elasticobject)
    modelling.init()
### Rigidication of the elastic Object which correspond to the clamping parts for creating the robot    
### To do so First Box need to be create to identified the indices of the elastic body that need to be rigidified
    selection  = modelling.addChild("Selection")
    groupeInd=[]
    commandBox = addOrientedBoxRoi(selection, position=[list(i) for i in elasticobject.dofs.rest_position.value],
                            name="BoxROI1", translation=[9.386,0.0,8.61], eulerRotation=[00.0,-45.0,0.0], scale=[2, 6, 2])
    commandBox.init()
    groupeInd.append([ind for ind in commandBox.indices.value]) # indices inditified from the box 1
    commandBox2 = addOrientedBoxRoi(selection, position=[list(i) for i in elasticobject.dofs.rest_position.value],
                            name="BoxROI2", translation=[-4.7,7.7,8.61], eulerRotation=[0.0,-45.0,120.0], scale=[2, 6, 2])
    commandBox2.init()
    groupeInd.append([ind for ind in commandBox2.indices.value])# indices inditified from the box 2
    commandBox3 = addOrientedBoxRoi(selection, position=[list(i) for i in elasticobject.dofs.rest_position.value],
                            name="BoxROI3", translation=[-4.7,-7.7,8.61], eulerRotation=[0.0, -45.0, 240.0], scale=[2, 6, 2])
    commandBox3.init()
    groupeInd.append([ind for ind in commandBox3.indices.value])# indices inditified from the box 3

### Constraints added to restrain the robot's movement (simulation of the 2 of 3 actuators fixed )
    elasticobject.addObject('FixedConstraint', name="FC2", indices=commandBox2.indices.value)
    elasticobject.addObject('FixedConstraint', name="FC3", indices=commandBox3.indices.value)
    elasticobject.addObject('PartialFixedConstraint', indices=commandBox.indices.value, fixedDirections='0 1 1 1 1 1')

### Rigidification of the center of the robot using a sphereDOI to link the border points to one unique frame.
    o = elasticobject.addObject('SphereROI', name='SphereRoi', template='Rigid3',centers=[0.0,0.0,0.5], radii=[1.2], drawSphere=True)
    o.init()
    groupeInd.append(list(o.indices.value))
    groupeFrames=[10.0,0.0,8.61, 00.0,-45.0,0.0],[-4.9,8.6,8.61, 0.0,-45.0,120.0],[-4.9,-8.6,8.61, 0.0, -45.0, 240.0],[0., 0., 0.,  0.,0.,0.,1.] # Frames associated to the different boxes
### Rigidification using the frame and the indices gathered thanks to the three boxes
    rigidifiedstruct = Rigidify(modelling, elasticobject, groupIndices=groupeInd, frames=groupeFrames, name="RigidifiedStructure")
    rigidifiedstruct.DeformableParts.addObject("UncoupledConstraintCorrection") # Additional solver condition to reduce computing errors
    rigidifiedstruct.RigidParts.RigidifiedParticules.addObject("UncoupledConstraintCorrection")# Additional solver condition to reduce computing errors
    rigidifiedstruct.RigidParts.addObject("UncoupledConstraintCorrection")# Additional solver condition to reduce computing errors
### Visual to render the rigidification of the elastic element
    setData(rigidifiedstruct.RigidParts.RigidifiedParticules.dofs, showObject=True, showObjectScale=0.1, drawMode=1, showColor=[1., 1., 0., 1.])
    setData(rigidifiedstruct.DeformableParts.dofs, showObject=True, showObjectScale=0.1, drawMode=2)
    setData(rigidifiedstruct.RigidParts.dofs, showObject=True, showObjectScale=1, drawMode=1)
####################################################################################################################################

###  Motor Management ####
### motors creation, placement and definition of the maximal and the minimal range.
    forceActuator=20e3
    SetDepAct = 0.250
    depAct    = 10
    arm1 = elasticobject.addChild('Arm1')
    arm1.addObject('MechanicalObject', template='Rigid3', position=rigidifiedstruct.RigidParts.dofs.position[0])
    arm1.addObject('SlidingActuator',name='Act1',template='Rigid3d', indices='0', direction='1 0 0 0 0 0', maxForce=forceActuator, minForce=-forceActuator, maxDispVariation=SetDepAct, maxPositiveDisp=depAct,maxNegativeDisp=depAct)
    # arm1.addObject('RigidRigidMapping', index=0,globalToLocalCoords=True)
    arm1.addObject('BarycentricMapping', mapForces="false", mapMasses="false")
    # 
    simulationNode = rootNode.addChild("Simulation")
    simulationNode.addObject("EulerImplicitSolver",name="TimeIntegrationSchema")
    simulationNode.addObject("CGLinearSolver", iterations=50, tolerance=1e-5, threshold=1e-05)
    simulationNode.addChild(rigidifiedstruct)
    simulationNode.addChild(arm1)

### Additional forces, momentum and environmental effet
    # rigidifiedstruct.RigidParts.RigidifiedParticules.addObject("ConstantForceField",name="cff",force='0.0 0.0 0.0', showArrowSize=0.03) # force used to compute the stiffness
    # rigidifiedstruct.RigidParts.RigidifiedParticules.addObject("DiagonalVelocityDampingForceField",name="dvdff",dampingCoefficient='0.1 0.1 0.1') # additional damping to compute the stiffness.
### Add a free center for goal gestion.
    freeCenter = rigidifiedstruct.addChild('FreeCenter')
    freeCenter.addObject('MechanicalObject', name="dofs", template="Rigid3", position=[0., 0., 0., 0., 0., 0., 1.], showObject=False, showObjectScale=0.5)
    freeCenter.addChild(rigidifiedstruct.RigidParts)
                        
### Controller of the DeltaRobot    
    modelling.addObject(DeltaController(node=rootNode)) #

### Additional lines to solve modeling problem that doesn't work because of the difference of the structure of my programming
    modelling.addObject('MechanicalMatrixMapper',
                                 name        = "deformableAndFreeCenterCoupling",
                                 template    = 'Vec3,Rigid3',
                                 object1     = rigidifiedstruct.DeformableParts.dofs.getLinkPath(),
                                 object2     = rigidifiedstruct.FreeCenter.dofs.getLinkPath(),
                                 nodeToParse = rigidifiedstruct.DeformableParts.ElasticMaterialObject.getLinkPath())
    return rootNode

class DeltaController(Sofa.Core.Controller):

    def __init__(self, *a, **kw):
        print('---------- Entering init ----------')
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        self.stepsize = 0.1

    def onKeypressedEvent(self, event):
        key = event['key']

        if key == Key.uparrow:
            # self.node.Modelling.Arm1.Act1.displacement = self.node.Modelling.Arm1.Act1.displacement.value + self.stepsize
            self.node.Modelling.ElasticMaterialObject.Arm1.Act1.displacement = self.node.Modelling.ElasticMaterialObject.Arm1.Act1.displacement.value + self.stepsize
        elif key == Key.downarrow:
            # self.node.Modelling.Arm1.Act1.displacement = self.node.Modelling.Arm1.Act1.displacement.value - self.stepsize
            self.node.Modelling.ElasticMaterialObject.Arm1.Act1.displacement = self.node.Modelling.ElasticMaterialObject.Arm1.Act1.displacement.value - self.stepsize
            

        # if key == Key.leftarrow:
        #     self.actuators[1].ServoMotor.angleIn = self.actuators[1].ServoMotor.angleOut.value + self.stepsize
        # elif key == Key.rightarrow:
        #     self.actuators[1].ServoMotor.angleIn = self.actuators[1].ServoMotor.angleOut.value - self.stepsize

        # if key == Key.plus:
        #     self.actuators[2].ServoMotor.angleIn = self.actuators[2].ServoMotor.angleOut.value + self.stepsize
        # elif key == Key.minus:
        #     self.actuators[2].ServoMotor.angleIn = self.actuators[2].ServoMotor.angleOut.value - self.stepsize

        return 0