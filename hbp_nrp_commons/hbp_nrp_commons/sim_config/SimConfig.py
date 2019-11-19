# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END

"""
This module represents the configuration of a running simulation
"""

import os
import logging
from enum import Enum
from pyxb import ValidationError, NamespaceError

from hbp_nrp_cle.robotsim.RobotManager import Robot
from hbp_nrp_commons.sim_config.SimConfUtil import SimConfUtil
from hbp_nrp_commons.workspace.Settings import Settings
from hbp_nrp_commons.generated import bibi_api_gen as bibi_parser, exp_conf_api_gen as exc_parser
from hbp_nrp_cleserver.bibi_config.bibi_configuration_script import (get_all_neurons_as_dict)
from hbp_nrp_cleserver.bibi_config.bibi_configuration_script import (generate_tf, get_tf_name)

__author__ = 'Hossain Mahmud'

logger = logging.getLogger('SimConfig')


class Version(Enum):  # pragma: no cover
    """
    Enumeration for exc and bibi version
    """

    CURRENT = "Current"


class SimulationType(Enum):  # pragma: no cover
    """
    Enumeration for different simulation type
    """

    NEST_SYNC = 0x11000001
    NEST_DIST = 0x11000002
    SPINNAKER_SYNC = 0x11000003
    NENGO_SYNC = 0x11000004
    ROBOT_ROS_SYNC = 0x11000006
    NEST_DIRECT_SYNC = 0x11000007


class ResourceType(Enum):  # pragma: no cover
    """
    Enumeration for model types. Robot resources are enhanced in a different structure
    """
    MODEL = 0x11100001
    ROBOT = 0x11100002
    BRAIN = 0x11100003
    ENVIRONMENT = 0x11100005


class ResourcePath(object):  # pragma: no cover
    """
    Path place holder
    """

    def __init__(self, rel_path, located_at=None):
        """
        Initializes a resource path
        :param string rel_path: relative path
        :param string located_at: from where the rel_path is relative
        """
        self.rel_path = rel_path
        self.abs_path = None if located_at is None else os.path.join(located_at, rel_path)


class ExperimentResource(object):  # pragma: no cover
    """
    Generalized model description of experiment resources. Can be BRAIN, WORLD, or any other MODEL
    """

    def __init__(self, resource_type, resource_rel_path,
                 is_custom=False, model_name=None, located_at=None):
        """
        Initializes an experiment resource

        :param string resource_type: type of the resource
        :param string resource_rel_path: relative path of the resource
        :param bool is_custom: True if the resource is custom, False otherwise
        :param string model_name: id name in the model DB
        :param string located_at: location of the resources (where relative paths are valid)
        """

        if is_custom and not model_name:
            logger.warning("Cannot create custom resource without zip path")
            return None
        self.resource_type = resource_type
        self.resource_path = ResourcePath(resource_rel_path, located_at)
        self.is_custom = is_custom
        self.model = model_name


class _TF(object):  # pragma: no cover
    """
    Model to store TF information
    """
    def __init__(self, name, code, src=None, active=False):
        """
        Initialize a transfer function object

        :param name: name of the transfer function
        :param code: compiled (restricted) source code of transfer function
        :param src: source code (plain text) of transfer function
        """
        self.name = name
        self.code = code
        self.src = src
        self.active = active


class SimConfig(object):
    """
    Abstraction of exc and bibi configuration
    """
    def __init__(self, exc_abs_path, **params):
        """
        Initialize a sim config object

        :param exc_abs_path: absolute path to experiment configuration (exc) file
        :param par: dictionary of any other required params
        """

        # ground truth
        self._sim_dir = os.path.dirname(exc_abs_path)
        self._exc_path = ResourcePath(os.path.basename(exc_abs_path), self.sim_dir)

        # base assembly
        self._exc_dom = None
        self._bibi_dom = None

        self._sim_id = params.get('sim_id', None)
        self._token = params.get('token')
        self._ctx_id = params.get('ctx_id')

        # gazebo assembly
        self._timeout = params.get('timeout', None)
        self._timeout_type = params.get('timeout_type', None)
        self._gzserver_host = params.get('gzserver_host', 'local')
        self._experiment_id = params.get('experiment_id', None)
        self._reservation = params.get('reservation', None)
        self._rng_seed = params.get('rng_seed', None)

        self._playback_path = params.get('playback_path', None)
        self._physics_engine = None

        self._gzweb = None
        self._ros_launcher = None
        self._gazebo_recorder = None

        # exc info
        self.exc_version = None

        # bibi info
        self._populations_dict = {}
        self._tfs = []

        # paths
        self._bibi_path = None
        self._brain_abs_path = None

        self._robot_models = {}
        self._brain_model = None
        self._world_model = None

        self._simulation_type = None
        self._num_brain_processes = params.get('brain_processes', None)

        # paths from system config
        self._model_paths = [self._sim_dir]

        self.initialize()

    def initialize(self):
        """
        Initialize data members of the sim config
        """
        self._read_exc_and_bibi_dom_objects()
        self._read_dom_data()

        self._model_paths.append(os.path.join(self._sim_dir, 'robots'))
        if Settings.nrp_models_directory is not None:
            self._model_paths.append(Settings.nrp_models_directory)

    def _read_exc_and_bibi_dom_objects(self):
        """
        Parse experiment and bibi and return the DOM objects
        """
        # Read exc
        with open(self._exc_path.abs_path) as excFile:
            try:
                self._exc_dom = exc_parser.CreateFromDocument(excFile.read())
            except ValidationError as ve:
                raise Exception("Could not parse experiment config {0} due to validation "
                                "error: {1}".format(self._exc_path.abs_path, str(ve)))

        self._bibi_path = ResourcePath(self._exc_dom.bibiConf.src, self.sim_dir)
        logger.info("Bibi absolute path:" + self._bibi_path.abs_path)

        # Read bibi
        with open(self._bibi_path.abs_path) as bibiFile:
            try:
                self._bibi_dom = bibi_parser.CreateFromDocument(bibiFile.read())
            except ValidationError as ve:
                raise Exception("Could not parse brain configuration {0:s} due to validation "
                                "error: {1:s}".format(self._bibi_path.abs_path, str(ve)))
            except NamespaceError as ne:
                # first check to see if the BIBI file appears to have a valid namespace
                namespace = str(ne).split(" ", 1)[0]
                if not namespace.startswith("http://schemas.humanbrainproject.eu/SP10") or \
                        not namespace.endswith("BIBI"):
                    raise Exception("Unknown brain configuration file format for: {0:s} with "
                                    "namespace: {1:s}".format(self._bibi_path.abs_path, namespace))

                # notify the user that their file is out of date
                raise Exception("The BIBI file for the requested experiment is out of date and no "
                                "longer supported. Please contact neurorobotics@humanbrainproject."
                                "eu with the following information for assistance in updating this"
                                " file.\n\nExperiment Configuration:\n\tName: {0:s}\n\tBIBI: {1:s}"
                                "\n\tVersion: {2:s}".format(self._exc_dom.name,
                                                            self._bibi_path.abs_path, namespace))

        # set config version based of something
        self.exc_version = Version.CURRENT

    def get_world_model(self):
        """
        it gets the data necessary to create a world model
        """
        is_custom_model = False
        model_name = None
        if self._exc_dom.environmentModel.model:
            is_custom_model = True
            model_name = self._exc_dom.environmentModel.model

        self._world_model = ExperimentResource(
            resource_type=ResourceType.ENVIRONMENT,
            resource_rel_path=self._exc_dom.environmentModel.src,
            is_custom=is_custom_model,
            model_name=model_name,
            located_at=self._sim_dir)

    def get_brain_model(self):
        """
        it gets the data necessary to create the brain model
        """
        is_custom_model = False
        brain_model_name = None
        if self._bibi_dom.brainModel and self._bibi_dom.brainModel.model:
            is_custom_model = True
            brain_model_name = self._bibi_dom.brainModel.model

        if self._bibi_dom.brainModel:
            self._brain_model = ExperimentResource(
                resource_type=ResourceType.BRAIN,
                resource_rel_path=self._bibi_dom.brainModel.file,
                is_custom=is_custom_model,
                model_name=brain_model_name,
                located_at=self.sim_dir)

    def _read_dom_data(self):
        """
        Populate different models
        """

        # if user did not provide the number of brain processes in ROSPY request, use from bibi
        if self._num_brain_processes is None:
            self._num_brain_processes = self._exc_dom.bibiConf.processes
        try:
            self._simulation_type = {
                None: SimulationType.NEST_SYNC,  # default
                bibi_parser.SimulationMode.SynchronousNestSimulation: SimulationType.NEST_SYNC,
                bibi_parser.SimulationMode.SynchronousSpinnakerSimulation:
                    SimulationType.SPINNAKER_SYNC,
                bibi_parser.SimulationMode.SynchronousNengoSimulation: SimulationType.NENGO_SYNC,
                bibi_parser.SimulationMode.SynchronousRobotRosNest: SimulationType.ROBOT_ROS_SYNC,
                bibi_parser.SimulationMode.SynchronousDirectNestSimulation:
                    SimulationType.NEST_DIRECT_SYNC,
            }[self._bibi_dom.mode]
        except KeyError:
            raise Exception("Unsupported multi-process simulation mode requested: {}"
                            .format(str(self._bibi_dom.mode)))

        # Rename multi-process NEST for convenience
        if self._num_brain_processes > 1 and self._simulation_type is SimulationType.NEST_SYNC:
            self._simulation_type = SimulationType.NEST_DIST

        # Environment model
        self.get_world_model()

        # physics engine
        self._physics_engine = (self._exc_dom.physicsEngine
                                if self._exc_dom.physicsEngine is not None else 'ode')
        logger.info("Using physics engine {}".format(self._physics_engine))

        # Robot model. Read all bodyModel tag(s)
        self._read_robot_models()

        # Brain model
        self.get_brain_model()
        # Populations
        if self._bibi_dom.brainModel and self._bibi_dom.brainModel.populations:
            self._populations_dict = get_all_neurons_as_dict(self._bibi_dom.brainModel.populations)

        # Transfer functions
        for _tf in self._bibi_dom.transferFunction:
            code = generate_tf(_tf, self.sim_dir)
            name = get_tf_name(code)
            src = _tf.src if _tf.src else None  # must be not None and not ""
            active = bool(_tf.active) if _tf.active else False
            self._tfs.append(_TF(name, code, src, active))

    def _read_robot_models(self):
        """
        Populate _robot_models. To be used in the robot manager
        """
        for elem in self._bibi_dom.bodyModel:
            if elem.robotId is None:
                elem.robotId = 'robot'
            elif not elem.robotId or elem.robotId in self._robot_models:
                raise Exception("Multiple bodyModels with no or same robot id found")

            pose = None
            is_custom = False
            for rpose in self._exc_dom.environmentModel.robotPose:
                if (not rpose.robotId) or rpose.robotId == elem.robotId:
                    pose = SimConfUtil.convertXSDPosetoPyPose(rpose)

            if hasattr(elem, 'isCustom') and elem.isCustom is not None:
                is_custom = elem.isCustom
            else:
                elem.isCustom = False

            robot_model = None
            if is_custom:
                if not hasattr(elem, "model"):
                    raise Exception("No robotModelName is provided in bibi.bodyModel.model")
                robot_model = elem.model

            # Robot has specialized info. Create direct Robot object instead of ExperimentResource
            self._robot_models[elem.robotId] = Robot(
                rid=elem.robotId,
                # store the relative path for the time being
                sdf_abs_path=None if is_custom else elem.value(),
                display_name=elem.robotId,
                pose=pose,
                is_custom=is_custom,
                roslaunch_abs_path=None,
                model=robot_model)  # gets updated during the Assembly initialization

    def get_populations_dict(self):
        """
        :return: dict containing all the populations
        """
        return self._populations_dict

    def gzbridge_setting(self, name, default):
        """
        Gets the gzbridge setting
        """
        try:
            s = self._exc_dom.gzbridgesettings
            val = getattr(s, name)
            val = type(default)(val)
        # pylint: disable=broad-except
        except Exception:
            val = default
        return repr(val)

    @property
    def exc_dom(self):
        """
        Restrict external access of exc_dom
        :raises exception while accessing DOM object directly
        """
        # pylint: disable=no-self-use
        raise AttributeError("Direct use of exc DOM object inside CLE is forbidden")

    @property
    def bibi_dom(self):
        """
        Restrict external access of bibi_dom
        :raises exception while accessing DOM object directly
        """
        # pylint: disable=no-self-use
        raise AttributeError("Direct use of bibi DOM object inside CLE is forbidden")

    @property
    def token(self):
        """
        Returns the token assigned to this simulation
        """
        return self._token

    @property
    def ctx_id(self):
        """
        Returns the context id assigned to this simulation
        """
        return self._ctx_id

    @property
    def sim_id(self):
        """
        Gets the simulation id
        """
        return self._sim_id

    @property
    def sim_dir(self):
        """
        Gets the simulation directory
        """
        return self._sim_dir

    @property
    def timeout(self):
        """
        Gets the simulation directory
        """
        return self._timeout

    @property
    def timeout_type(self):
        """
        Gets the simulation directory
        """
        return self._timeout_type

    @property
    def exc_path(self):
        """
        Gets the simulation exc file path
        """
        return self._exc_path

    @property
    def bibi_path(self):
        """
        Gets the simulation bibi file ResourcePath
        """
        return self._bibi_path

    @property
    def playback_path(self):
        """
        Gets the playback path
        """
        return self._playback_path

    @property
    def simulation_type(self):
        """
        Gets the playback path
        """
        return self._simulation_type

    @property
    def gzserver_host(self):
        """
        Gets the gzserver host location
        """
        return self._gzserver_host

    @property
    def reservation(self):
        """
        Gets the reservation for cluster (typically a ssh-able hostname, etc.)
        """
        return self._reservation

    @property
    def experiment_id(self):
        """
        Gets the experiment id (typically the folder name in the storage server)
        """
        return self._experiment_id

    @property
    def model_paths(self):
        """
        Gets the model paths
        """
        return self._model_paths

    @property
    def world_model(self):
        """
        Gets the environment model info
        """
        return self._world_model

    @property
    def robot_models(self):
        """
        Gets the robot models
        """
        return self._robot_models

    @property
    def brain_model(self):
        """
        Gets the robot models
        """
        return self._brain_model

    @property
    def transfer_functions(self):
        """
        Gets the list of transfer functions
        """
        return self._tfs

    @property
    def physics_engine(self):
        """
        Gets the physics_engine
        """
        return self._physics_engine

    @property
    def ros_launch_abs_path(self):
        """
        Gets the ros_launch absolute path
        """
        return self._exc_dom.rosLaunch.src if self._exc_dom.rosLaunch else None

    @property
    def ext_robot_controller(self):
        """
        Gets the ext_robot_controller relative path
        """
        return self._bibi_dom.extRobotController

    @property
    def num_brain_processes(self):
        """
        Gets the number of brain processes
        """
        return self._num_brain_processes

    @property
    def rng_seed(self):
        """
        Gets the number of brain processes
        """
        return self._rng_seed if self._rng_seed else self._exc_dom.rngSeed

    @property
    def retina_config(self):
        """
        Gets the retina config file (if provided)
        """
        confs = [conf.src for conf in self._bibi_dom.configuration if conf.type == 'retina']
        if confs:
            return confs[0]

    @property
    def timestep(self):
        """
        Gets the number of brain processes
        """
        return None if self._bibi_dom.timestep is None else float(self._bibi_dom.timestep) / 1000.0
