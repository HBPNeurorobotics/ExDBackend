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
This module contains the abstract base class of a simulation assembly using the CLE and Gazebo
"""

import logging
import os
import sys
import subprocess

from RestrictedPython import compile_restricted
from hbp_nrp_backend import NRPServicesGeneralException
from hbp_nrp_backend.storage_client_api.StorageClient import StorageClient, Model
from hbp_nrp_commons.sim_config.SimConfig import ResourceType
from hbp_nrp_commons.workspace.SimUtil import SimUtil
from hbp_nrp_cleserver.server.GazeboSimulationAssembly import GazeboSimulationAssembly
from hbp_nrp_cle.externalsim.ExternalModuleManager import ExternalModuleManager
from hbp_nrp_commons.ZipUtil import ZipUtil
from hbp_nrp_backend import get_date_and_time_string

# These imports start NEST.
from hbp_nrp_cleserver.server.ROSCLEServer import ROSCLEServer
from hbp_nrp_cle.cle.ClosedLoopEngine import DeterministicClosedLoopEngine, ClosedLoopEngine
import hbp_nrp_cle.tf_framework as tfm
import hbp_nrp_cle.brainsim.config as brainconfig
from cle_ros_msgs import srv
from hbp_nrp_cle.cle.DeterministicClosedLoopEngineProfiler import \
    DeterministicClosedLoopEngineProfiler

logger = logging.getLogger(__name__)


class CLEGazeboSimulationAssembly(GazeboSimulationAssembly):
    """
    This class assembles the simulation using the CLE
    """

    def __init__(self, sim_config):
        """
        Creates a new simulation assembly to simulate an experiment using the CLE and Gazebo
        :param sim_config: config of the simulation to be managed
        """
        super(CLEGazeboSimulationAssembly, self).__init__(sim_config)
        self.cle_server = None
        self.simAssetsDir = os.path.join(sim_config.sim_dir, 'assets')
        self._simResourcesDir = os.path.join(sim_config.sim_dir, 'resources')
        self._simProfilerDir = ''
        self._sim_profiler_mode = sim_config.profiler

        self._storageClient = StorageClient()
        self._storageClient.set_sim_dir(sim_config.sim_dir)

    @property
    def storage_client(self):
        """
        Gets the storage client handler
        """
        return self._storageClient

    def _initialize(self, except_hook):
        """
        Internally initialize the simulation
        :param environment: The environment that should be simulated
        :param except_hook: A method that should be called when there is a critical error
        """
        # pylint: disable=too-many-locals

        # create the CLE server and lifecycle first to report any failures properly
        # initialize the cle server and services
        logger.info("Creating CLE Server")
        self.cle_server = ROSCLEServer(self.sim_config.sim_id,
                                       self.sim_config.timeout,
                                       self.sim_config.timeout_type,
                                       self.gzserver,
                                       self.ros_notificator)

        self.cle_server.setup_handlers(self)
        # Put the resources folder into the sys path for import
        try:
            SimUtil.makedirs(self._simResourcesDir)
            with open(os.path.join(self._simResourcesDir, '__init__.py'), 'w+'):
                pass  # make sure the __init__.py exists
        except IOError as err:
            logger.info("Failed to setup resource directory due to {err}".format(err=err))
        sys.path.insert(0, self._simResourcesDir)

        # start Gazebo simulator and bridge
        self._start_gazebo(extra_models=self.simAssetsDir + ':' + self.sim_dir)

        # load user textures in Gazebo
        self._load_textures()

        # load environment and robot models
        models, lights = self._load_environment(self.sim_config.world_model.resource_path.abs_path)

        # find robot
        self.robotManager.set_robot_dict(self.sim_config.robot_models)
        self._load_robot()

        robot_poses = {}
        for rid, robot in self.robotManager.get_robot_dict().iteritems():
            robot_poses[rid] = robot.pose

        # load robot adapters
        robotcomm, robotcontrol = self._create_robot_adapters()

        # load the brain
        braincontrol, braincomm, brainfile, brainconf = self._load_brain()

        # load external modules
        external_module_manager = ExternalModuleManager()

        # initialize the cle server and services
        logger.info("Preparing CLE Server")
        self.cle_server.cle = self.__load_cle(robotcontrol, robotcomm, braincontrol, braincomm,
                                              brainfile, brainconf, external_module_manager,
                                              robot_poses, models, lights)
        self.cle_server.prepare_simulation(except_hook)

        # load transfer functions
        self.__load_tfs()

        # Wait for the backend rendering environment to load (for any sensors/cameras)
        self._notify("Waiting for Gazebo simulated sensors to be ready")
        self.robotManager.scene_handler().wait_for_backend_rendering()
        # Spawns a new thread for the csv logger
        # pylint: disable=protected-access
        self.cle_server._csv_logger.initialize()

    def _prepare_simconfig_robots(self):
        """
        Reads robot list from bibi and poses from exc and populates robot manager
        :return: -
        """
        # pylint: disable=too-many-branches
        if not self.sim_config.robot_models:
            return

        for robot in self.sim_config.robot_models.values():
            if robot.isCustom:
                # pylint: disable=protected-access
                status, ret = self.cle_server._robotHandler.prepare_custom_robot(robot.model)
                if not status:
                    raise Exception("Could not prepare custom robot {err}".format(err=ret))
                sdf_abs_path = ret

            else:
                sdf_abs_path = SimUtil.find_file_in_paths(
                    os.path.join(robot.id, os.path.basename(robot.SDFFileAbsPath)),
                    self.sim_config.model_paths)

                # Perhaps it's a previously coned experiment? Try with modelTag.value() BUT
                # only look into the simulation_directory, as DELETE robot REST call, if called,
                # would delete this file. Only for the exps without robotid folder in the storage
                # TODO: backward compatibility code. Remove when we decide not to support anymore
                if not sdf_abs_path:
                    sdf_abs_path = SimUtil.find_file_in_paths(robot.SDFFileAbsPath, [self.sim_dir])

            # still couldn't find the SDF, abort!
            if not sdf_abs_path:
                raise Exception("Could not find robot file: {0}".format(robot.SDFFileAbsPath))

            robot.SDFFileAbsPath = sdf_abs_path

            # Find robot specific roslaunch file in the directory where the SDF resides
            # Take the first one (by name) if multiple available
            rosLaunchRelPath = next((f for f in os.listdir(os.path.dirname(robot.SDFFileAbsPath))
                                     if f.endswith('.launch')), None)
            robot.rosLaunchAbsPath = (None if rosLaunchRelPath is None
                                      else os.path.join(os.path.dirname(robot.SDFFileAbsPath),
                                                        rosLaunchRelPath))

    def _load_environment(self, world_file_abs_path):
        """
        Loads the environment and robot in Gazebo

        :param world_file_abs_path Path to the world sdf
        """
        self._notify("Loading experiment environment")
        return self.robotManager.scene_handler().parse_gazebo_world_file(world_file_abs_path)

    def _load_textures(self):
        """
        Loads custom textures in Gazebo
        """
        self._notify("Loading textures")

        try:
            textures = self.storage_client.get_textures(
                self.sim_config.experiment_id, self.sim_config.token)
        except:  # pylint: disable=bare-except
            logger.info("Non-existent textures or folder!")
            return  # ignore missing textures or texture folder

        try:
            if textures:
                self.robotManager.scene_handler().load_textures(textures)
        except:  # pylint: disable=bare-except
            logger.info("Timeout while trying to load textures.")

    # pylint: disable-msg=too-many-branches
    def _load_robot(self):
        """
        Loads robots defined in the bibi and initializes any external controller
        """
        # Set retina config for the robotManager
        if self.sim_config.retina_config:
            self._notify("Configuring Retina Camera Plugin")
            self.robotManager.retina_config = self.sim_config.retina_config

        self._notify("Loading robots")
        self._prepare_simconfig_robots()
        for robot in self.robotManager.get_robot_dict().values():
            self.robotManager.initialize(robot)

        # load external robot controller
        if self.sim_config.ext_robot_controller is not None:
            robot_controller_filepath = SimUtil.find_file_in_paths(
                self.sim_config.ext_robot_controller, self.sim_config.model_paths)
            if not os.path.isfile(robot_controller_filepath) and self.sim_dir is not None:
                robot_controller_filepath = os.path.join(self.sim_dir,
                                                         self.sim_config.ext_robot_controller)
            if os.path.isfile(robot_controller_filepath):
                self._notify("Loading external robot controllers")  # +1
                res = subprocess.call([robot_controller_filepath, 'start'])
                if res > 0:
                    logger.error("The external robot controller could not be loaded")
                    self.shutdown()
                    return

    def _create_robot_adapters(self):  # pragma: no cover
        """
        Creates the adapter components for the robot side

        :return: A tuple of the communication and control adapter for the robot side
        """
        raise NotImplementedError("This method must be overridden in an implementation")

    def _load_brain(self):
        """
        Loads the neural simulator, interfaces, and configuration
        """

        # Create interfaces to brain
        self._notify("Loading neural simulator")
        brainconfig.rng_seed = self.rng_seed
        braincomm, braincontrol = self._create_brain_adapters()

        self._notify("Loading brain and population configuration")
        if not self.sim_config.brain_model:
            return braincontrol, braincomm, None, None

        if self.sim_config.brain_model.is_custom:
            self._extract_brain_zip()

        brain_abs_path = self.sim_config.brain_model.resource_path.abs_path
        brain_rel_path = self.sim_config.brain_model.resource_path.rel_path
        if not os.path.exists(brain_abs_path):
            logger.info(
                "Cannot find specified brain file {file} in {dir}. Searching in default "
                "directories {default}".format(file=brain_rel_path, dir=self.sim_dir,
                                               default=str(self.sim_config.model_paths)))
            brain_abs_path = SimUtil.find_file_in_paths(brain_rel_path, self.sim_config.model_paths)

            if brain_abs_path:
                self.sim_config.brain_model.resource_path.abs_path = brain_abs_path
            else:
                raise NRPServicesGeneralException(
                    "Could not find brain file: {}".format(brain_rel_path))

        neurons_config = self.sim_config.get_populations_dict()

        return braincontrol, braincomm, brain_abs_path, neurons_config

    def _extract_brain_zip(self):
        """
        Checks for validity, and extracts a zipped brain. First we
        make sure that the zip referenced in the bibi exists in the
        list of user brains, then we unzip it on the fly in the temporary
        simulation directory. After the extraction we also make sure to copy
        the .py from the experiment folder cause the user may have modified it
        """
        # pylint: disable=too-many-locals
        brain = Model(
            self.sim_config.brain_model.model,
            ResourceType.BRAIN)

        storage_brain_zip_data = self._storageClient.get_model(
            self.sim_config.token,
            self.sim_config.ctx_id, brain)

        if storage_brain_zip_data:
            # Get the data
            # Write the zip in sim dir
            zip_model_path = self._storageClient.get_model_path(
                self.sim_config.token,
                self.sim_config.ctx_id,
                brain)

            brain_abs_zip_path = os.path.join(self.sim_dir, zip_model_path)

            if not os.path.exists(os.path.dirname(brain_abs_zip_path)):
                os.makedirs(os.path.dirname(brain_abs_zip_path))

            with open(brain_abs_zip_path, 'w') as brain_zip:
                brain_zip.write(storage_brain_zip_data)
            # Extract and flatten
            # FixME: not sure exactly why flattening is required
            ZipUtil.extractall(zip_abs_path=self.sim_config.brain_model.zip_path.abs_path,
                               extract_to=self.sim_dir, overwrite=False, flatten=True)

            # copy back the .py from the experiment folder, cause we don't want the one
            # in the zip, cause the user might have made manual changes
            # TODO: verify if this still required and why only one file is copied
            brain_name = os.path.basename(self.sim_config.brain_model.resource_path.rel_path)
            self._storageClient.clone_file(
                brain_name, self.sim_config.token, self.sim_config.experiment_id)

        # if the zip is not there, prompt the user to check his uploaded models
        else:
            raise NRPServicesGeneralException(
                "Could not find selected brain model {name} in the list of uploaded models. "
                "Please make sure that it has been uploaded correctly".format(
                    name=self.sim_config.brain_model.model),
                "Zipped model retrieval failed")

    def _create_brain_adapters(self):  # pragma: no cover
        """
        Creates the adapter components for the neural simulator

        :return: A tuple of the communication and control adapter for the neural simulator
        """
        raise NotImplementedError("This method must be overridden in an implementation")

    # pylint: disable=too-many-arguments
    def __load_cle(self, roscontrol, roscomm, braincontrol, braincomm,
                   brain_file_path, neurons_config, external_module_manager,
                   robot_poses, models, lights):
        """
        Load the ClosedLoopEngine and initializes all interfaces

        :param roscontrol Robot Control Adapter to use
        :param roscomm Robot Communication Adapter to use
        :param braincontrol Brain Control Adapter to use
        :param braincomm Brain Communication Adapter to use
        :param brain_file_path Accessible path to brain file
        :param neurons_config Neuron configuration specified in the BIBI
        :param external_module_manager External IBA Module Manager
        :param robot_post Initial robot pose
        :param models Initial models loaded into the environment
        :param lights Initial lights loaded into the environment
        """

        # Needed in order to cleanup global static variables
        self._notify("Connecting brain simulator to robot")
        tfm.start_new_tf_manager()

        # Create transfer functions manager
        tfmanager = tfm.config.active_node

        # set adapters
        tfmanager.robot_adapter = roscomm
        tfmanager.brain_adapter = braincomm

        # integration timestep between simulators, convert from ms to s (default to CLE value)
        timestep = (ClosedLoopEngine.DEFAULT_TIMESTEP
                    if self.sim_config.timestep is None else self.sim_config.timestep)

        roscontrol.set_robots(self.robotManager.get_robot_dict())

        # initialize CLE
        self._notify("Initializing CLE")

        if self._sim_profiler_mode == srv.CreateNewSimulationRequest.PROFILER_CLE_STEP or \
                self._sim_profiler_mode == srv.CreateNewSimulationRequest.PROFILER_CPROFILE:
            # creates a folder in sim_dir to store profiler data and passes it to the CLE
            self._simProfilerDir = '_'.join(['profiler_data', get_date_and_time_string()])
            profile_tmp_dir = str(os.path.join(self.sim_dir, self._simProfilerDir))
            SimUtil.makedirs(profile_tmp_dir)

            cle = DeterministicClosedLoopEngineProfiler(roscontrol, roscomm,
                                                        braincontrol, braincomm, tfmanager,
                                                        external_module_manager, timestep,
                                                        self._sim_profiler_mode,
                                                        profile_tmp_dir)
        else:
            cle = DeterministicClosedLoopEngine(roscontrol, roscomm,
                                                braincontrol, braincomm,
                                                tfmanager, external_module_manager,
                                                timestep)

        if brain_file_path:
            cle.initialize(brain_file_path, **neurons_config)
        else:
            cle.initialize()

        # Set initial pose
        cle.initial_robot_poses = robot_poses
        # Set initial models and lights
        cle.initial_models = models
        cle.initial_lights = lights

        return cle

    def __load_tfs(self):
        """
        Loads and connects all transfer functions
        """
        self._notify("Loading transfer functions")

        for tf in self.sim_config.transfer_functions:
            self._notify("Loading transfer function: {}".format(tf.name))
            #tf.code = correct_indentation(tf.code, 0)
            tf.code = tf.code.strip() + "\n"
            logger.debug("TF: " + tf.name + "\n" + tf.code + '\n')

            try:
                new_code = compile_restricted(tf.code, '<string>', 'exec')
            # pylint: disable=broad-except
            except Exception as e:
                logger.error("Error while compiling the transfer function {name} in restricted "
                             "mode with error {err}".format(name=tf.name, err=str(e)))
                tfm.set_flawed_transfer_function(tf.code, tf.name, e)
                continue

            try:
                tfm.set_transfer_function(tf.code, new_code, tf.name, tf.active, tf.priority)
            except tfm.TFLoadingException as loading_e:
                logger.error(loading_e)
                tfm.set_flawed_transfer_function(tf.code, tf.name, loading_e)

    def _handle_gazebo_shutdown(self):
        """
        Handles the case that Gazebo was shut down

        :param close_error: Any exception happened while closing Gazebo
        """
        super(CLEGazeboSimulationAssembly, self)._handle_gazebo_shutdown()
        if self.cle_server is not None and self.cle_server.lifecycle is not None:
            # Set the simulation to halted
            self.cle_server.lifecycle.failed()
            # If not already stopped, free simulation resources
            self.cle_server.lifecycle.stopped()

    def run(self):
        """
        Runs the simulation
        """
        self.cle_server.run()

    def _shutdown(self, notifications):
        """
        Shutdown the CLE and any hooks before shutting down Gazebo

        :param notifications: A flag indicating whether notifications should be attempted to send
        """
        try:
            if notifications:
                self.ros_notificator.update_task(
                    "Shutting down Closed Loop Engine", update_progress=True, block_ui=False)

            self.robotManager.shutdown()
            self.cle_server.shutdown()

            # copy cProfile stats to Storage
            if self._simProfilerDir:
                profile_tmp_dir = str(os.path.join(self.sim_dir, self._simProfilerDir))
                folder_uuid = self.storage_client.create_folder(self.sim_config.token,
                                                                self.sim_config.experiment_id,
                                                                self._simProfilerDir)['uuid']

                for f_name in os.listdir(profile_tmp_dir):
                    f_path = str(os.path.join(profile_tmp_dir, f_name))
                    with open(f_path, 'rb') as f:
                        self.storage_client.create_or_update(
                            self.sim_config.token, folder_uuid, f_name,
                            f.read(), 'application/octet-stream')

        # pylint: disable=broad-except
        except Exception, e:
            logger.error("The cle server could not be shut down")
            logger.exception(e)

        finally:
            # Restore sys path. Make sure that all instances are removed (if present)
            sys.path[:] = (path for path in sys.path if path is not self._simResourcesDir)
