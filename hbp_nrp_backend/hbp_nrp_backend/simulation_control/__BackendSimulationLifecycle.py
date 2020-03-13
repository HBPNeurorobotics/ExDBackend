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
This module contains the implementation of the backend simulation lifecycle
"""
import os
import io
import rospy
import datetime
import logging
from hbp_nrp_commons.simulation_lifecycle import SimulationLifecycle
from hbp_nrp_commons.generated import exp_conf_api_gen
from hbp_nrp_backend.cle_interface import TOPIC_LIFECYCLE
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClient
from hbp_nrp_backend.cle_interface.PlaybackClient import PlaybackClient
from hbp_nrp_backend.cle_interface.ROSCLESimulationFactoryClient \
    import ROSCLESimulationFactoryClient
from hbp_nrp_backend import NRPServicesGeneralException
from hbp_nrp_backend.__UserAuthentication import UserAuthentication
from hbp_nrp_backend.simulation_control import timezone
from hbp_nrp_commons.sim_config.SimConfig import ResourceType
from hbp_nrp_backend.storage_client_api.StorageClient import StorageClient, Model
from hbp_nrp_cleserver.server.SimulationServer import TimeoutType
from cle_ros_msgs.srv import SimulationRecorderRequest
from hbp_nrp_commons.ZipUtil import ZipUtil
from hbp_nrp_commons.workspace.Settings import Settings
from hbp_nrp_commons.workspace.SimUtil import SimUtil

import tempfile
import time

__author__ = 'Georg Hinkel, Manos Angelidis'


logger = logging.getLogger(__name__)


class BackendSimulationLifecycle(SimulationLifecycle):
    """
    This class implements the backend simulation lifecycle
    """

    def __init__(self, simulation, initial_state='created'):
        """
        Creates a new backend simulation lifecycle

        :param simulation: The simulation for which the simulation lifecycle is created
        """
        super(BackendSimulationLifecycle, self).__init__(TOPIC_LIFECYCLE(simulation.sim_id),
                                                         initial_state)
        self.__simulation = simulation
        self._sim_dir = None
        self.__models_path = Settings.nrp_models_directory
        self.__experiment_path = None
        self.__textures_loaded = False
        self.__storageClient = StorageClient()

    @property
    def simulation(self):
        """
        Gets the simulation for which the lifecycle is controlled
        :return:
        """
        return self.__simulation

    def _load_state_machines(self, exc):  # pragma: no cover
        """
        Parses the experiment configuration, loads state machines

        :param experiment_path: Path the experiment configuration
        """
        state_machine_paths = {}
        if exc.experimentControl is not None and not self.simulation.playback_path:
            state_machine_paths.update({sm.id: os.path.join(self._sim_dir, sm.src)
                                        for sm in
                                        exc.experimentControl.stateMachine
                                        if isinstance(sm, exp_conf_api_gen.SMACHStateMachine)})

        if exc.experimentEvaluation is not None and not self.simulation.playback_path:
            state_machine_paths.update({sm.id: os.path.join(self._sim_dir, sm.src)
                                        for sm in
                                        exc.experimentEvaluation.stateMachine
                                        if isinstance(sm, exp_conf_api_gen.SMACHStateMachine)})

        self.simulation.state_machine_manager.add_all(
            state_machine_paths, self.simulation.sim_id, self.sim_dir)
        self.simulation.state_machine_manager.initialize_all()
        logger.info("Requesting simulation resources")

        return exc

    def _prepare_custom_environment(self, exc):
        """
        Download and extracts zipped environment defined in the exc

        :param exc: The exc DOM object
        """

        # pylint: disable=too-many-locals
        env_model = Model(exc.environmentModel.model, ResourceType.ENVIRONMENT)
        data = self.__storageClient.get_model(
            UserAuthentication.get_header_token(),
            self.simulation.ctx_id,
            env_model
        )

        # if the zip is not there, prompt the user to check his uploaded models
        if not data:
            raise NRPServicesGeneralException(
                "Could not find selected zip {} in the list of uploaded custom models. Please make "
                "sure that it has been uploaded correctly".format(
                    os.path.dirname(exc.environmentModel.model)),
                "Zipped model retrieval failed")

        ZipUtil.extractall(zip_abs_path=io.BytesIO(data),
                           extract_to=os.path.join(self._sim_dir, 'assets'), overwrite=True)

    def initialize(self, state_change):
        """
        Initializes the simulation

        :param state_change: The state change that caused the simulation to be initialized
        """

        simulation = self.simulation
        if not simulation.playback_path:
            self._sim_dir = SimUtil.init_simulation_dir()

        try:
            if not simulation.private:
                raise NRPServicesGeneralException(
                    "Only private experiments are supported", "CLE error", 500)

            self.__storageClient.clone_all_experiment_files(
                token=UserAuthentication.get_header_token(),
                experiment=simulation.experiment_id,
                destination_dir=self._sim_dir,
                exclude=['recordings/'] if not simulation.playback_path else []
            )

            # divine knowledge about the exc name
            self.__experiment_path = os.path.join(self._sim_dir, 'experiment_configuration.exc')

            with open(self.__experiment_path) as exd_file:
                exc = exp_conf_api_gen.CreateFromDocument(exd_file.read())

            self._load_state_machines(exc)
            if exc.environmentModel.model:  # i.e., custom zipped environment
                self._prepare_custom_environment(exc)

            simulation.timeout_type = (TimeoutType.SIMULATION
                                       if exc.timeout.time == TimeoutType.SIMULATION
                                       else TimeoutType.REAL)

            timeout = exc.timeout.value()

            if simulation.timeout_type == TimeoutType.REAL:
                timeout = datetime.datetime.now(timezone) + datetime.timedelta(seconds=timeout)
                simulation.kill_datetime = timeout
            else:
                simulation.kill_datetime = None

            logger.info("simulation timeout initialized")

            simulation_factory_client = ROSCLESimulationFactoryClient()
            simulation_factory_client.create_new_simulation(
                self.__experiment_path,
                simulation.gzserver_host, simulation.reservation, simulation.brain_processes,
                simulation.sim_id, str(timeout), simulation.timeout_type,
                simulation.playback_path,
                UserAuthentication.get_header_token(),
                self.simulation.ctx_id,
                self.simulation.experiment_id,
                self.simulation.profiler
            )
            if not simulation.playback_path:
                simulation.cle = ROSCLEClient(simulation.sim_id)
            else:
                simulation.cle = PlaybackClient(simulation.sim_id)
            logger.info("simulation initialized")

        except IOError as e:
            raise NRPServicesGeneralException(
                "Error while accessing simulation models (" +
                repr(e.message) + ")",
                "Models error")
        except rospy.ROSException as e:
            raise NRPServicesGeneralException(
                "Error while communicating with the CLE (" + repr(e.message) + ")",
                "CLE error")
        except rospy.ServiceException as e:
            raise NRPServicesGeneralException(
                "Error starting the simulation. (" + repr(e.message) + ")",
                "rospy.ServiceException",
                data=e.message)
        # pylint: disable=broad-except
        except Exception as e:
            raise NRPServicesGeneralException(
                "Error starting the simulation. (" + repr(e) + ")",
                "Unknown exception occured",
                data=e.message)

    def start(self, state_change):
        """
        Starts the simulation

        :param state_change: The state change that led to starting the simulation
        """
        logger.info("Starting State Machines...")
        try:
            self.simulation.state_machine_manager.start_all(False)
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Starting state machines failed")
            logger.exception(e)
            # The frontend will be notified of any state machine issues directly
            # over the cle_error topic

    def stop(self, state_change):
        """
        Stops the simulation and releases all resources

        :param state_change: The state change that led to releasing simulation resources
        """

        if self.simulation.cle is not None:

            is_recording = self.simulation.cle.command_simulation_recorder(
                SimulationRecorderRequest.STATE).value

            if is_recording is True:
                self.save_record_to_user_storage()

        self.simulation.kill_datetime = None

        if self.simulation.cle is not None:
            self.simulation.cle.stop_communication(
                "Simulation server released")
        logger.info("State machine outcomes: %s", ", ".join("%s: %s" % (
            sm.sm_id, str(sm.result)) for sm in self.simulation.state_machines))

        self.simulation.state_machine_manager.shutdown()

    def pause(self, state_change):
        """
        Pauses the simulation

        :param state_change: The state change that paused the simulation
        """
        # From the backend side, there is nothing to do because state machines
        # keep running
        pass

    def fail(self, state_change):
        """
        Fails the simulation

        :param state_change: The state change which resulted in failing the simulation
        """
        self.simulation.kill_datetime = None
        if self.simulation.cle is not None:
            self.simulation.cle.stop_communication("Simulation has failed")
        self.simulation.state_machine_manager.terminate_all()

    def reset(self, state_change):
        """
        Resets the simulation

        :param state_change: The state change that led to reseting the simulation
        """
        logger.info("State machine outcomes: %s", ", "
                    .join("%s: %s" % (sm.sm_id, str(sm.result))
                          for sm in self.simulation.state_machines))
        self.simulation.state_machine_manager.terminate_all()

        logger.info("simulation reset")

    def save_record_to_user_storage(self):
        """
        Save the record to user storage

        """

        client_record_folder = 'recordings'
        record_path = self.simulation.cle.command_simulation_recorder(
                           SimulationRecorderRequest.STATE).message

        file_name = 'recording_{timestamp}.{ext}'.format(
            timestamp=time.strftime('%Y-%m-%d_%H-%M-%S'),
            ext='zip')

        temp_dest = os.path.join(tempfile.gettempdir(), file_name)
        ZipUtil.create_from_path(record_path, temp_dest)
        client = StorageClient()

        client.create_folder(self.simulation.token,
                                self.simulation.experiment_id,
                                client_record_folder)

        try:
            with open(temp_dest, 'rb') as record_file:
                zip_data = record_file.read()
                client.create_or_update(
                    self.simulation.token,
                    self.simulation.experiment_id,
                    os.path.join(client_record_folder, file_name),
                    zip_data,
                    "application/octet-stream")

        finally:
            os.remove(temp_dest)

        return file_name

    @property
    def experiment_path(self):
        """
        Gets the experiment_path

        :return: The experiment_path
        """
        return self.__experiment_path

    @experiment_path.setter
    def experiment_path(self, value):
        """
        Sets the experiment_path

        """
        self.__experiment_path = value

    @property
    def sim_dir(self):
        """
        Gets the simulation root folder

        :return: The _sim_dir
        """
        return self._sim_dir
