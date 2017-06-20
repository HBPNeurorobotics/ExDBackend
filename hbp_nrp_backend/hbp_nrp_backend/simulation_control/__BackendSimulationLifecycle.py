# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
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

from hbp_nrp_commons.simulation_lifecycle import SimulationLifecycle
from hbp_nrp_commons.generated import exp_conf_api_gen
from hbp_nrp_backend.cle_interface import TOPIC_LIFECYCLE
from hbp_nrp_backend.cle_interface.ROSCLEClient import ROSCLEClient
from hbp_nrp_backend.cle_interface.ROSCLESimulationFactoryClient \
    import ROSCLESimulationFactoryClient
import logging
from hbp_nrp_backend import NRPServicesGeneralException
from hbp_nrp_backend.simulation_control import timezone
from flask import request
import os
import shutil
import rospy
import tempfile
import datetime

__author__ = 'Georg Hinkel'


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
        self.__simulation_root_folder = ""
        self.__models_path = os.environ.get('NRP_MODELS_DIRECTORY')
        self.__experiment_path = os.environ.get('NRP_EXPERIMENTS_DIRECTORY')

    @property
    def simulation(self):
        """
        Gets the simulation for which the lifecycle is controlled
        :return:
        """
        return self.__simulation

    def _parse_exp_and_initialize_paths(self, experiment_path, environment_path):
        """
        Parses the experiment configuration, loads state machines and updates the environment path
        with the one in the configuration file if none is passed as an argument, for supporting
        custom environments.

        :param experiment_path: Path the experiment configuration.
        :param environment_path: Path to the environment configuration.
        """
        # parse experiment
        with open(experiment_path) as exd_file:
            experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())

        state_machine_paths = {}
        if experiment.experimentControl is not None:
            state_machine_paths.update({sm.id: os.path.join(self.__simulation_root_folder, sm.src)
                                        for sm in
                                        experiment.experimentControl.stateMachine
                                        if isinstance(sm, exp_conf_api_gen.SMACHStateMachine)})

        if experiment.experimentEvaluation is not None:
            state_machine_paths.update({sm.id: os.path.join(self.__simulation_root_folder, sm.src)
                                        for sm in
                                        experiment.experimentEvaluation.stateMachine
                                        if isinstance(sm, exp_conf_api_gen.SMACHStateMachine)})

        self.simulation.state_machine_manager.add_all(state_machine_paths, self.simulation.sim_id)
        self.simulation.state_machine_manager.initialize_all()
        logger.info("Requesting simulation resources")

        if environment_path is None or environment_path.strip() == '':
            environment_path = os.path.join(self.models_path, str(experiment.environmentModel.src))

        return experiment, environment_path

    def initialize(self, state_change):
        """
        Initializes the simulation

        :param state_change: The state change that caused the simulation to be initialized
        """
        simulation = self.simulation
        try:
            using_collab_storage = simulation.context_id is not None
            if using_collab_storage:
                # TODO: fix dependencies so these import are not necessary anymore
                from hbp_nrp_backend.rest_server.__UserAuthentication import UserAuthentication
                from hbp_nrp_backend.collab_interface.NeuroroboticsCollabClient import \
                NeuroroboticsCollabClient
                client = NeuroroboticsCollabClient(
                    UserAuthentication.get_header_token(request), simulation.context_id)
                collab_paths = client.clone_experiment_template_from_collab()
                self.__experiment_path = collab_paths['experiment_conf']
                self.__simulation_root_folder = os.path.dirname(self.__experiment_path)
                environment_path = collab_paths['environment_conf']
            else:
                self.__experiment_path = os.path.join(
                    self.__experiment_path, simulation.experiment_conf
                )
                logger.info(self.__experiment_path)
                self.__simulation_root_folder = os.path.dirname(self.__experiment_path)
                environment_path = simulation.environment_conf
            experiment, environment_path = \
                self._parse_exp_and_initialize_paths(self.__experiment_path,
                                                     environment_path)

            simulation.kill_datetime = datetime.datetime.now(timezone) \
                + datetime.timedelta(seconds=experiment.timeout)
            logger.info("simulation timeout initialized")

            simulation_factory_client = ROSCLESimulationFactoryClient()
            simulation_factory_client.create_new_simulation(
                environment_path, self.__experiment_path,
                simulation.gzserver_host, simulation.reservation, simulation.brain_processes,
                simulation.sim_id, str(simulation.kill_datetime)
            )
            simulation.cle = ROSCLEClient(simulation.sim_id)
            logger.info("simulation initialized")

        except IOError as e:
            raise NRPServicesGeneralException(
                "Error while accessing simulation models (" + e.message + ")",
                "Models error")
        except rospy.ROSException as e:
            raise NRPServicesGeneralException(
                "Error while communicating with the CLE (" + e.message + ")",
                "CLE error")

    def start(self, state_change):
        """
        Starts the simulation

        :param state_change: The state change that lead to starting the simulation
        """
        logger.info("starting State Machines...")
        try:
            self.simulation.state_machine_manager.start_all()
        # pylint: disable=broad-except
        except Exception, e:
            logger.error("Starting State Machines Failed")
            logger.exception(e)
            # The frontend will be notified of any state machine issues directly
            # over the cle_error topic

    def stop(self, state_change):
        """
        Stops the simulation and releases all resources

        :param state_change: The state change that lead to releasing simulation resources
        """
        if self.simulation.cle is not None:
            self.simulation.cle.stop_communication("Simulation server released")
        logger.info("State machine outcomes: %s", ", ".join("%s: %s" % (sm.sm_id, str(sm.result))
                    for sm in self.simulation.state_machines))

        self.simulation.state_machine_manager.shutdown()
        self.simulation.kill_datetime = None

        using_collab_storage = self.simulation.context_id is not None

        if using_collab_storage:
            path_to_cloned_configuration_folder = os.path.split(self.__experiment_path)[0]
            if tempfile.gettempdir() in path_to_cloned_configuration_folder:
                logger.debug(
                    "removing the temporary configuration folder %s",
                    path_to_cloned_configuration_folder
                )
                shutil.rmtree(path_to_cloned_configuration_folder)

    def pause(self, state_change):
        """
        Pauses the simulation

        :param state_change: The state change that paused the simulation
        """
        self.simulation.state_machine_manager.terminate_all()

    def fail(self, state_change):
        """
        Fails the simulation

        :param state_change: The state change which resulted in failing the simulation
        """
        if self.simulation.cle is not None:
            self.simulation.cle.stop_communication("Simulation has failed")
        self.simulation.state_machine_manager.terminate_all()
        self.simulation.kill_datetime = None

    def reset(self, state_change):
        """
        Resets the simulation

        :param state_change: The state change that lead to reseting the simulation
        """
        logger.info("State machine outcomes: %s", ", "
                    .join("%s: %s" % (sm.sm_id, str(sm.result))
                          for sm in self.simulation.state_machines))
        self.simulation.state_machine_manager.terminate_all()

        logger.info("simulation reset")

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
    def models_path(self):
        """
        Gets the models_path

        :return: The models_path
        """
        return self.__models_path

    @models_path.setter
    def models_path(self, value):
        """
        Sets the models_path

        """
        self.__models_path = value

    @property
    def simulation_root_folder(self):
        """
        Gets the simulation root folder

        :return: The __simulation_root_folder
        """
        return self.__simulation_root_folder
