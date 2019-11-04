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
This module contains the simulation class
"""

__author__ = 'Georg Hinkel'

from hbp_nrp_backend.simulation_control import timezone
from hbp_nrp_excontrol.StateMachineManager import StateMachineManager
from hbp_nrp_backend.simulation_control.__BackendSimulationLifecycle import \
    BackendSimulationLifecycle
from hbp_nrp_backend.simulation_control.__PlaybackSimulationLifecycle import \
    PlaybackSimulationLifecycle
from tempfile import NamedTemporaryFile
from flask_restful import fields
from flask_restful_swagger import swagger
import datetime
import logging

logger = logging.getLogger(__name__)


@swagger.model
class Simulation(object):
    """
    The data class for simulations
    """
    # pylint: disable=too-many-arguments
    def __init__(self,
                 sim_id,
                 experiment_id,
                 owner,
                 sim_gzserver_host,
                 reservation=None,
                 sim_brain_processes=1,
                 state='created',
                 private=False,
                 playback_path=None,
                 ctx_id=None,
                 token=None):
        """
        Creates a new simulation

        :param sim_id: The simulation id
        :param owner: The name of the user owning the simulation
        :param experiment_id: The experiment ID of the experiment to be run
        :param sim_gzserver_host: Denotes where the simulation will run once started. Set to 'local'
                                  for localhost and 'lugano' for a dedicated machine on the Lugano
                                  viz cluster.
        :param reservation: (optional) the name of the cluster reservation subsequently used to
                            allocate a job
        :param sim_brain_processes: (optional) Number of brain processes to use (overrides ExD
                                    configuration)
        :param state: (optional) The initial state (created by default)
        :param playback_path: (optional) The simulation recording to playback (Path to recording
                              root)
        :param private: (optional) Defines whether the simulation is based on a private experiment
        :param ctx_id: (optional) The context id of the collab if we are running a collab based
                       simulation
        :param token: the request token
        """
        self.__sim_id = sim_id
        self.__owner = owner
        self.__gzserver_host = sim_gzserver_host
        self.__experiment_id = experiment_id
        self.__reservation = reservation
        self.__brain_processes = sim_brain_processes
        self.__creation_datetime = datetime.datetime.now(tz=timezone)
        self.__cle = None
        self.__state_machines_manager = StateMachineManager()
        self.__kill_datetime = self.__creation_datetime + datetime.timedelta(minutes=30)
        self.__creationUniqueID = None
        self.__playback_path = playback_path
        self.__timeout_type = None
        self.__token = token

        # enable the full dynamic backend lifecycle for non-playback launches
        if playback_path is None:
            self.__lifecycle = BackendSimulationLifecycle(self, state)
        else:
            self.__lifecycle = PlaybackSimulationLifecycle(self, state)

        self.__private = private
        self.__ctx_id = ctx_id

    @property
    def ctx_id(self):
        """
        :return: unique creation ID (used by Frontend to identify this simulation)
        """
        return self.__ctx_id

    @property
    def creationUniqueID(self):
        """
        :return: unique creation ID (used by Frontend to identify this simulation)
        """
        return self.__creationUniqueID

    @creationUniqueID.setter
    def creationUniqueID(self, new_ID):
        """
        Sets the simulation unique creation ID (used by Frontend to identify this simulation)

        :param new_ID: The new ID
        """
        self.__creationUniqueID = new_ID

    @property
    def errors(self):
        """
        :return: the number of errors for the current simulation
        """
        is_valid = (self.cle is not None) and self.cle.valid
        is_error = not is_valid or self.state in ['failed', 'halted']

        return int(is_error)

    @property
    def token(self):
        """
        :return: the token
        """

        return self.__token

    @property
    def timeout_type(self):
        """
        Gets the type of timeout
        """
        return self.__timeout_type

    @timeout_type.setter
    def timeout_type(self, new_value):
        """
        Sets the type of timeout

        :param new_value: The new timeout type
        """
        self.__timeout_type = new_value

    @property
    def kill_datetime(self):
        """
        Gets the time when the simulation should be killed
        """
        return self.__kill_datetime

    @kill_datetime.setter
    def kill_datetime(self, new_value):
        """
        Sets the time when the simulation should be killed

        :param new_value: The time when the simulation should be killed
        """
        self.__kill_datetime = new_value

    @property
    def sim_id(self):
        """
        Gets the simulation ID
        """
        return self.__sim_id

    @property
    def owner(self):
        """
        The owner of this simulation

        :return: The owner name
        """
        return self.__owner

    @property
    def creation_datetime(self):
        """
        The creation datetime of this simulation
        :return: The creation datetime
        """
        return self.__creation_datetime

    @property
    def creation_date(self):
        """
        The creation date of this simulation

        :return: The creation date
        """
        return self.__creation_datetime.isoformat()

    @property
    def experiment_id(self):
        """
        The experiment ID of the navigation item of the collab portal used
        to create the experiment

        :return: The associated navigation item experiment_id
        """
        return self.__experiment_id

    @property
    def private(self):
        """
        Defines whether the simulation is based on a private experiment

        :return: A boolean defining whether the sim is private
        """
        return self.__private

    @property
    def lifecycle(self):
        """
        Gets the lifecycle of this simulation

        :return: The lifecycle instance
        """
        return self.__lifecycle

    @property
    def state(self):
        """
        Gets the state of the simulation

        :return: The state of the simulation as a string
        """
        return self.__lifecycle.state

    @state.setter
    def state(self, new_state):
        """
        Sets the simulation in a new state

        :param new_state: The new state
        """
        self.__lifecycle.accept_command(new_state)

    @property
    def cle(self):
        """
        The CLE for this simulation
        :return: The CLE instance
        """
        return self.__cle

    @cle.setter
    def cle(self, cle):
        """
        Sets the CLE for this simulation
        :param cle: The new CLE
        """
        self.__cle = cle

    @property
    def state_machine_manager(self):
        """
        Gets the associated component managing the state machines of the simulation
        """
        return self.__state_machines_manager

    @property
    def state_machines(self):
        """
        Get state_machines contained in this simulation

        :return: The list of state machine instances
        """
        return self.__state_machines_manager.state_machines

    def get_state_machine(self, name):
        """
        Gets the state machine with the given name or none

        :param name: The state machine name
        :return: The state machine with the given name
        """
        return self.__state_machines_manager.get_state_machine(name)

    def get_state_machine_code(self, name):
        """
        Get state machine code

        :param name: name of the state machine
        :return: source code of the state machine (string) or None when \
        state machine is not found
        """
        sm = self.get_state_machine(name)
        if sm is not None and sm.sm_path is not None:
            with open(sm.sm_path, "r") as sm_file:
                return sm_file.read()

        return None

    def set_state_machine_code(self, name, python_code):
        """
        Set state machine code.
        Only works when simulation is in state "paused"

        :param   name:        name of the state machine
        :type    name:        string
        :param   python_code: source code of the state machine
        :type    python_code: string
        :raise:  NameError, SyntaxError, AttributeError, ...
        """
        with NamedTemporaryFile(prefix='sm_', suffix='.py', delete=False) as tmp:
            with tmp.file as sm_file:
                file_path = tmp.name
                sm_file.write(python_code)
        logger.info(
            "Creating new file for state machine {0}: {1}".format(name, file_path))

        sm = self.get_state_machine(name)
        if sm is None:
            sm = self.state_machine_manager.create_state_machine(
                name, self.sim_id, self.lifecycle.sim_dir)
        else:
            sm.request_termination()
            sm.wait_termination()

        sm.sm_path = file_path

        if not sm.is_running:
            sm.initialize_sm()

    def delete_state_machine(self, name):
        """
        Delete state machine.

        :param   name:        name of the state machine
        :type    name:        string
        :return: (pair) --    A pair made of a
                              (bool) boolean, which is True on success, False otherwise, and a
                              (string) string indicating either a successful delete operation
                              or that the state machine was not found
        :rtype:  bool, string
        :raise:  NameError
        """

        sm = self.get_state_machine(name)
        if sm is not None:
            self.state_machines.remove(sm)
            if sm.is_running:
                sm.request_termination()
                sm.wait_termination()
            return True, "Success"

        return False, "State machine '{0}' not found.".format(name)

    def delete_all_state_machines(self):
        """
        Delete all state machines.
        """

        allsm = list(self.state_machines)
        for sm in allsm:
            self.delete_state_machine(sm.sm_id)

    @property
    def gzserver_host(self):
        """
        Gets the host/cluster where the gzserver is running or will run once the simulation is
        started.
        """
        return self.__gzserver_host

    @property
    def reservation(self):
        """
        Gets the cluster reservation that will be used to allocate a job running Gazebo.
        If the self.__reservation is None, the allocation request will not refer to any reservation.
        """
        return self.__reservation

    @property
    def brain_processes(self):
        """
        Gets the number of brain processes used for this simulation, overrides value in experiment
        configuration file
        """
        return self.__brain_processes

    @property
    def playback_path(self):
        """
        Gets the simulation recording path being used to playback this simulation, None if this is
        a live simulation
        """
        return self.__playback_path

    @playback_path.setter
    def playback_path(self, playback_path):
        """
        Sets the playback path
        :param playback_path: new playback path
        """
        self.__playback_path = playback_path

    resource_fields = {
        'state': fields.String,
        'simulationID': fields.Integer(attribute='sim_id'),
        'environmentConfiguration': fields.String(attribute='environment_conf'),
        'owner': fields.String(attribute='owner'),
        'creationDate': fields.String(attribute=lambda x: x.creation_date),
        'gzserverHost': fields.String(attribute='gzserver_host'),
        'reservation': fields.String(attribute='reservation'),
        'experimentID': fields.String(attribute='experiment_id'),
        'brainProcesses': fields.Integer(attribute='brain_processes'),
        'creationUniqueID': fields.String(attribute='creationUniqueID'),
        'playbackPath': fields.String(attribute='playback_path')
    }

    required = ['state', 'simulationID',
                'environmentConfiguration', 'owner',
                'creationDate', 'gzserverHost',
                'reservation', 'experimentID',
                'brainProcesses', 'creationUniqueID']


class InvalidStateTransitionException(Exception):
    """
    Represents that an invalid state transition was attempted
    """
    pass


class InvalidTransferFunctionCodeException(Exception):
    """
    Represents that invalid source code for transfer function was submitted
    """
    pass
