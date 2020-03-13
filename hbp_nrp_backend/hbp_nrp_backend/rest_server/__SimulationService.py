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
This module contains the REST services to setup the simulation
"""

__author__ = 'GeorgHinkel'

from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_backend import NRPServicesClientErrorException
from hbp_nrp_backend.rest_server import api
from hbp_nrp_backend.rest_server.RestSyncMiddleware import RestSyncMiddleware
from hbp_nrp_backend.rest_server.__SimulationControl import SimulationControl
from hbp_nrp_backend.__UserAuthentication import UserAuthentication
from flask import request
from flask_restful import Resource, fields, marshal, marshal_with
from flask_restful_swagger import swagger
import logging
import time
import random
import threading

# pylint: disable=R0201

logger = logging.getLogger(__name__)


class SimulationService(Resource):
    """
    The module to setup simulations
    """

    comm_lock = threading.Lock()

    @swagger.model
    class _Experiment(object):
        """
        Represents a simulation ID object. Only used for swagger documentation
        """

        resource_fields = {
            'brainProcesses': fields.Integer,
            'experimentID': fields.String(),
            'environmentConfiguration': fields.String(),
            'gzserverHost': fields.String(),
            'reservation': fields.String(),
            'state': fields.String(),
            'private': fields.Boolean,
            'playbackPath': fields.String(),
            'ctx-id': fields.String()
        }
        required = ['experimentID']

    @swagger.operation(
        notes='This is the entry point for the NRP REST server since'
              ' this is where you actually create simulations',
        responseClass=Simulation.__name__,
        parameters=[
            {
                "name": "body",
                "paramType": "body",
                "dataType": _Experiment.__name__,
                "required": True
            }
        ],
        responseMessages=[
            {
                "code": 402,
                "message": "Another simulation is already running on the server"
            },
            {
                "code": 401,
                "message": "gzserverHost is not valid"
            },
            {
                "code": 400,
                "message": "Experiment configuration is not valid"
            },
            {
                "code": 201,
                "message": "Simulation created successfully"
            }
        ],
    )
    def post(self):
        # pylint: disable=R0914
        """
        Creates a new simulation which is neither 'initialized' nor 'started'.
        :< json int brainProcesses: Number of brain processes to use (overrides ExD Configuration)
        :< json string experimentID: The experiment ID of the experiment
        :> json string gzserverHost: The host where gzserver will be run: local for using the same
                                     machine of the backend, lugano to use a dedicated instance on
                                     the Lugano viz cluster
        :< json string reservation: the name of the cluster reservation subsequently used to
                                    allocate a job
        :< json string state: The initial state of the simulation
        :< json boolean private: Defines whether the simulation is based on a private experiment
        :< json string playbackPath: Path to simulation recording to play (optional)
        :< json string ctx-id: The context id of the collab if we are running a collab based
                               simulation

        :> json string owner: The simulation owner (Unified Portal user name or 'hbp-default')
        :> json integer simulationID: The id of the simulation (needed for further REST calls)
        :> json string creationDate: Date of creation of this simulation
        :> json string creationUniqueID: The simulation unique creation ID that is used by the
                                         Frontend to identify this simulation
        :> json string profiler: indicates the profiler mode for the simulation: disabled,
                                 cle_step, cprofile

        :status 400: Experiment configuration is not valid
        :status 401: gzserverHost is not valid
        :status 402: Another simulation is already running on the server
        :status 201: Simulation created successfully
        """
        # Use context manager to lock access to simulations while a new simulation is created
        with SimulationService.comm_lock:
            body = request.get_json(force=True)
            sim_id = len(simulations)
            if 'experimentID' not in body:
                raise NRPServicesClientErrorException('Experiment ID not given.')

            if ('gzserverHost' in body) and (body.get('gzserverHost') not in ['local', 'lugano']):
                raise NRPServicesClientErrorException('Invalid gazebo server host.', error_code=401)

            if True in [s.state not in ['stopped', 'failed'] for s in simulations]:
                raise NRPServicesClientErrorException(
                    'Another simulation is already running on the server.', error_code=409)

            if 'brainProcesses' in body and \
                    (not isinstance(body.get('brainProcesses'), int)
                     or body.get('brainProcesses') < 1):
                raise NRPServicesClientErrorException('Invalid number of brain processes.')

            if ('profiler' in body) and (body.get('profiler') not in ['disabled', 'cle_step',
                                                                      'cprofile']):
                raise NRPServicesClientErrorException('Invalid profiler mode.', error_code=400)

        sim_gzserver_host = body.get('gzserverHost', 'local')
        sim_reservation = body.get('reservation', None)
        sim_experiment_id = body.get('experimentID', None)
        sim_state = body.get('state', 'created')
        playback_path = body.get('playbackPath', None)
        sim_owner = UserAuthentication.get_user()
        sim_brain_processes = body.get('brainProcesses', 1)
        private = body.get('private', False)
        ctx_id = body.get('ctxId', None)
        token = UserAuthentication.get_header_token()
        profiler = body.get('profiler', 'disabled')

        sim = Simulation(sim_id,
                         sim_experiment_id,
                         sim_owner,
                         sim_gzserver_host,
                         sim_reservation,
                         sim_brain_processes,
                         sim_state,
                         playback_path=playback_path,
                         private=private,
                         ctx_id=ctx_id,
                         token=token,
                         profiler=profiler)

        # TODO: remove me. I probably am not used anywhere
        sim.creationUniqueID = body.get('creationUniqueID', str(time.time() + random.random()))

        simulations.append(sim)

        sim.state = "initialized"

        return marshal(simulations[sim_id], Simulation.resource_fields), 201, {
            'location': api.url_for(SimulationControl, sim_id=sim_id),
            'gzserverHost': sim_gzserver_host
        }

    @swagger.operation(
        notes='Gets the list of all simulations on the server,'
              ' no matter what state',
        responseClass=list.__name__,
        responseMessages=[
            {
                "code": 200,
                "message": "Simulations retrieved successfully"
            }
        ]
    )
    @RestSyncMiddleware.threadsafe
    @marshal_with(Simulation.resource_fields)
    def get(self):
        """
        Gets the list of simulations on this server.

        :> json int experimentID:
        :> json int brainProcesses:
        :> json int environmentConfiguration:
        :> json string owner: The simulation owner (Unified Portal user name or 'hbp-default')
        :> json int reservation:
        :> json string creationDate: Date of creation of this simulation
        :> json string creationUniqueID:
        :> json string gzserverHost: Denotes where the gzserver will run once the simulation is
                                     started, local for localhost, lugano for a remote execution on
                                     the Lugano viz cluster.
        :> json string state: The current state of the simulation
        :> json int simulationID: The id of the simulation (needed for further REST calls)

        :status 200: Simulations retrieved successfully
        """
        # Acquire lock before getting simulation
        with SimulationService.comm_lock:
            return simulations, 200
