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
Unit tests for the simulation setup
"""

__author__ = 'GeorgHinkel'


import unittest
import json
import datetime
import threading
import time
from hbp_nrp_backend.rest_server.__SimulationService import SimulationService
from mock import patch, MagicMock, PropertyMock
from hbp_nrp_backend.simulation_control import simulations
from hbp_nrp_backend.rest_server.tests import RestTest


class TestSimulationService(RestTest):

    response = []

    def setUp(self):
        self.now = datetime.datetime.now()
        # Ensure that the patcher is cleaned up correctly even in exceptional cases
        del simulations[:]
        self.patch_state = patch('hbp_nrp_backend.simulation_control.__Simulation.Simulation.state',
                                 new_callable=PropertyMock)
        self.mock_state = self.patch_state.start()

    def _postService(self):
        self.response = self.client.post('/simulation',
                                         data=json.dumps({"experimentID": "my_cloned_experiment",
                                                          "gzserverHost": "local",
                                                          "reservation": "user_workshop"}))

    @patch('hbp_nrp_backend.simulation_control.__Simulation.datetime')
    @patch('hbp_nrp_backend.rest_server.__SimulationService.time')
    @patch('hbp_nrp_backend.rest_server.__SimulationService.random')
    def test_simulation_service_post(self, mocked_random, mocked_time, mocked_date_time):
        mocked_date_time.datetime = MagicMock()
        mocked_date_time.datetime.now = MagicMock(return_value=self.now)

        mocked_random.random = MagicMock(return_value=0)
        mocked_time.time = MagicMock(return_value=0)
        self.mock_state.return_value = "initialized"

        # Check that thread_locking is working
        self.response = []
        post_sim_info_thread = threading.Thread(target=TestSimulationService._postService, args=(self,))
        with SimulationService.comm_lock:
            post_sim_info_thread.start()
            time.sleep(2)
            self.assertEqual(self.response, [])     # There should be no response when communication is locked

        post_sim_info_thread.join()

        self.assertEqual(self.response.status_code, 201)
        self.assertEqual(self.response.headers['Location'], 'http://localhost/simulation/0')
        expected_response_data = {
            'owner': "default-owner",
            'state': "initialized",
            'creationDate': self.now.isoformat(),
            'simulationID': 0,
            'environmentConfiguration': None,
            'gzserverHost': 'local',
            "experimentID": "my_cloned_experiment",
            'brainProcesses': 1,
            'creationUniqueID': '0',
            'reservation': 'user_workshop',
            'playbackPath': None
        }
        erd = json.dumps(expected_response_data)
        self.assertEqual(self.response.data.strip(), erd)
        self.assertEqual(len(simulations), 1)
        simulation = simulations[0]
        self.assertEqual(simulation.experiment_id, 'my_cloned_experiment')

    def _getService(self):
        self.response = self.client.get('/simulation')

    def test_simulation_service_get(self):
        exp_id = '0a008f825ed94400110cba4700725e4dff2f55d1'
        param = json.dumps({
            'gzserverHost': 'local',
            'experimentID': exp_id,
            'reservation': 'user_workshop'
        })
        response = self.client.post('/simulation', data=param)
        self.assertEqual(response.status_code, 201)

        # Check that thread_locking is working
        self.response = []
        with SimulationService.comm_lock:
            get_sim_info_thread = threading.Thread(target=TestSimulationService._getService, args=(self,))
            get_sim_info_thread.start()
            time.sleep(2)
            self.assertEqual(self.response, [])     # There should be no response when communication is locked

        # Check response only after get call is complete
        get_sim_info_thread.join()

        self.assertEqual(self.response.status_code, 200)
        self.assertEqual(len(simulations), 1)
        simulation = simulations[0]
        self.assertEqual(simulation.gzserver_host, 'local')
        self.assertEqual(simulation.experiment_id, exp_id)
        self.assertEqual(simulation.reservation, 'user_workshop')

    def test_simulation_service_no_experiment_id(self):
        rqdata = {
            "gzserverHost": "local"
        }
        response = self.client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 400)
        self.assertEqual(len(simulations), 0)

    def test_simulation_service_wrong_gzserver_host(self):
        rqdata = {
            "experimentID": "my_cloned_experiment",
            "gzserverHost": "luxano"   # Wrong server here
        }
        response = self.client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 401)
        self.assertEqual(len(simulations), 0)

    def test_simulation_service_wrong_brain_processes(self):
        rqdata = {
            "experimentID": "my_cloned_experiment",
            "gzserverHost": "local",
            "brainProcesses": -1
        }
        response = self.client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 400)
        self.assertEqual(len(simulations), 0)

    def test_simulation_service_wrong_profile(self):
        rqdata = {
            "experimentID": "my_cloned_experiment",
            "gzserverHost": "local",
            "profiler": "not_a_profile_mode"
        }
        response = self.client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 400)
        self.assertEqual(len(simulations), 0)

    def test_simulation_service_another_sim_running(self):
        rqdata = {
            "experimentID": "my_cloned_experiment",
            "gzserverHost": "lugano"
        }
        s = MagicMock('hbp_nrp_backend.simulation_control.Simulation')()
        s.state = 'started'
        self.client.post('/simulation', data=json.dumps(rqdata))
        response = self.client.post('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 409)

    def test_simulation_service_wrong_method(self):
        rqdata = {
            "experimentID": "my_cloned_experiment",
            "gzserverHost": "local"
        }
        response = self.client.put('/simulation', data=json.dumps(rqdata))

        self.assertEqual(response.status_code, 405)
        self.assertEqual(len(simulations), 0)


if __name__ == '__main__':
    unittest.main()
