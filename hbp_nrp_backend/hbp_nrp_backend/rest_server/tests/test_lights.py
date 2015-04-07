"""
Unit tests for the simulation configuration
"""

__author__ = 'GeorgHinkel'

import unittest
import sys
import mock
import rospy

ros_service_object = mock.Mock()
rospy.wait_for_service = mock.Mock(return_value=ros_service_object)
rospy.ServiceProxy = mock.Mock(return_value=ros_service_object)

import hbp_nrp_backend.simulation_control.tests.unit_tests as utc
from hbp_nrp_backend.rest_server import app
from hbp_nrp_backend.simulation_control import simulations, Simulation
from hbp_nrp_cle.cle.ROSCLEClient import ROSCLEClientException


class TestSimulationService(unittest.TestCase):
    def setUp(self):
        self.client = app.test_client()

        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', 'default-owner', 'created'))


    def test_put_light(self):
        response = self.client.put('/simulation/0/interaction/light', data='{"name":"foo"}')
        self.assertEqual('"Changed light intensity"', response.data)
        self.assertEqual(200, response.status_code)
