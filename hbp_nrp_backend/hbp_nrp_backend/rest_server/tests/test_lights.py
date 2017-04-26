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
Unit tests for the simulation configuration
"""

__author__ = 'Georg Hinkel'

import unittest
import mock
import rospy
from hbp_nrp_backend.rest_server.tests import RestTest

ros_service_object = mock.Mock()
rospy.wait_for_service = mock.Mock(return_value=ros_service_object)
rospy.ServiceProxy = mock.Mock(return_value=ros_service_object)

from hbp_nrp_backend.rest_server import init
from hbp_nrp_backend.simulation_control import simulations, Simulation


class TestSimulationService(RestTest):
    def setUp(self):
        del simulations[:]
        simulations.append(Simulation(0, 'experiment1', None, 'default-owner', 'created'))

    def test_put_light(self):
        response = self.client.put('/simulation/0/interaction/light', data='{"name":"foo"}')
        self.assertEqual('"Changed light intensity"', response.data.strip())
        self.assertEqual(200, response.status_code)

if __name__ == '__main__':
    unittest.main()
