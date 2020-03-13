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
Unit tests for the ROSCLESimulationFactoryClient
"""
from mock import patch, Mock
from hbp_nrp_backend.cle_interface.ROSCLESimulationFactoryClient import ROSCLESimulationFactoryClient

__author__ = 'Alessandro Ambrosano, Georg'

import unittest


class TestROSCLESimulationFactoryClient(unittest.TestCase):

    @patch('hbp_nrp_backend.cle_interface.ROSCLESimulationFactoryClient.rospy.ServiceProxy')
    def test_create_new_simulation(self, mock_service_proxy):
        roscle = ROSCLESimulationFactoryClient()
        service_proxy = mock_service_proxy()
        self.assertTrue(service_proxy.wait_for_service.called)
        self.assertIsNone(roscle.create_new_simulation(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12))
        service_proxy.assert_once_called_with(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)
        service_proxy.side_effect = Exception
        self.assertRaises(Exception, roscle.create_new_simulation,
                          1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)


if __name__ == '__main__':
    unittest.main()
