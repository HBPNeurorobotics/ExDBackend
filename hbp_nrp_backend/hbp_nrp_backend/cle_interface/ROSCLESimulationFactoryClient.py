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
Helper class that takes care of making the appropriate ROS call(s) to
start a simulation.
"""

import rospy
# This package comes from the catkin package ROSCLEServicesDefinitions
# in the root folder of the GazeboRosPackage repository.
from cle_ros_msgs import srv
from hbp_nrp_backend.cle_interface import SERVICE_CREATE_NEW_SIMULATION

__author__ = "Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli"


class ROSCLESimulationFactoryClient(object):

    """
    Helper class for the simulation factory ROS service started with class
    ROSCLESimulationFactory.
    """

    def __init__(self):
        """
        Create the client. If the service is not available after 10 seconds, a ROSException
        will be raised.
        """

        self.__create_new_simulation_service = rospy.ServiceProxy(
            SERVICE_CREATE_NEW_SIMULATION, srv.CreateNewSimulation
        )
        self.__create_new_simulation_service.wait_for_service(timeout=10)

    def create_new_simulation(self, environment_file, experiment_conf, gzserver_host,
                              reservation, brain_processes, sim_id, timeout):
        """
        Start the simulation.
        """
        self.__create_new_simulation_service(environment_file, experiment_conf,
                                             gzserver_host, reservation,
                                             brain_processes, sim_id, timeout)
