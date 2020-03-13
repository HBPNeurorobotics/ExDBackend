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
ROSCLESimulationFactory unit test
"""

import hbp_nrp_cle
from hbp_nrp_cleserver.server import ROSCLESimulationFactory, ROS_CLE_NODE_NAME, SERVICE_VERSION,\
    SERVICE_CREATE_NEW_SIMULATION
import logging
from mock import patch, MagicMock, Mock
from testfixtures import log_capture, LogCapture
import unittest
from datetime import datetime, timedelta
import pytz
import threading
from cle_ros_msgs import srv
import os
import time
import sys
PATH = os.path.dirname(__file__)
tz = pytz.timezone("Europe/Zurich")


__author__ = 'Georg Hinkel, Bernd Eckstein'


class TestROSCLESimulationFactory(unittest.TestCase):

    LOGGER_NAME = 'hbp_nrp_cleserver'

    class MockedServiceRequest(object):
        environment_file = "environment_file.sdf"
        exd_config_file = os.path.join(PATH, "experiment_data/ExDXMLExample.exc")
        gzserver_host = "local"
        brain_processes = 1
        sim_id = 0
        timeout = str(datetime.now(tz) + timedelta(minutes=5))
        timeout_type = 'real'
        reservation = 'user_workshop'
        playback_path = None
        token = None
        ctx_id = None
        experiment_id = None
        profiler = 0

    @patch('hbp_nrp_cleserver.server.ROSCLESimulationFactory.logger')
    def setUp(self, mocked_logger):
        unittest.TestCase.setUp(self)

        # Mock the respective objects and make them available for all tests.
        # Also have a look at the following link:
        # https://docs.python.org/3.5/library/unittest.mock-examples.html
        # #applying-the-same-patch-to-every-test-method
        rospy_patcher = patch('hbp_nrp_cleserver.server.ROSCLESimulationFactory.rospy')

        # Ensure that the patchers are cleaned up correctly even in exceptional cases
        # e.g. when an exception was thrown.
        self.addCleanup(rospy_patcher.stop)

        # Start the patch for all tests
        self.__mocked_rospy = rospy_patcher.start()

        self.__ros_cle_simulation_factory = ROSCLESimulationFactory.ROSCLESimulationFactory()
        self.assertEqual(mocked_logger.debug.call_count, 1)
        self.assertEqual(self.__ros_cle_simulation_factory.running_simulation_thread, None)

        self.mocked_service_request = self.MockedServiceRequest()

        self.simulation_event = threading.Event()

        def mocked_simulation(mocked_self, cle_launcher):
            self.simulation_event.wait()

        # Make sure we do not really start a simulation
        self.original_simulation = ROSCLESimulationFactory.ROSCLESimulationFactory._ROSCLESimulationFactory__simulation
        ROSCLESimulationFactory.ROSCLESimulationFactory._ROSCLESimulationFactory__simulation = mocked_simulation
        self.simulation_event.clear()

    def mockThreading(self):
        threading_patcher = patch('hbp_nrp_cleserver.server.ROSCLESimulationFactory.threading')
        self.addCleanup(threading_patcher.stop)
        self.__mocked_threading = threading_patcher.start()
        self.__mocked_thread = ROSCLESimulationFactory.threading.Thread()
        self.__mocked_threading.Thread = MagicMock(return_value=self.__mocked_thread)
        self.__ros_cle_simulation_factory.\
            running_simulation_thread = MagicMock(spec=threading.Thread)

    def tearDown(self):
        # remove all handlers after each test!
        logging.getLogger(self.LOGGER_NAME).handlers = []
        ROSCLESimulationFactory.ROSCLESimulationFactory._ROSCLESimulationFactory__simulation = self.original_simulation

    def test_run(self):
        self.__ros_cle_simulation_factory.initialize()
        self.__ros_cle_simulation_factory.run()
        self.__mocked_rospy.init_node.assert_called_once_with(ROS_CLE_NODE_NAME, anonymous=True)
        self.__mocked_rospy.Service.assert_any_call(
            SERVICE_CREATE_NEW_SIMULATION,
            srv.CreateNewSimulation,
            self.__ros_cle_simulation_factory.create_new_simulation
        )
        self.__mocked_rospy.Service.assert_any_call(
            SERVICE_VERSION,
            srv.GetVersion,
            self.__ros_cle_simulation_factory.get_version
        )
        self.assertEqual(self.__mocked_rospy.Service.call_count, 3)
        self.__mocked_rospy.spin.assert_called_once_with()

    @patch('hbp_nrp_cleserver.server.ROSCLESimulationFactory.ServerConfigurations', MagicMock())
    def test_is_simulation_running(self):
        self.assertFalse(
            self.__ros_cle_simulation_factory.is_simulation_running(self.mocked_service_request)
        )

        self.simulation_event.clear()

        self.__ros_cle_simulation_factory.create_new_simulation(self.mocked_service_request)
        self.assertTrue(
            self.__ros_cle_simulation_factory.is_simulation_running(self.mocked_service_request)
        )

        self.simulation_event.set()

        time.sleep(1)
        self.assertFalse(
            self.__ros_cle_simulation_factory.is_simulation_running(self.mocked_service_request)
        )

    @patch('hbp_nrp_cleserver.server.ROSCLESimulationFactory.logger')
    @patch('hbp_nrp_cleserver.server.LocalGazebo.os')
    @patch('hbp_nrp_cleserver.server.LocalGazebo.Watchdog', new=Mock())
    @patch('hbp_nrp_cleserver.server.ROSCLESimulationFactory.ServerConfigurations', MagicMock())
    def test_create_new_simulation_dead_thread(self, mocked_os, mocked_logger):
        self.mockThreading()
        self.__ros_cle_simulation_factory.\
            running_simulation_thread.is_alive = MagicMock(return_value=False)

        with patch('hbp_nrp_cleserver.server.LocalGazebo.LocalGazeboServerInstance.gazebo_master_uri')\
            as mock_gazebo_master_uri:
            mock_gazebo_master_uri.__get__ = Mock(return_value=None)
            self.__ros_cle_simulation_factory.create_new_simulation(
                self.mocked_service_request
            )

        self.assertEqual(mocked_logger.info.call_count, 5)
        self.assertEqual(mocked_logger.error.call_count, 0)

        self.assertEqual(self.__mocked_threading.Thread.call_count, 1)
        self.assertEqual(
            self.__ros_cle_simulation_factory.running_simulation_thread,
            self.__mocked_thread
        )
        self.assertTrue(self.__mocked_thread.daemon)
        self.assertTrue(self.__mocked_thread.start.called)

    @patch('hbp_nrp_cleserver.server.ROSCLESimulationFactory.logger')
    def test_create_new_simulation_living_thread(self, mocked_logger):
        self.mockThreading()
        self.__ros_cle_simulation_factory.running_simulation_thread.is_alive = MagicMock(
            return_value=True
        )

        self.assertRaises(Exception, self.__ros_cle_simulation_factory.create_new_simulation,
            self.mocked_service_request
        )

        self.assertEqual(mocked_logger.info.call_count, 1)
        self.assertEqual(mocked_logger.error.call_count, 1)

        self.assertNotEqual(self.__ros_cle_simulation_factory.running_simulation_thread,
                            self.__mocked_thread)
        self.__ros_cle_simulation_factory.running_simulation_thread.is_alive.\
            assert_called_once_with()

    @log_capture(level=logging.WARNING)
    def test_set_up_logger(self, logcapture):
        ROSCLESimulationFactory.set_up_logger('cle_logfile.txt')
        self.assertTrue(os.path.isfile('cle_logfile.txt'))
        os.remove('cle_logfile.txt')

        ROSCLESimulationFactory.set_up_logger(None)
        logcapture.check((
            self.LOGGER_NAME,
            'WARNING',
            'Could not write to specified logfile or no logfile specified, logging to stdout now!'
        ))
        self.assertEqual(logging.getLogger().getEffectiveLevel(), logging.INFO)

        ROSCLESimulationFactory.set_up_logger(None, True)
        self.assertEqual(logging.getLogger().getEffectiveLevel(), logging.DEBUG)

    @log_capture(level=logging.CRITICAL)
    def test_except_hook(self, logcapture):
        ROSCLESimulationFactory.set_up_logger(None)
        sys.excepthook(Exception, Exception("Something really bad happened"), "This is a stacktrace")
        logcapture.check((
            self.LOGGER_NAME,
            'CRITICAL',
            "Unhandled exception of type <type 'exceptions.Exception'>: Something really bad happened"
        ))

    def test_get_version(self):
        cle_version = str(self.__ros_cle_simulation_factory.get_version(None))
        self.assertEqual(cle_version, hbp_nrp_cle.__version__)

    @patch('traceback.extract_stack')
    @patch('sys._current_frames')
    @patch('hbp_nrp_cleserver.server.ROSCLESimulationFactory.logger.info')
    @patch('hbp_nrp_cleserver.server.ROSCLEServer.logger.info')
    def test_print_full_stack_trace(self, mocked_info_2, mocked_info_1, current_frames,
                                    extract_stack):
        with LogCapture() as logcapture:
            current_frames.return_value = {'1234': 'dummystack'}
            extract_stack.return_value = [['dummy_file', 42, 'dummy_name', 'dummy_line']]
            ROSCLESimulationFactory.print_full_stack_trace(None, None)
            logcapture.check(
                            (self.LOGGER_NAME, 'WARNING', '*** STACKTRACE - START ***'),
                            (self.LOGGER_NAME, 'WARNING', '# ThreadID: 1234'),
                            (self.LOGGER_NAME,
                             'WARNING', 'File: "dummy_file", line 42, in dummy_name'),
                            (self.LOGGER_NAME, 'WARNING', '  dummy_line'),
                            (self.LOGGER_NAME, 'WARNING', '*** STACKTRACE - END ***'))

class Args(object):
    logfile = None
    pycharm = False
    verbose = False
    vsdebug = False

class TestSimulationFactoryMain(unittest.TestCase):
    @patch("hbp_nrp_cleserver.server.ROSCLESimulationFactory.argparse.ArgumentParser.parse_args")
    @patch("hbp_nrp_cleserver.server.ROSCLESimulationFactory.signal")
    @patch("hbp_nrp_cleserver.server.ROSCLESimulationFactory.ROSCLESimulationFactory")
    def test_main(self, factory, signal, argparse):
        argparse.return_value = Args
        ROSCLESimulationFactory.main()
        self.assertTrue(signal.signal.called)
        factory.assert_called_once_with()
        factory().initialize.assert_called_once_with()
        factory().run.assert_called_once_with()
        argparse.assert_called_with()

if __name__ == '__main__':
    unittest.main()
