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
This module contains the unit tests for the cle launcher initialization
"""

import unittest
from mock import patch, MagicMock, Mock
from hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly import CLEGazeboSimulationAssembly
from hbp_nrp_cleserver.server.LuganoVizClusterGazebo import XvfbXvnError
from hbp_nrp_cle.mocks.robotsim import MockRobotControlAdapter, MockRobotCommunicationAdapter
from hbp_nrp_cle.mocks.externalsim.MockExternalModuleManager import MockExternalModuleManager
import hbp_nrp_cle.tf_framework as tfm
from hbp_nrp_cle.robotsim.RobotManager import Robot
from hbp_nrp_commons.MockUtil import MockUtil

class MockedGazeboHelper(object):

    def load_gazebo_world_file(self, world):
        return {}, {}

    @staticmethod
    def parse_gazebo_world_file(world):
        return {}, {}

    def __getattr__(self, x):
        return Mock()

    def wait_for_backend_rendering(self):
        pass


class MockedClosedLoopEngine(Mock):

    DEFAULT_TIMESTEP = 0.02


mockedOS = Mock()
mockedOS.environ = {'ROS_MASTER_URI': "localhost:0815"}
mockedOS.path.join.return_value = ''
mockedOS.listdir.return_value = []


class SomeWeirdTFException(Exception):
    pass


@patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.ZipUtil", new=Mock())
@patch("hbp_nrp_cleserver.server.SimulationAssembly.os", new=Mock())
@patch("hbp_nrp_cleserver.server.SimulationAssembly.ROSNotificator", new=Mock())
@patch("hbp_nrp_cle.robotsim.RosControlAdapter.RosControlAdapter", new=MockRobotControlAdapter)
@patch("hbp_nrp_cle.robotsim.RosCommunicationAdapter.RosCommunicationAdapter", new=MockRobotCommunicationAdapter)
@patch("hbp_nrp_cleserver.server.GazeboSimulationAssembly.LocalGazeboBridgeInstance", new=Mock())
@patch("hbp_nrp_cleserver.server.GazeboSimulationAssembly.ROSLaunch", new=Mock())
@patch("hbp_nrp_cle.cle.DeterministicClosedLoopEngine.GazeboHelper", new=MockedGazeboHelper)
@patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.ClosedLoopEngine", new=MockedClosedLoopEngine())
@patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.CLEGazeboSimulationAssembly._create_brain_adapters", new=Mock(return_value=(Mock(), Mock())))
@patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.CLEGazeboSimulationAssembly._create_robot_adapters", new=Mock(return_value=(Mock(), Mock())))
@patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.os.environ", new=mockedOS.environ)
@patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.StorageClient", new=Mock())
@patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.os.listdir", new=Mock(return_value=[]))
@patch("hbp_nrp_cleserver.server.GazeboSimulationAssembly.rospy", new=MagicMock())
@patch("hbp_nrp_cleserver.server.GazeboSimulationAssembly.rosnode", new=MagicMock())
@patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.ExternalModuleManager", new=MockExternalModuleManager)

class TestCLELauncherInit(unittest.TestCase):

    @patch("hbp_nrp_cleserver.server.GazeboSimulationAssembly.LocalGazeboServerInstance")
    def setUp(self, mock_gazebo):
        self.m_ziputil = MockUtil.fakeit(self, 'hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.ZipUtil')
        self.mock_gazebo = mock_gazebo
        self.m_sim_util = MockUtil.fakeit(self, 'hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.SimUtil')
        self.m_sim_util.find_file_in_paths.return_value = '/a/robot/under/the/rainbow/model.sdf'

        self.m_simconf = MagicMock()
        self.m_simconf.sim_id = 123
        self.m_simconf.token = 123
        self.m_simconf.experiment_id = 'experiment'
        self.m_simconf.timeout = 'YYYY-MM-DD-HH:MM:SS'
        self.m_simconf.timeout_type = 'real'
        self.m_simconf.sim_dir = '/my/experiment'
        self.m_simconf.gzserver_host = 'local'
        self.m_simconf.ros_notificator = 'real'
        self.m_simconf.world_model.resource_path.abs_path = '/my/experiment/world.sdf'
        self.m_simconf.brain_model.is_custom = False
        self.m_simconf.brain_model.resource_path.rel_path = 'brain.sdf'
        self.m_simconf.brain_model.resource_path.abs_path = '/my/experiment/brain.sdf'
        # alternate set is_custom in individual test
        self.m_simconf.brain_model.zip_path.rel_path = 'brains/brain.zip'
        self.m_simconf.brain_model.zip_path.abs_path = '/my/experiment/brains/brain.sdf'

        self.m_simconf.robot_models = {
            'bb8': Robot('bb8', '/my/experiment/bb8/model.sdf', 'my bb8', Mock(), False, 'my/roslauch/path',
                         'modelname'),
            'r2d2': Robot('r2d2', '/my/experiment/r2d2/model.zip', 'my r2d2', Mock(), True,
                          '/my/experiment/ros.launch', 'modelname')
        }
        self.m_simconf.retina_config = 'retina_configuration'
        self.m_simconf.ext_robot_controller = 'xyz.model'
        self.m_simconf.ros_launch_abs_path = '/my/experiment/ros.launch'
        self.patch_cleserver = patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.ROSCLEServer")
        self.mocked_cleserver = self.patch_cleserver.start()
        #self.mocked_cleserver._robotHandler.download_custom_robot.return_value = "somewhere/over/the/rainbow"
        self.mocked_cleserver.return_value._robotHandler.prepare_custom_robot.return_value = (True, '/my/experiment/r2d2/model.sdf')

        self.patch_robotManager = patch("hbp_nrp_cleserver.server.GazeboSimulationAssembly.RobotManager")
        self.mocked_robotManager = self.patch_robotManager.start()
        self.mocked_robotManager.return_value.scene_handler.return_value.parse_gazebo_world_file.return_value = {}, {}

        with patch("hbp_nrp_cleserver.server.GazeboSimulationAssembly.rospy"), patch("hbp_nrp_cleserver.server.GazeboSimulationAssembly.rosnode"):
            self.launcher = CLEGazeboSimulationAssembly(self.m_simconf)

    def tearDown(self):
        with patch("hbp_nrp_cleserver.server.GazeboSimulationAssembly.rospy") as patched_rospy, patch("hbp_nrp_cleserver.server.GazeboSimulationAssembly.rosnode") as patched_rosnode:
            patched_rospy.get_param_names.return_value = []
            patched_rosnode.get_node_names.return_value = []
            self.launcher.shutdown()
        self.patch_robotManager.stop()
        self.patch_cleserver.stop()

    def test_gazebo_start_exception_catches_xvfbxvn_error(self):
        self.mock_gazebo().start.side_effect = XvfbXvnError
        exception_caught = False
        try:
            self.launcher.initialize(None)
        except Exception, e:
            self.assertTrue(str(e).startswith("Recoverable error occurred"), msg=e)
            exception_caught = True
        self.assertTrue(exception_caught)

    @patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.tfm")
    @patch("hbp_nrp_backend.storage_client_api.StorageClient.os")
    def test_wrong_transfer_function_doesnt_abort_initialization(self,mock_os, mock_tfm):
        mock_tfm.set_transfer_function.side_effect = SomeWeirdTFException
        mock_os.environ.get.return_value = ""
        try:
            self.launcher.initialize(None)
        except SomeWeirdTFException:
            raise Exception("Faulty TF should not abort initialization")

        # tf should be set as faulty
        mock_tfm.set_faulty_transfer_function.assert_called_once()

    @patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.tfm")
    @patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.compile_restricted")
    @patch("hbp_nrp_backend.storage_client_api.StorageClient.os")
    def test_faulty_tf_does_not_aborts_initialization_syntax_error(self, mock_os, comp_restricted_mock, mock_tfm):
        mock_tfm.set_faulty_transfer_function = MagicMock()
        mock_os.environ.get.return_value = ""
        comp_restricted_mock.side_effect = SyntaxError()

        try:
            self.launcher.initialize(None)
        except SyntaxError:
            self.fail('CLEGazeboSimulationAssembly.initialize should NOT propagate a SyntaxError.')

        mock_tfm.set_faulty_transfer_function.assert_called_once()

    @patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.tfm")
    @patch("hbp_nrp_backend.storage_client_api.StorageClient.os")
    def test_faulty_tf_does_not_aborts_initialization_TFLoading_exception(self, mock_os, mock_tfm):
        mock_tfm.set_faulty_transfer_function = MagicMock()
        mock_os.environ.get.return_value = ""
        mock_tfm.TFLoadingException = tfm.TFLoadingException  # restore TFLoadingException class in the mock
        mock_tfm.set_transfer_function = MagicMock(side_effect=tfm.TFLoadingException('tf_foo', 'foo error'))

        try:
            self.launcher.initialize(None)
        except tfm.TFLoadingException:
            self.fail('CLEGazeboSimulationAssembly.initialize should NOT propagate a TFLoadingException.')

        mock_tfm.set_faulty_transfer_function.assert_called_once()

    @patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.tfm")
    @patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.compile_restricted")
    def test_faulty_tf_does_not_aborts_initialization_Syntax_exception(self, comp_restricted_mock, mock_tfm):

        mock_tfm.set_faulty_transfer_function = MagicMock()
        comp_restricted_mock.side_effect = SyntaxError()

        try:
            self.launcher.initialize(None)
        except SyntaxError:
            self.fail(
                'CLEGazeboSimulationAssembly.initialize should NOT propagate a SyntaxError.')

        mock_tfm.set_faulty_transfer_function.assert_called_once()

    def __launch_callback(self, *_):
        self.launcher.gzserver.gazebo_died_callback()

    def test_gazebo_crash_aborts_initialization(self):
        self.mock_gazebo().start.side_effect = self.__launch_callback
        exception_caught = False
        try:
            self.launcher.initialize(None)
        except Exception, e:
            self.assertTrue(str(e).startswith("The simulation must abort"), msg=e)
            exception_caught = True

        self.assertTrue(exception_caught)
        self.mocked_cleserver.return_value.lifecycle.failed.assert_called_once_with()
        self.mocked_cleserver.return_value.lifecycle.stopped.assert_called_once_with()
