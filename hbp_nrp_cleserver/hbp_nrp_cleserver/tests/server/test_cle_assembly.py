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
This module contains the unit tests for the cle launcher
"""

import unittest
import os
from mock import patch, Mock
from hbp_nrp_cleserver.server.ServerConfigurations import SynchronousNestSimulation, SynchronousRobotRosNest
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen
from hbp_nrp_cle.mocks.robotsim import MockRobotControlAdapter, MockRobotCommunicationAdapter
from hbp_nrp_backend import NRPServicesGeneralException

PATH = os.path.split(__file__)[0]


def robot_value():
    return "robots/this_is_a_robot.sdf"


class CustomModel(object):
    def __init__(self):
        self.customModelPath = None
        self.value = robot_value
        self.path = 'test'
        self.robotId = 'robot'


@patch("hbp_nrp_backend.storage_client_api.StorageClient.StorageClient")
class TestCLEGazeboSimulationAssembly(unittest.TestCase):
    def setUp(self):
        dir = os.path.split(__file__)[0]
        with open(os.path.join(dir, "experiment_data/milestone2.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        with open(os.path.join(dir, "experiment_data/ExDXMLExample.exc")) as exd_file:
            exd = exp_conf_api_gen.CreateFromDocument(exd_file.read())
        exd.path = "/somewhere/over/the/rainbow/exc"
        exd.dir = "/somewhere/over/the/rainbow"
        bibi.path = "/somewhere/over/the/rainbow/bibi"
        self.models_path_patch = patch(
            "hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.models_path", new="/somewhere/near/the/rainbow")
        self.models_path_patch.start()
        self.launcher = SynchronousNestSimulation(
            42, exd, bibi, gzserver_host="local")

    def tearDown(self):
        self.models_path_patch.stop()

    @patch("hbp_nrp_backend.storage_client_api.StorageClient.find_file_in_paths")
    @patch("hbp_nrp_backend.storage_client_api.StorageClient.get_model_basepath")
    def test_robot_path_sdf(self, mocked_get_model_basepath, mock_find_file_path, mocked_storage):
        mocked_get_model_basepath.return_value = PATH
        mock_find_file_path.return_value = os.path.join(
            PATH, 'this_is_a_robot.sdf')
        robot = CustomModel()
        robot_file, _ = self.launcher._get_robot_abs_path(robot)
        self.assertEqual(
            robot_file, os.path.join(
                PATH, 'this_is_a_robot.sdf'))
        self.assertEqual("", self.launcher._CLEGazeboSimulationAssembly__tmp_robot_dir)

    @patch("tempfile.mkdtemp")
    @patch("zipfile.ZipFile")
    @patch("hbp_nrp_backend.storage_client_api.StorageClient.find_file_in_paths")
    @patch("hbp_nrp_backend.storage_client_api.StorageClient.get_model_basepath")
    def test_robot_path_zip(self, mocked_get_model_basepath, mock_find_file_path, mocked_zip,
                            mocked_temp, mocked_storage):
        mocked_get_model_basepath.return_value = PATH
        mock_find_file_path.return_value = os.path.join(
            PATH, 'this_is_a_robot.sdf')
        mocked_temp.return_value = "/tmp/under/the/rainbow"
        robot_file, _ = self.launcher._get_robot_abs_path(CustomModel())

        self.assertEqual(
            robot_file, os.path.join(PATH, 'this_is_a_robot.sdf'))

    @patch("hbp_nrp_backend.storage_client_api.StorageClient.find_file_in_paths")
    @patch("hbp_nrp_backend.storage_client_api.StorageClient.get_model_basepath")
    def test_robot_path_storage(self, mocked_get_model_basepath, mock_find_file_path, mocked_storage):
        mocked_get_model_basepath.return_value = PATH
        mock_find_file_path.return_value = os.path.join(
            PATH, 'this_is_a_robot.sdf')
        robot = CustomModel()
        mocked_storage().get_folder_uuid_by_name.return_value = 'robots'
        mocked_storage().get_simulation_directory.return_value = PATH
        mocked_storage().get_file.return_value = '<sdf></sdf>'
        robot_file, _ = self.launcher._get_robot_abs_path(robot)
        self.assertEqual(
            robot_file, os.path.join(
                PATH, 'this_is_a_robot.sdf'))

    @patch("tempfile.mkdtemp")
    @patch("zipfile.ZipFile")
    def test_robot_path_storage2(self,  mocked_zip, mocked_temp, mocked_storage):
        robot = CustomModel()
        mocked_storage().get_simulation_directory.return_value = os.path.join(
            PATH, 'husky_model', 'husky_model.zip')

        def val():
            return 'storage://this_is_a_robot.zip'
        robot.value = val
        mocked_temp.return_value = "/tmp/under/the/rainbow"
        mocked_storage().get_folder_uuid_by_name.return_value = 'robots'
        mocked_storage().get_simulation_directory.return_value = PATH
        mocked_storage().get_file.return_value = '<sdf></sdf>'
        robot_file, _ = self.launcher._get_robot_abs_path(robot)
        self.assertEqual(
            robot_file,
            os.path.join(PATH, 'robots', 'this_is_a_robot', 'model.sdf')
        )

        mocked_zip.assert_called_once_with(
            os.path.join(PATH, "this_is_a_robot.zip"))
        mocked_zip().__enter__().getinfo.assert_called_once_with("this_is_a_robot/model.sdf")
        mocked_zip().__enter__().extractall.assert_called_once_with(
            path=os.path.join(PATH, "robots"))

    def test_custom_robot_fails(self, mocked_storage):
        def val():
            return 'robot.sdf'
        robot = CustomModel()
        robot.customModelPath = 'robot.zip'
        robot.value = val
        self.assertRaises(NRPServicesGeneralException, self.launcher._get_robot_abs_path, robot)

    @patch("zipfile.ZipFile")
    def test_custom_robot_succeeds(self, mocked_zip, mocked_storage):
        def val():
            return 'robot.sdf'
        robot = CustomModel()
        robot.customModelPath = 'robot.zip'
        robot.value = val
        mocked_storage().get_custom_models.return_value = [
            dict(path='robot.zip')]
        mocked_storage().get_custom_model.return_value = 'zippedData'
        mocked_storage().get_simulation_directory.return_value = os.path.join(
            PATH, 'experiment_data')
        self.launcher._get_robot_abs_path(robot)
        mocked_zip.assert_called_once_with(
            os.path.join(PATH, 'experiment_data', 'robot.zip'))
        mocked_zip().__enter__().extractall.assert_called_once_with(
            path=os.path.join(PATH, 'experiment_data', 'assets'))

    def test_custom_brain_fails(self, mocked_storage):
        with open(os.path.join(PATH, "experiment_data/milestone2_2.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        self.launcher._SimulationAssembly__bibi = bibi

        def val():
            return 'brain.py'
        brain = CustomModel()
        brain.customModelPath = 'brain.zip'
        brain.value = val
        self.assertRaises(NRPServicesGeneralException, self.launcher._load_brain,
                          brain)

    @patch("zipfile.ZipFile")
    def test_custom_brain_succeeds(self, mocked_zip, mocked_storage):
        mocked_storage().get_custom_models.return_value = [
            dict(path='brain.zip')]
        mocked_storage().get_custom_model.return_value = 'zippedData'
        mocked_storage().get_simulation_directory.return_value = os.path.join(
            PATH, 'experiment_data')
        with open(os.path.join(PATH, "experiment_data/milestone2_2.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        self.launcher._SimulationAssembly__bibi = bibi
        self.launcher._extract_brain_zip()
        mocked_zip.assert_called_once_with(
            os.path.join(PATH, 'experiment_data', 'brain.zip'))

    def test_load_brain_storage(self, mocked_storage):
        mocked_storage().get_folder_uuid_by_name.return_value = 'brains'
        mocked_storage().get_simulation_directory.return_value = PATH
        with open(os.path.join(PATH, 'experiment_data/braitenberg.py')) as brain:
            brain_contents = brain.read()
        mocked_storage().get_file.return_value = brain_contents
        with open(os.path.join(PATH, "experiment_data/milestone2_1.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        self.launcher._SimulationAssembly__bibi = bibi
        braincontrol, braincomm, brainfilepath, neurons_config = self.launcher._load_brain(
            1234)
        self.assertEqual(brainfilepath, os.path.join(PATH, 'braitenberg.py'))
        self.assertEqual(neurons_config, {u'sensors': slice(
            0L, 5L, None), u'actors': slice(5L, 8L, None)})

    def test_invalid_simulation(self, mocked_storage):
        dir = os.path.split(__file__)[0]
        with open(os.path.join(dir, "experiment_data/milestone2.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        with open(os.path.join(dir, "experiment_data/ExDXMLExample.exc")) as exd_file:
            exd = exp_conf_api_gen.CreateFromDocument(exd_file.read())
        exd.path = "/somewhere/over/the/rainbow/exc"
        exd.dir = "/somewhere/over/the/rainbow"
        exd.physicsEngine = None
        bibi.path = "/somewhere/over/the/rainbow/bibi"
        models_path_patch = patch("hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.models_path",
                                  new=None)
        models_path_patch.start()
        with self.assertRaises(Exception):
            SynchronousNestSimulation(42, exd, bibi, gzserver_host="local")
        models_path_patch.stop()

        with self.assertRaises(Exception):
            SynchronousNestSimulation(42, exd, bibi, gzserver_host="bullshit")

    @patch("hbp_nrp_cle.robotsim.RosControlAdapter.RosControlAdapter", new=MockRobotControlAdapter)
    @patch("hbp_nrp_cle.robotsim.RosCommunicationAdapter.RosCommunicationAdapter", new=MockRobotCommunicationAdapter)
    def test_load_robot_adapters(self, mocked_storage):
        (comm, ctrl) = self.launcher._create_robot_adapters()
        self.assertIsNotNone(comm)
        self.assertIsNotNone(ctrl)


class TestCLEGazeboSimulationAssemblyRobot(TestCLEGazeboSimulationAssembly):

    def setUp(self):
        dir = os.path.split(__file__)[0]
        with open(os.path.join(dir, "experiment_data/milestone2.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        with open(os.path.join(dir, "experiment_data/ExDXMLExample.exc")) as exd_file:
            exd = exp_conf_api_gen.CreateFromDocument(exd_file.read())
        exd.path = "/somewhere/over/the/rainbow/exc"
        exd.dir = "/somewhere/over/the/rainbow"
        bibi.path = "/somewhere/over/the/rainbow/bibi"
        self.models_path_patch = patch(
            "hbp_nrp_cleserver.server.CLEGazeboSimulationAssembly.models_path", new="/somewhere/near/the/rainbow")
        self.models_path_patch.start()
        self.launcher = SynchronousRobotRosNest(
            42, exd, bibi, gzserver_host="local")

    @patch("hbp_nrp_cle.robotsim.RosRobotControlAdapter.RosRobotControlAdapter", new=MockRobotControlAdapter)
    @patch("hbp_nrp_cle.robotsim.RosCommunicationAdapter.RosCommunicationAdapter", new=MockRobotCommunicationAdapter)
    def test_load_robot_adapters(self):
        (comm, ctrl) = self.launcher._create_robot_adapters()
        self.assertIsNotNone(comm)
        self.assertIsNotNone(ctrl)


if __name__ == '__main__':
    unittest.main()