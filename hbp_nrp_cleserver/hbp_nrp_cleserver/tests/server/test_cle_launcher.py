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
from hbp_nrp_cleserver.server import CLELauncher
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen

MockOs = Mock()
MockOs.environ = {'NRP_MODELS_DIRECTORY': '/somewhere/near/the/rainbow',
                  'ROS_MASTER_URI': "localhost:0815"}
MockOs.path.join.return_value = "/a/really/nice/place"

PATH = os.path.split(__file__)[0]


class TestCLELauncher(unittest.TestCase):
    def setUp(self):
        dir = os.path.split(__file__)[0]
        with open(os.path.join(dir, "experiment_data/milestone2.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        with open(os.path.join(dir, "experiment_data/ExDXMLExample.exc")) as exd_file:
            exd = exp_conf_api_gen.CreateFromDocument(exd_file.read())
        with patch("hbp_nrp_cleserver.server.CLELauncher.os", MockOs):
            self.launcher = CLELauncher.CLELauncher(
                exd, bibi, "/somewhere/over/the/rainbow", "gz_host", None, 42, None, None)
        self.launcher.models_path = "models_path"

    def test_robot_path_sdf(self):
        robot_file = self.launcher._get_robot_abs_path(
            "robots/this_is_a_robot.sdf")
        self.assertEqual(robot_file, "models_path/robots/this_is_a_robot.sdf")
        self.assertIsNone(self.launcher._CLELauncher__tmp_robot_dir)

    @patch("tempfile.mkdtemp")
    @patch("zipfile.ZipFile")
    def test_robot_path_zip(self, mocked_zip, mocked_temp):
        mocked_temp.return_value = "/tmp/under/the/rainbow"
        robot_file = self.launcher._get_robot_abs_path(
            "robots/this_is_a_robot.zip")

        self.assertEqual(
            self.launcher._CLELauncher__tmp_robot_dir, "/tmp/under/the/rainbow")
        self.assertEqual(
            robot_file, "/tmp/under/the/rainbow/this_is_a_robot/model.sdf")

        self.assertTrue(mocked_temp.called)
        mocked_zip.assert_called_once_with(
            "models_path/robots/this_is_a_robot.zip")
        mocked_zip().__enter__().getinfo.assert_called_once_with("this_is_a_robot/model.sdf")
        mocked_zip().__enter__().extractall.assert_called_once_with(
            path="/tmp/under/the/rainbow")

    @patch("hbp_nrp_backend.storage_client_api.StorageClient.StorageClient")
    def test_robot_path_storage(self, mocked_storage):
        mocked_storage().get_folder_uuid_by_name.return_value = 'robots'
        mocked_storage().get_temp_directory.return_value = PATH
        mocked_storage().get_file.return_value = '<sdf></sdf>'
        robot_file = self.launcher._get_robot_abs_path(
            "storage://this_is_a_robot.sdf")

        self.assertEqual(
            robot_file, os.path.join(PATH, 'this_is_a_robot.sdf'))

    @patch("hbp_nrp_backend.storage_client_api.StorageClient.StorageClient")
    @patch("tempfile.mkdtemp")
    @patch("zipfile.ZipFile")
    def test_robot_path_storage(self, mocked_zip, mocked_temp, mocked_storage):
        mocked_temp.return_value = "/tmp/under/the/rainbow"
        mocked_storage().get_folder_uuid_by_name.return_value = 'robots'
        mocked_storage().get_temp_directory.return_value = PATH
        mocked_storage().get_file.return_value = '<sdf></sdf>'
        robot_file = self.launcher._get_robot_abs_path(
            "storage://this_is_a_robot.zip")

        self.assertEqual(
            robot_file, "/tmp/under/the/rainbow/this_is_a_robot/model.sdf")
        
        mocked_zip.assert_called_once_with(
            os.path.join(PATH,"this_is_a_robot.zip"))
        mocked_zip().__enter__().getinfo.assert_called_once_with("this_is_a_robot/model.sdf")
        mocked_zip().__enter__().extractall.assert_called_once_with(
            path="/tmp/under/the/rainbow")
    
    @patch("hbp_nrp_backend.storage_client_api.StorageClient.StorageClient")
    def test_load_brain_storage(self, mocked_storage):
        mocked_storage().get_folder_uuid_by_name.return_value = 'brains'
        mocked_storage().get_temp_directory.return_value = PATH
        with open(os.path.join(PATH,'experiment_data/braitenberg.py')) as brain:
            brain_contents = brain.read()
        mocked_storage().get_file.return_value = brain_contents
        with open(os.path.join(PATH, "experiment_data/milestone2_1.bibi")) as bibi_file:
            bibi = bibi_api_gen.CreateFromDocument(bibi_file.read())
        self.launcher._CLELauncher__bibi_conf = bibi
        braincontrol, braincomm, brainfilepath, neurons_config = self.launcher._CLELauncher__load_brain(1234)
        self.assertEqual(brainfilepath, os.path.join(PATH,'braitenberg.py'))
        self.assertEqual(neurons_config, {u'sensors': slice(0L, 5L, None), u'actors': slice(5L, 8L, None)})
