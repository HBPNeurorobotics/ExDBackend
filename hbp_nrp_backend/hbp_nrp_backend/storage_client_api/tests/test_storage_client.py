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
Storage client unit test
"""
import inspect
import unittest
import shutil
import os
import requests
from mock import patch, MagicMock, mock_open, call, Mock
from hbp_nrp_commons.generated import bibi_api_gen, exp_conf_api_gen
from hbp_nrp_backend.rest_server import app, NRPServicesGeneralException
from hbp_nrp_backend.storage_client_api import StorageClient
from hbp_nrp_backend.storage_client_api.StorageClient import _FlattenedExperimentDirectory

# Used to mock all the http requests by providing a response and a
# status code


class MockResponse:

    def __init__(self, json_data, status_code, text=None):
        self.json_data = json_data
        self.status_code = status_code
        self.text = text

    def json(self):
        return self.json_data

# Functions that return fake responses based on the mock response class

# AUTHENTICATE HTTP RESPONSES


def mocked_authenticate_post_ok(*args, **kwargs):
    return MockResponse({"token": "FakeToken"}, 200)


def mocked_request_not_ok(*args, **kwargs):
    return MockResponse(None, 404)

# GET EXPERIMENT HTTP RESPONSES


def mocked_get_experiments_ok(*args, **kwargs):
    response = [
        {
            "uuid": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
            "name": "New Experiment_3",
            "parent": "89857775-6215-4d53-94ee-fb6c18b9e2f8"
        },
        {
            "uuid": "b246cc8e-d844-4826-ae5b-d2c023b893d8",
            "name": "New Experiment_2",
            "parent": "89857775-6215-4d53-94ee-fb6c18b9e2f8"
        }
    ]
    return MockResponse(response, 200)


create_experiment_response = {
    "uuid": "8cb4fbea-f3cf-4ade-ad46-a570a1ab3b15",
    "entity_type": "folder",
    "name": "experiment",
    "description": "",
    "parent": "89857775-6215-4d53-94ee-fb6c18b9e2f8",
    "created_by": "302416",
    "created_on": "2017-09-12T08:15:39.471269Z",
    "modified_by": "302416",
    "modified_on": "2017-09-12T08:15:39.471353Z"
}

# CREATE EXPERIMENT HTTP RESPONSES


def mocked_create_experiment_ok(*args, **kwargs):
    return MockResponse(create_experiment_response, 200, "8cb4fbea-f3cf-4ade-ad46-a570a1ab3b15")


def mocked_create_experiment_exists(*args, **kwargs):
    return MockResponse(create_experiment_response, 200, "Experiment already exists")

# DELETE FILE HTTP RESPONSES


def mocked_delete_experiment_ok(*args, **kwargs):
    return MockResponse("Success", 200)

# CREATE OR UPDATE HTTP RESPONSES


def mocked_create_or_update_ok(*args, **kwargs):
    return MockResponse({
        "uuid": "8b4b993a-1324-4dbd-bbc1-91c85a996792"
    }, 200)

# CREATE FOLDER HTTP RESPONSES


def mocked_create_folder_ok(*args, **kwargs):
    return MockResponse({
        "uuid": "5b1a2363-1529-40cd-a8b7-94bfd6dea23d",
        "entity_type": "folder",
        "name": "fakeFolder",
        "description": "",
        "parent": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
        "created_by": "302416",
        "created_on": "2017-09-12T16:46:07.506604Z",
        "modified_by": "302416",
        "modified_on": "2017-09-12T16:46:07.506664Z"
    }, 200)

# LIST FILES HTTP RESPONSES


def mocked_list_files_ok(*args, **kwargs):
    return MockResponse([{
        "uuid": "07b35b8f-67cd-4e94-8bec-5ede8049590d",
        "name": "env_editor.autosaved",
        "parent": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
        "contentType": "text/plain",
        "type": "file",
        "modifiedOn": "2017-08-31T13:56:34.306090Z"
    },
        {
            "uuid": "6a63d03e-6dad-4793-80d7-8e32a83ddd14",
            "name": "simple_move_robot.py",
            "parent": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
            "contentType": "application/hbp-neurorobotics.tfs+python",
            "type": "file",
            "modifiedOn": "2017-08-30T12:32:47.842214Z"
        }], 200)


class TestNeuroroboticsStorageClient(unittest.TestCase):

    def setUp(self):
        self.models_directory = os.path.join(
            os.path.dirname(inspect.getfile(self.__class__)), 'mocked_models'
        )
        self.experiments_directory = os.path.join(
            os.path.dirname(inspect.getfile(
                self.__class__)), 'experiment_files'
        )
        self.temporary_directory_to_clean = []

    def tearDown(self):
        for dir in self.temporary_directory_to_clean:
            if dir.startswith('/tmp'):
                shutil.rmtree(dir)
        self.temporary_directory_to_clean = []

    # TEMP DIRECTORY
    def test_temp_folder_exists(self):
        with patch('os.listdir', return_value=['/tmp/other', '/tmp/nrpTemp']) as mock_temp, \
                patch('os.path.join', return_value='/tmp/nrpTemp') as mock_join, \
                patch('tempfile.mkdtemp', return_value='/tmp/nrpTemp') as mock_temp_make:

            client = StorageClient.StorageClient()
            fake_temp_directory = client.get_temp_directory()
            self.assertEqual(fake_temp_directory, '/tmp/nrpTemp')

    def test_temp_folder_not_exists(self):
        with patch('os.listdir', return_value=['/tmp/other']) as mock_temp, \
                patch('tempfile.mkdtemp', return_value='/tmp/nrpTemp') as mock_temp_make:

            client = StorageClient.StorageClient()
            fake_temp_directory = client.get_temp_directory()
            self.assertEqual(fake_temp_directory, '/tmp/nrpTemp')

    # AUTHENTICATE
    @patch('requests.post', side_effect=mocked_authenticate_post_ok)
    def test_authenticate_successfully(self, mocked_post):
        client = StorageClient.StorageClient()
        res = client.authenticate("fakeUser", "fakePassword")
        self.assertEqual(res, {'token': 'FakeToken'})

    @patch('requests.post', side_effect=mocked_request_not_ok)
    def test_authenticate_not_ok(self, mocked_post):
        client = StorageClient.StorageClient()
        with self.assertRaises(Exception) as context:
            client.authenticate("non_existing_user", "non_existing_password")

        self.assertTrue(
            'Failed to communicate with the storage server, status code 404' in context.exception)

    @patch('requests.post')
    def test_authenticate_connection_error(self, mocked_post):
        client = StorageClient.StorageClient()
        mocked_post.side_effect = requests.exceptions.ConnectionError()
        with self.assertRaises(requests.exceptions.ConnectionError) as context:
            client.authenticate("non_existing_user", "non_existing_password")
        self.assertEqual(requests.exceptions.ConnectionError, context.expected)

    # LIST EXPERIMENTS
    @patch('requests.get', side_effect=mocked_get_experiments_ok)
    def test_get_experiments_successfully(self, mocked_get):
        client = StorageClient.StorageClient()
        res = client.list_experiments("fakeToken", 'ctx')
        self.assertEqual(res[0]['name'], "New Experiment_3")
        self.assertEqual(
            res[1]['uuid'], "b246cc8e-d844-4826-ae5b-d2c023b893d8")

    @patch('requests.get', side_effect=mocked_request_not_ok)
    def test_get_experiments_failed(self, mocked_get):
        client = StorageClient.StorageClient()
        with self.assertRaises(Exception) as context:
            client.list_experiments("fakeToken", 'ctx')

        self.assertTrue(
            'Failed to communicate with the storage server, status code 404' in context.exception)

    @patch('requests.get')
    def test_get_experiment_connection_error(self, mocked_get):
        client = StorageClient.StorageClient()
        mocked_get.side_effect = requests.exceptions.ConnectionError()
        with self.assertRaises(requests.exceptions.ConnectionError) as context:
            client.list_experiments("fakeToken", 'ctx')
        self.assertEqual(requests.exceptions.ConnectionError, context.expected)

    # CREATE EXPERIMENT
    @patch('requests.put', side_effect=mocked_create_experiment_ok)
    def test_create_experiment_successfully(self, mocked_put):
        client = StorageClient.StorageClient()
        res = client.create_experiment(
            "fakeToken", "fakeExperiment", "fakeContextId")
        self.assertEqual(
            res, "8cb4fbea-f3cf-4ade-ad46-a570a1ab3b15")

    @patch('requests.put', side_effect=mocked_create_experiment_exists)
    def test_create_experiment_exists(self, mocked_put):
        client = StorageClient.StorageClient()
        res = client.create_experiment(
            "fakeToken", "fakeExperiment", "fakeContextId")
        self.assertEqual(
            res, None)

    @patch('requests.put', side_effect=mocked_request_not_ok)
    def test_create_experiment_fail(self, mocked_put):
        client = StorageClient.StorageClient()
        with self.assertRaises(Exception) as context:
            client.create_experiment(
                "fakeToken", "fakeExperiment", "fakeContextId")

        self.assertTrue(
            'Failed to communicate with the storage server, status code 404' in context.exception)

    @patch('requests.put')
    def test_create_experiment_connection_error(self, mocked_put):
        client = StorageClient.StorageClient()
        mocked_put.side_effect = requests.exceptions.ConnectionError()
        with self.assertRaises(requests.exceptions.ConnectionError) as context:
            client.create_experiment(
                "fakeToken", "fakeExperiment", "fakeContextId")
        self.assertEqual(requests.exceptions.ConnectionError, context.expected)

    # GET FILE
    @patch('requests.get')
    def test_get_file_by_name_successfully(self, mocked_get):
        client = StorageClient.StorageClient()

        def get_fake_experiment_file(token, headers):
            with open(os.path.join(self.experiments_directory, "experiment_configuration.exc")) as exd_file:
                exp_file_contents = exd_file.read()
                return MockResponse(None, 200, exp_conf_api_gen.CreateFromDocument(exp_file_contents))

        mocked_get.side_effect = get_fake_experiment_file
        res = client.get_file(
            "fakeToken", "fakeExperiment", "experiment_configuration.exc", byname=True)

        self.assertEqual(res.name, "Baseball tutorial experiment - Exercise")

    @patch('requests.get')
    def test_get_file_name_successfully(self, mocked_get):
        client = StorageClient.StorageClient()

        def get_fake_experiment_file(token, headers):
            with open(os.path.join(self.experiments_directory, "experiment_configuration.exc")) as exd_file:
                exp_file_contents = exd_file.read()
                return MockResponse(None, 200, exp_conf_api_gen.CreateFromDocument(exp_file_contents))

        mocked_get.side_effect = get_fake_experiment_file
        res = client.get_file(
            "fakeToken", "fakeExperiment", "experiment_configuration.exc")
        self.assertEqual(res.maturity, "production")

    @patch('requests.get', side_effect=mocked_request_not_ok)
    def test_get_file_fail(self, mocked_put):
        client = StorageClient.StorageClient()
        with self.assertRaises(Exception) as context:
            client.get_file(
                "fakeToken", "fakeExperiment", "experiment_configuration.exc")

        self.assertTrue(
            'Failed to communicate with the storage server, status code 404' in context.exception)

    @patch('requests.get')
    def test_get_file_connection_error(self, mocked_put):
        client = StorageClient.StorageClient()
        mocked_put.side_effect = requests.exceptions.ConnectionError()
        with self.assertRaises(requests.exceptions.ConnectionError) as context:
            client.get_file(
                "fakeToken", "fakeExperiment", "experiment_configuration.exc")

        self.assertEqual(requests.exceptions.ConnectionError, context.expected)

    # DELETE FILE
    @patch('requests.delete', side_effect=mocked_delete_experiment_ok)
    def test_delete_file_successfully(self, mocked_delete):
        client = StorageClient.StorageClient()
        res = client.delete_file(
            "fakeToken", "fakeExperiment", "experiment_configuration.exc")
        self.assertEqual(res, "Success")

    @patch('requests.delete', side_effect=mocked_request_not_ok)
    def test_delete_file_failed(self, mocked_delete):
        client = StorageClient.StorageClient()
        with self.assertRaises(Exception) as context:
            client.delete_file(
                "fakeToken", "fakeExperiment", "experiment_configuration.exc")

        self.assertTrue(
            'Failed to communicate with the storage server, status code 404' in context.exception)

    @patch('requests.delete')
    def test_delete_file_connection_error(self, mocked_put):
        client = StorageClient.StorageClient()
        mocked_put.side_effect = requests.exceptions.ConnectionError()
        with self.assertRaises(requests.exceptions.ConnectionError) as context:
            client.delete_file(
                "fakeToken", "fakeExperiment", "experiment_configuration.exc")

        self.assertEqual(requests.exceptions.ConnectionError, context.expected)

    # CREATE OR UPDATE
    @patch('requests.post', side_effect=mocked_create_or_update_ok)
    def test_create_or_update_successfully(self, mocked_post):
        client = StorageClient.StorageClient()
        res = client.create_or_update(
            "fakeToken",
            "fakeExperiment",
            "experiment_configuration.exc",
            "FakeContent",
            "text/plain")
        self.assertEqual(res, 200)

    @patch('requests.post', side_effect=mocked_request_not_ok)
    def test_create_or_update_failed(self, mocked_post):
        client = StorageClient.StorageClient()
        with self.assertRaises(Exception) as context:
            client.create_or_update(
                "fakeToken",
                "fakeExperiment",
                "experiment_configuration.exc",
                "FakeContent",
                "text/plain")

        self.assertTrue(
            'Failed to communicate with the storage server, status code 404' in context.exception)

    @patch('requests.post')
    def test_create_or_update_connection_error(self, mocked_post):
        client = StorageClient.StorageClient()
        mocked_post.side_effect = requests.exceptions.ConnectionError()
        with self.assertRaises(requests.exceptions.ConnectionError) as context:
            client.create_or_update(
                "fakeToken",
                "fakeExperiment",
                "experiment_configuration.exc",
                "FakeContent",
                "text/plain")
        self.assertEqual(requests.exceptions.ConnectionError, context.expected)

    # CREATE FOLDER
    @patch('requests.post', side_effect=mocked_create_folder_ok)
    def test_create_folder_successfully(self, mocked_post):
        client = StorageClient.StorageClient()
        res = client.create_folder(
            "fakeToken",
            "fakeExperiment",
            "fakeName")
        self.assertEqual(res['uuid'], '5b1a2363-1529-40cd-a8b7-94bfd6dea23d')
        self.assertEqual(res['name'], 'fakeFolder')

    @patch('requests.post', side_effect=mocked_request_not_ok)
    def test_create_folder_failed(self, mocked_post):
        client = StorageClient.StorageClient()
        with self.assertRaises(Exception) as context:
            res = client.create_folder(
                "fakeToken",
                "fakeExperiment",
                "fakeName")

        self.assertTrue(
            'Failed to communicate with the storage server, status code 404' in context.exception)

    @patch('requests.post')
    def test_create_folder_connection_error(self, mocked_post):
        client = StorageClient.StorageClient()
        mocked_post.side_effect = requests.exceptions.ConnectionError()
        with self.assertRaises(requests.exceptions.ConnectionError) as context:
            client.create_folder(
                "fakeToken",
                "fakeExperiment",
                "fakeName")
        self.assertEqual(requests.exceptions.ConnectionError, context.expected)

    # LIST FILES
    @patch('requests.get', side_effect=mocked_list_files_ok)
    def test_list_files_successfully(self, mocked_post):
        client = StorageClient.StorageClient()
        res = client.list_files(
            "fakeToken",
            "fakeExperiment")
        self.assertEqual(
            res[0]['uuid'], '07b35b8f-67cd-4e94-8bec-5ede8049590d')
        self.assertEqual(res[1]['name'], 'simple_move_robot.py')

    @patch('requests.get', side_effect=mocked_request_not_ok)
    def test_list_files_failed(self, mocked_post):
        client = StorageClient.StorageClient()
        with self.assertRaises(Exception) as context:
            client.list_files(
                "fakeToken",
                "fakeExperiment")

        self.assertTrue(
            'Failed to communicate with the storage server, status code 404' in context.exception)

    @patch('requests.get')
    def test_create_folder_connection_error(self, mocked_post):
        client = StorageClient.StorageClient()
        mocked_post.side_effect = requests.exceptions.ConnectionError()
        with self.assertRaises(requests.exceptions.ConnectionError) as context:
            client.list_files(
                "fakeToken",
                "fakeExperiment")
        self.assertEqual(requests.exceptions.ConnectionError, context.expected)

    # CLONE FILE
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.list_files')
    def test_clone_does_not_exist(self, mocked_list):
        client = StorageClient.StorageClient()
        mocked_list.return_value = [{
            "uuid": "07b35b8f-67cd-4e94-8bec-5ede8049590d",
            "name": "env_editor.autosaved",
            "parent": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
            "contentType": "text/plain",
            "type": "file",
            "modifiedOn": "2017-08-31T13:56:34.306090Z"
        },
            {
                "uuid": "6a63d03e-6dad-4793-80d7-8e32a83ddd14",
                "name": "simple_move_robot.py",
                "parent": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
                "contentType": "application/hbp-neurorobotics.tfs+python",
                "type": "file",
                "modifiedOn": "2017-08-30T12:32:47.842214Z"
            }]
        res = client.clone_file("fakeFile",
                                "fakeToken",
                                "fakeExperiment")
        self.assertEqual(res, None)

    # PARSE AND CHECK IS VALID
    def test_parse_and_check(self):
        client = StorageClient.StorageClient()
        experiment_path = os.path.join(
            self.experiments_directory, "experiment_configuration.exc")
        res = client.parse_and_check_file_is_valid(experiment_path,
                                                   exp_conf_api_gen.CreateFromDocument,
                                                   exp_conf_api_gen.ExD_)
        self.assertEqual(res.name, 'Baseball tutorial experiment - Exercise')

    # CLONE ALL EXPERIMENT FILES
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.list_files')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.get_file')
    def test_clone_all_experiment_files(self, mocked_list, mocked_get):
        with patch('tempfile.mkdtemp', return_value='/tmp/nrpTemp') as mock_temp_make:
            mocked_get.side_effect = None
            client = StorageClient.StorageClient()
            experiment_path = os.path.join(
                self.experiments_directory, "experiment_configuration.exc")
            mocked_list.return_value = [{
                "uuid": "07b35b8f-67cd-4e94-8bec-5ede8049590d",
                "name": "env_editor.autosaved",
                "parent": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
                "contentType": "text/plain",
                "type": "file",
                "modifiedOn": "2017-08-31T13:56:34.306090Z"
            },
                {
                    "uuid": "6a63d03e-6dad-4793-80d7-8e32a83ddd14",
                    "name": "simple_move_robot.py",
                    "parent": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
                    "contentType": "application/hbp-neurorobotics.tfs+python",
                    "type": "file",
                    "modifiedOn": "2017-08-30T12:32:47.842214Z"
                }]
            with patch("__builtin__.open", mock_open(read_data="data")) as mock_file:
                res = client.clone_all_experiment_files("fakeToken",
                                                        "fakeExperiment")

                self.assertIn('nrpTemp', res[0])

    @patch('hbp_nrp_backend.storage_client_api.StorageClient.get_experiment_basepath')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.get_model_basepath')
    def test_flatten_bibi_configuration(self, get_model_basepath_mock, get_experiment_basepath_mock):
        get_model_basepath_mock.return_value = self.models_directory
        get_experiment_basepath_mock.return_value = self.experiments_directory
        exp_configuration = 'ExDXMLExample.exc'
        expected_list = ['experiment_configuration.exc',
                         'bibi_configuration.bibi',
                         'braitenberg_husky_linear_twist.py',
                         'csv_spike_monitor.py',
                         'model.sdf',
                         'braitenberg.py',
                         'robot.config',
                         'csv_joint_state_monitor.py',
                         'ExDXMLExample.3ds',
                         'ExDXMLExample.png',
                         'robot_description.launch',
                         'virtual_room.sdf',
                         'csv_robot_position.py']

        with _FlattenedExperimentDirectory(exp_configuration, None) as temporary_folder:
            bibi_configuration_file = 'milestone2_python_tf.bibi'
            l = os.listdir(temporary_folder)
            # make sure we do not copy things we aren't expecting
            self.assertEqual(len(l), len(expected_list))
            for expected_file in expected_list:
                self.assertIn(expected_file, l)
            flattened_bibi_configuration_file = os.path.join(
                temporary_folder,
                'bibi_configuration.bibi'
            )
            flattened_exp_configuration_file = os.path.join(
                temporary_folder,
                'experiment_configuration.exc'
            )
            with open(flattened_exp_configuration_file) as e:
                exp_configuration_dom = exp_conf_api_gen.CreateFromDocument(
                    e.read())

            with open(flattened_bibi_configuration_file) as b:
                bibi_configuration_dom = bibi_api_gen.CreateFromDocument(
                    b.read())

            for tf in bibi_configuration_dom.transferFunction:
                if hasattr(tf, "src") and tf.src:
                    self.assertEqual(tf.src, os.path.basename(tf.src))
                    path = os.path.join(temporary_folder, tf.src)
                    self.assertEqual(os.path.exists(path), True)

            self.assertEqual(
                bibi_configuration_dom.bodyModel,
                bibi_api_gen.SDFFilename('model.sdf')
            )
            self.assertEqual(
                bibi_configuration_dom.brainModel.file,
                bibi_api_gen.PythonFilename('braitenberg.py')
            )

    @patch('hbp_nrp_backend.storage_client_api.StorageClient.get_experiment_basepath')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.get_model_basepath')
    def test_flattened_parse_model_config_files(self, get_model_basepath_mock, get_experiment_basepath_mock):
        expected_list = ['fakeRobot.sdf',
                         'experiment_configuration.exc',
                         'TemplateEmpty.png',
                         'TemplateEmpty.json',
                         'bibi_configuration.bibi',
                         'fakeEnv.sdf',
                         'fakeBrain.py']
        get_model_basepath_mock.return_value = self.models_directory
        get_experiment_basepath_mock.return_value = self.experiments_directory
        exp_configuration = 'experiment_configuration.exc'
        mock_paths = {
            "envPath": os.path.join("fakeEnvFolder", "model.config"),
            "robotPath": os.path.join("fakeRobotFolder", "model.config"),
            "brainPath": os.path.join("fakeBrainFolder", "fakeBrain.py")
        }
        with _FlattenedExperimentDirectory(exp_configuration, mock_paths) as temporary_folder:
            self.assertEqual('fakeRobot.sdf' in os.listdir(
                temporary_folder), True)
            self.assertEqual('experiment_configuration.exc' in os.listdir(
                temporary_folder), True)
            self.assertEqual('bibi_configuration.bibi' in os.listdir(
                temporary_folder), True)
            self.assertEqual('fakeBrain.py' in os.listdir(
                temporary_folder), True)

    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.list_experiments')
    def test_create_unique_experiment_id(self, mocked_list):
        mocked_list.return_value = [
            "Experiment",  "Experiment_2"]
        client = StorageClient.StorageClient()
        unique_exp_name = client.create_unique_experiment_id(
            'Experiment', 0, mocked_list)
        self.assertEqual(unique_exp_name, 'Experiment_0')

    @patch('os.environ.get')
    def test_get_model_basepath_ok(self, mock_env_get):
        mock_env_get.return_value = 'NRP/Models'
        path = StorageClient.get_model_basepath()
        self.assertEqual('NRP/Models', path)

    @patch('os.environ.get')
    def test_get_model_basepath_ok(self, mock_env_get):
        mock_env_get.return_value = None
        self.assertRaises(Exception, StorageClient.get_model_basepath)

    @patch('hbp_nrp_backend.storage_client_api.StorageClient.get_experiment_basepath')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.create_experiment')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.create_or_update')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.list_experiments')
    def test_clone_experiment_to_storage(self, mock_list, mock_create_update, mock_create_exp, get_experiment_basepath_mock):
        get_experiment_basepath_mock.return_value = self.experiments_directory
        mock_create_exp.return_value = 'Fake UUID'
        mock_create_update.return_value = None
        mock_list.return_value = [
            {"name": "Experiment"}, {"name": "Experiment_1"}]
        client = StorageClient.StorageClient()
        exp_configuration = 'ExDXMLExample.exc'
        exp_uuid = client.clone_experiment_template_to_storage(
            'token', exp_configuration)
        self.assertEqual('Fake UUID', exp_uuid)


if __name__ == '__main__':
    unittest.main()
