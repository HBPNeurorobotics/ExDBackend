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
from mock import patch, MagicMock, mock_open, Mock
from hbp_nrp_commons.generated import exp_conf_api_gen
from hbp_nrp_backend.storage_client_api import StorageClient
from hbp_nrp_backend import NRPServicesGeneralException
from hbp_nrp_commons.sim_config.SimConfig import ResourceType
from hbp_nrp_commons.workspace.SimUtil import SimUtil

# Used to mock all the http requests by providing a response and a
# status code


class MockResponse:

    def __init__(self, json_data, status_code, text=None, content=None, raw=None):
        self.json_data = json_data
        self.status_code = status_code
        self.text = text
        self.content = content
        self.raw = raw

    def json(self):
        return self.json_data

# Functions that return fake responses based on the mock response class

# GET USER HTTP RESPONSES

def mocked_get_user_ok(*args, **kwargs):
    return MockResponse({"id": "fake_id"}, 200)


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
    },
        {
            "uuid": "6a63d03e-6dad-4793-80d7-8e32a83ddd13",
            "name": "resources",
            "parent": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
            "contentType": "application/hbp-neurorobotics.tfs+python",
            "type": "folder",
            "modifiedOn": "2017-08-30T12:32:47.842214Z"
    }], 200)


def mocked_get_models_ok(*args, **kwargs):
    return MockResponse([{'name': 'testZip1', 'type': 'robots', 'path': 'somepath'},
                        {'name': 'testZip2', 'type': 'robots', 'path': 'somepath'}], 200)


def mocked_get_model_ok(*args, **kwargs):
    return MockResponse({'name': 'testZip1'}, 200, content='Test')


def mocked_get_texture_ok(*args, **kwargs):
    return MockResponse({'name': 'texture.png'}, 200, content='imageData')


class TestStorageClient(unittest.TestCase):

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

    # GET USER
    @patch('requests.get', side_effect=mocked_get_user_ok)
    def test_get_user_successfully(self, mocked_get):
        client = StorageClient.StorageClient()
        res = client.get_user("faketoken")
        self.assertEqual(res, {"id": "fake_id"})

    @patch('requests.get', side_effect=mocked_request_not_ok)
    def test_get_user_not_ok(self, mocked_get):
        client = StorageClient.StorageClient()
        with self.assertRaises(Exception) as context:
            client.get_user("wrong token")

        self.assertTrue(
            'Could not verify auth token, status code 404' in context.exception)

    @patch('requests.get')
    def test_get_user_connection_error(self, mocked_get):
        client = StorageClient.StorageClient()
        mocked_get.side_effect = requests.exceptions.ConnectionError()
        with self.assertRaises(requests.exceptions.ConnectionError) as context:
            client.get_user("wrong token")
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

    # GET FILE
    @patch('requests.get')
    def test_get_file_by_name_successfully(self, mocked_get):
        client = StorageClient.StorageClient()

        def get_fake_experiment_file(token, headers, stream):
            with open(os.path.join(self.experiments_directory, "experiment_configuration.exc")) as exd_file:
                exp_file_contents = exd_file.read()
                return MockResponse(None, 200, exp_conf_api_gen.CreateFromDocument(exp_file_contents))

        mocked_get.side_effect = get_fake_experiment_file
        res = client.get_file(
            "fakeToken", "fakeExperiment", "experiment_configuration.exc", by_name=True)

        self.assertEqual(res.name, "Baseball tutorial experiment - Exercise")

    @patch('requests.get')
    def test_get_texture_by_name_successfully(self, mocked_get):
        client = StorageClient.StorageClient()

        class Object(object):
            pass

        def get_fake_texture_file(token, headers, stream):
            raw_response = Object()
            raw_response.decode_content = True
            return MockResponse(None, 200, None, None, raw_response)

        mocked_get.side_effect = get_fake_texture_file
        res = client.get_file(
            "fakeToken", "fakeExperiment", "fake.png", by_name=True, is_fileobject=True)
        self.assertIsInstance(res, Object)

    @patch('requests.get')
    def test_get_zip_by_name_successfully(self, mocked_get):
        client = StorageClient.StorageClient()

        class Object(object):
            pass

        def get_fake_zip_file(token, headers, stream):
            content_response = Object()
            return MockResponse(None, 200, None, content_response, None)

        mocked_get.side_effect = get_fake_zip_file
        res = client.get_file(
            "fakeToken", "fakeExperiment", "fake.zip", by_name=True, zipped=True)
        self.assertIsInstance(res, Object)

    @patch('requests.get')
    def test_get_file_name_successfully(self, mocked_get):
        client = StorageClient.StorageClient()

        def get_fake_experiment_file(token, headers, stream):
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

    def test_filter_textures(self):
        textures_names = [
            'a.txt',
            'b',
            'c.jpg',
            'd.jpeg',
            'e.png',
            'f.gif',
            'g.zip',
            ]
        textures = [{'name': name} for name in textures_names]
        filtered_textures = StorageClient.StorageClient.filter_textures(textures)
        expectation = textures[2:6]
        self.assertEqual(filtered_textures, expectation)


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

    # GET CUSTOM MODELS
    @patch('requests.get', side_effect=mocked_get_models_ok)
    def test_get_models_successfully(self, mocked_get):
        client = StorageClient.StorageClient()
        res = client.get_models(
            "fakeToken",
            "fakeContextId",
            ResourceType.ROBOT)
        self.assertEqual(res[0].name, 'testZip1')

    @patch('requests.get', side_effect=mocked_request_not_ok)
    def test_get_models_failed(self, mocked_get):
        client = StorageClient.StorageClient()
        with self.assertRaises(Exception) as context:
            client.get_models(
                "fakeToken",
                "fakeContextId",
                ResourceType.ROBOT)

        self.assertTrue(
            'Failed to communicate with the storage server, status code 404' in context.exception)

    @patch('requests.get')
    def test_get_models_connection_error(self, mocked_get):
        client = StorageClient.StorageClient()
        mocked_get.side_effect = requests.exceptions.ConnectionError()
        with self.assertRaises(requests.exceptions.ConnectionError) as context:
            client.get_models(
                "fakeToken",
                "fakeContextId",
                ResourceType.ROBOT)
        self.assertEqual(requests.exceptions.ConnectionError, context.expected)

    # GET CUSTOM MODEL
    @patch('requests.get', side_effect=mocked_get_model_ok)
    def test_get_model_successfully(self, mocked_get):
        client = StorageClient.StorageClient()
        model = MagicMock()
        model.name = 'model_brain'
        model.type = ResourceType.BRAIN
        res = client.get_model(
            "fakeToken",
            "fakeContextId",
            model)
        self.assertEqual(res, 'Test')

    @patch('requests.get', side_effect=mocked_request_not_ok)
    def test_get_custom_model_failed(self, mocked_get):
        client = StorageClient.StorageClient()
        model = MagicMock()
        model.name = 'model_brain'
        model.type = ResourceType.BRAIN
        with self.assertRaises(Exception) as context:
            client.get_model(
                "fakeToken",
                "fakeContextId",
                model
                )

        self.assertTrue(
            'Failed to communicate with the storage server, status code 404' in context.exception)

    @patch('requests.get')
    def test_get_model_connection_error(self, mocked_get):
        client = StorageClient.StorageClient()
        mocked_get.side_effect = requests.exceptions.ConnectionError()
        model = MagicMock()
        model.name = 'model_brain'
        model.type = ResourceType.BRAIN
        with self.assertRaises(requests.exceptions.ConnectionError) as context:
            client.get_model(
                "fakeToken",
                "fakeContextId",
                 model,
                )
        self.assertEqual(requests.exceptions.ConnectionError, context.expected)

    # GET TEXTURES
    @patch('requests.get', side_effect=mocked_get_texture_ok)
    def test_get_texture_successfully(self, mocked_get):
        client = StorageClient.StorageClient()
        res = client.get_textures(
            "fakeToken",
            "fakeExperiment")
        self.assertEqual(res, {"name": 'texture.png'})

    @patch('requests.get', side_effect=mocked_request_not_ok)
    def test_get_texture_failed(self, mocked_get):
        client = StorageClient.StorageClient()
        with self.assertRaises(Exception) as context:
            client.get_textures(
                "fakeToken",
                "fakeContextId")

        self.assertTrue(
            'Failed to communicate with the storage server, status code 404' in context.exception)

    @patch('requests.get')
    def test_get_texture_connection_error(self, mocked_get):
        client = StorageClient.StorageClient()
        mocked_get.side_effect = requests.exceptions.ConnectionError()
        with self.assertRaises(requests.exceptions.ConnectionError) as context:
            client.get_textures("fakeToken", "fakeContextId")
        self.assertEqual(requests.exceptions.ConnectionError, context.expected)

    def test_filter_textures(self):
        textures = [{"name": "test.png"}, {"name": "test2.JPG"}, {
            "name": "test3.jPeG"}, {"name": "test4.gif"}, {"name": "test5.txt"}]
        client = StorageClient.StorageClient()
        result = client.filter_textures(textures)
        self.assertEqual(result, [{"name": "test.png"}, {"name": "test2.JPG"}, {
            "name": "test3.jPeG"}, {"name": "test4.gif"}])

    # Resources
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.list_files')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.get_file')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.copy_file_content')
    def test_copy_folder_content_to_tmp(self, mocked_copy_file_content, mocked_get, mocked_list_files):
        client = StorageClient.StorageClient()
        client._sim_dir = '/tmp'

        resources_item = {
            "uuid": "89857775-6215-4d53-94ee-fb6c18b9e2f8",
            "name": "resources",
            "parent": "89857775-6215-4d53-94ee-fb6c18b9e2g8",
            "type": "folder"
        }
        mocked_list_files.return_value = [
            {
                "uuid": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
                "name": "fakeFileName",
                "parent": "89857775-6215-4d53-94ee-fb6c18b9e2f8",
                "type": "file"
            }
        ]

        mocked_get.return_value = 'test'
        client.copy_folder_content_to_tmp("fakeToken", resources_item)

        self.assertTrue(client.list_files.called)
        mocked_copy_file_content.assert_called_once_with('fakeToken', '/tmp/resources', '89857775-6215-4d53-94ee-fb6c18b9e2f8', 'fakeFileName', is_fileobject=False)

    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.list_files')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.copy_folder_content_to_tmp')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.get_file')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.SimUtil')
    def test_copy_resources_folder(self, mocked_sim_util, mocked_get, mocked_copy_folder_content_to_tmp, mocked_list_files):
        client = StorageClient.StorageClient()
        client._StorageClient__resources_path = '/tmp/some/path'
        mocked_list_files.return_value = [{
            "uuid": "3ce08569-bdb7-49ee-a751-5640f4b8745646",
            "name": "resources",
            "parent": "89857775-6215-4d53-94ee-fb6c18b9e2f8",
            "type": "folder"
        }]

        client.copy_resources_folder("fakeToken", "fakeExperiment")
        mocked_copy_folder_content_to_tmp.assert_called_once()

        mocked_sim_util.clear_dir.return_value = ''
        self.assertTrue(client.copy_folder_content_to_tmp.called)

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
    def test_list_files_connection_error(self, mocked_post):
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

    def test_parse_and_check_fails(self):
        # invalid exc file
        experiment_path = os.path.join(self.experiments_directory, "experiment_configuration.exc")
        self.assertRaises(NRPServicesGeneralException,
                          StorageClient.StorageClient.parse_and_check_file_is_valid,
                          experiment_path,
                          lambda _: None,
                          exp_conf_api_gen.ExD_)

    def test_parse_and_check_throws(self):
        # invalid exc file
        experiment_path = os.path.join(self.experiments_directory, "experiment_configuration.exc")
        self.assertRaises(NRPServicesGeneralException,
                          StorageClient.StorageClient.parse_and_check_file_is_valid,
                          experiment_path,
                          lambda _: Exception(),
                          exp_conf_api_gen.ExD_)

    # CLONE ALL EXPERIMENT FILES
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.list_files')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.get_file')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.generate_textures')
    def test_clone_all_experiment_files(self, mocked_gen_texture, mocked_get, mocked_list):
        mocked_get.side_effect = None

        experiment_name = "fakeExperiment"

        env_editor_name = "env_editor.autosaved"
        exp_conf_name = 'experiment_configuration.exc'
        simple_robot_name = 'simple_move_robot.py'

        tfs_dir_name = 'transfer_functions'
        tfs_dir_uuid = "6a63d03e-6dad-4793-80d7-8e32a83aaa66"

        mock_experiment_dir_list = [
            {
                "uuid": "07b35b8f-67cd-4e94-8bec-5ede8049590d",
                "name": env_editor_name,
                "parent": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
                "contentType": "text/plain",
                "type": "file",
                "modifiedOn": "2017-08-31T13:56:34.306090Z"
            },
            {
                "uuid": "6a63d03e-6dad-4793-80d7-8e32a83eee78",
                "name": exp_conf_name,
                "parent": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
                "contentType": "text/plain",
                "type": "file",
                "modifiedOn": "2017-08-30T11:23:47.842214Z"
            },
            {
                "uuid": tfs_dir_uuid,
                "name": tfs_dir_name,
                "parent": "3ce08569-bdb7-49ee-a751-5640f4b879d4",
                "contentType": "application/hbp-neurorobotics.tfs+python",
                "type": "folder",
                "modifiedOn": "2017-08-30T12:32:47.842214Z"
            }
        ]

        # transfer_functions/simple_move_robot.py
        mock_tfs_dir_list = [{
            "uuid": "6a63d03e-6dad-4793-80d7-8e32a83ddd14",
            "name": simple_robot_name,
            "parent": "6a63d03e-6dad-4793-80d7-8e32a83aaa66",
            "contentType": "application/hbp-neurorobotics.tfs+python",
            "type": "file",
            "modifiedOn": "2017-08-30T12:32:47.842214Z"
        }
        ]

        def mocked_list_fun(_token, experiment, folder):
            if experiment == experiment_name:
                return mock_experiment_dir_list
            elif experiment == tfs_dir_uuid:
                return mock_tfs_dir_list
            return []

        mocked_list.side_effect = mocked_list_fun

        client = StorageClient.StorageClient()

        with patch("__builtin__.open", mock_open(read_data="data")) as mocked_open, \
                patch('hbp_nrp_backend.storage_client_api.StorageClient.SimUtil') as mocked_sim_util:
            sim_dir = '/some/path/over/the/rainbow'
            client.clone_all_experiment_files("fakeToken", experiment_name, destination_dir=sim_dir)

            mocked_open.assert_any_call(os.path.join(sim_dir, env_editor_name), 'w')
            mocked_open.assert_any_call(os.path.join(sim_dir, exp_conf_name), 'w')
            mocked_open.assert_any_call(os.path.join(sim_dir, tfs_dir_name, simple_robot_name), 'w')

    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.list_experiments')
    def test_get_folder_uuid_by_name_ok(self, mocked_get):
        mocked_get.return_value = [{"name": 'Experiment_0', "uuid": "Experiment_0_uuid"}, {
            "name": 'Experiment_1', "uuid": "Experiment_1_uuid"}]
        client = StorageClient.StorageClient()
        uuid = client.get_folder_uuid_by_name(
            'fakeToken', 'fake_context_id', 'Experiment_0')
        self.assertEqual(uuid, 'Experiment_0_uuid')
        mocked_get.assert_called_with(
            'fakeToken', 'fake_context_id', name='Experiment_0', get_all=True)

    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.copy_file_content')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.create_material_from_textures')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.create_textures_paths')
    @patch('hbp_nrp_backend.storage_client_api.StorageClient.StorageClient.get_textures')
    def test_generate_textures_ok(self, mocked_get_textures, mock_create_textures, mock_create_material_from_textures, mock_copy_file_content):
        mocked_get_textures.return_value = [
            {"name": 'test1.png'}, {"name": "test2.gif"}]
        client = StorageClient.StorageClient()
        client._StorageClient__texture_directories = ['/somewhere/near/the/rainbow']
        client.generate_textures('fakeExperiment', 'fake_token')
        mocked_get_textures.assert_called_with('fakeExperiment', 'fake_token')
        mock_create_textures.assert_called()
        mock_create_material_from_textures.assert_called()
        mock_copy_file_content.assert_called_with(
            'fake_token', '/somewhere/near/the/rainbow/materials/textures', 'fakeExperiment%2Fresources%2Ftextures', 'test2.gif', is_fileobject=True)

    def test_create_material_from_textures_ok(self):
        with patch("__builtin__.open", mock_open(read_data="data")) as mocked_open:
            client = StorageClient.StorageClient()
            client._StorageClient__texture_directories = ['/somewhere/near/the/rainbow']
            client.create_material_from_textures([{"name": 'texture1.png'}, {"name": 'texture2.jpeg'}])
            mocked_open.assert_called_with('/somewhere/near/the/rainbow/materials/scripts/custom.material', 'w')

    def test_check_file_extension(self):
        example1 = [{u'uiid': u'/test_folder/experiment_configuration.exc', u'name': u'experiment_configuration.exc'},
                    {u'uiid': u'/test_folder/.experiment_configuration.exc.swp', u'name': u'.experiment_configuration.exc.swp'}]
        client = StorageClient.StorageClient()
        self.assertTrue(client.check_file_extension(
            example1[1]['name'], ['.swp']))
        self.assertFalse(client.check_file_extension(
            example1[1]['name'], ['.txt']))
        self.assertFalse(client.check_file_extension(
            example1[0]['name'], ['.swp']))
        self.assertTrue(client.check_file_extension(
            example1[0]['name'], ['.exc']))


if __name__ == '__main__':
    unittest.main()
