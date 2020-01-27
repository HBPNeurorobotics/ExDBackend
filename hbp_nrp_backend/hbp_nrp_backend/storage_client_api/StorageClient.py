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
Wrapper around the storage API
"""

import os
import logging
import requests
import shutil
import urllib
import re
import textwrap
import tempfile
from pyxb import ValidationError
from xml.sax import SAXParseException
from hbp_nrp_backend import NRPServicesGeneralException
from hbp_nrp_commons.sim_config.SimConfig import ResourceType
from hbp_nrp_commons.workspace.Settings import Settings
from hbp_nrp_commons.workspace.SimUtil import SimUtil

__author__ = "Manos Angelidis"
logger = logging.getLogger(__name__)


class Model(object):  # pragma: no cover
    """
    This class represents the model that we have in the model DB.
    """

    def __init__(self, nameModel, typeModel, pathModel=None):
        """
        Creates the model object.
        """
        self.name = nameModel
        self.type = typeModel
        self.path = pathModel


class ModelType(object):  # pragma: no cover
    """
    Enumeration for model types. Model resources are enhanced in a different structure
    """
    types = {
        ResourceType.ROBOT: 'robots',
        ResourceType.ENVIRONMENT: 'environments',
        ResourceType.BRAIN: 'brains'
    }

    @staticmethod
    def getResourceType(value):
        """
        returns the key of the valueToFind form the types dictionary.
        """
        return [key for key, value in ModelType.types.iteritems() if value == value][0]


class StorageClient(object):
    """
    Wrapper around the storage server API. Users of this class should first
    call the authentication function to retrieve a token, before making the
    requests to the storage server
    """

    __instance = None

    def __new__(cls):
        """
        Overridden new for the singleton implementation
        :return: Singleton instance
        """

        if StorageClient.__instance is None:
            StorageClient.__instance = object.__new__(cls)

            # pylint: disable=protected-access
            StorageClient.__instance._sim_dir = None
            # adding the resources folder into the created sim dir
            StorageClient.__instance.__resources_path = None

        return StorageClient.__instance

    def __init__(self):
        """
        Creates the storage client
        """
        self.__proxy_url = Settings.storage_uri

        # Paths related to the textures
        self.__texture_directories = (Settings.local_gazebo_path
                                      + Settings.gzweb_assets_media_path
                                      + Settings.gzweb_custom_textures_path)

        # folders in resources we want to filter
        self.__filtered_resources = ['textures']

    def set_sim_dir(self, sim_dir):
        """
        Sets the sim_dir for this client

        :param sim_dir: Simulation directory
        """

        self._sim_dir = sim_dir

    def get_user(self, token):
        """
        Retrieves the user id for the specified authentication token
        :param token: the authentication token
        :return: the user id
        """

        try:
            res = requests.get(self.__proxy_url + '/identity/me',
                               headers={'Authorization': 'Bearer ' + token})
            if 200 <= res.status_code < 300:
                return res.json()
            else:
                raise Exception(
                    'Could not verify auth token, status code ' +
                    str(res.status_code))
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def can_acess_experiment(self, token, context_id, experiment_id):
        """
        Verifies if an authenticated user can access an experiment
         :param token: The authentication token
        :param context_id: Optional context idenfifier
        :param experiment_id: The  experiment id
        :return: Whether the user can access the experiment
        """

        return self.list_experiments(token, context_id, name=experiment_id) is not None

    def list_experiments(self, token, context_id, get_all=False, name=None):
        """
        Lists the experiments the user has access to depending on his token
        :param token: a valid token to be used for the request
        :param context_id: the context_id if we are using collab storage
        :param get_all: a parameter to return all the available experiments,
                        not only the ones available to a specific user
        :param name: if we want to get a specific folder i.e. the robots
        :return: an array of all the available to the user experiments
        """
        query_args = {}
        if name:
            query_args['filter'] = urllib.quote_plus(name)
        if get_all:
            query_args['all'] = str(get_all).lower()

        try:
            res = requests.get(
                '{proxy_url}/storage/experiments?{params}'.format(
                    proxy_url=self.__proxy_url,
                    get_all=str(get_all).lower(),
                    params=urllib.urlencode(query_args)),
                headers={'Authorization': 'Bearer ' + token, 'context-id': context_id}
            )

            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code ' +
                    str(res.status_code))
            else:
                return res.json()
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def get_file(self, token, experiment, filename, by_name=False, zipped=False, is_fileobject=False):
        """
        Gets a file under an experiment based on the filename and on the filetype
        Depending on the file type we either:
            return the .content of the response if it is a zip
            return the .raw of the response if it is an image
            return the .text if it is a text file
        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :param filename: the name of the file to return
        :param zipped: flag denoting that "filename" is a zip file
        :param is_fileobject: flag denoting that "filename" is an object file
        :return: if successful, the content of the file
        """
        try:
            request_url = '{proxy_url}/storage/{experiment}/{filename}?byname={by_name}'.format(
                proxy_url=self.__proxy_url,
                experiment=experiment,
                filename=filename,
                by_name=str(by_name).lower()
            )

            res = requests.get(request_url,
                               headers={'Authorization': 'Bearer ' + token},
                               stream=is_fileobject)

            # TODO what about missing files? i.e. 204
            if res.status_code < 200 or res.status_code >= 300:
                raise Exception('Failed to communicate with the storage server, status code {}'
                                .format(res.status_code))
            if zipped:
                return res.content

            if is_fileobject:
                res.raw.decode_content = True
                return res.raw

            return res.text
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def delete_file(self, token, experiment, filename):
        """
        Deletes a file under under an experiment based on the
        experiment name and the filename. Needs the user token
        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :param filename: the name of the file to delete
        :return: if successful, the name of the deleted file
        """
        try:
            request_url = '{proxy_url}/storage/{path}'.format(
                proxy_url=self.__proxy_url,
                path=urllib.quote(os.path.join(experiment, filename), safe='')
            )

            res = requests.delete(request_url, headers={'Authorization': 'Bearer ' + token})

            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code ' +
                    str(res.status_code))
            else:
                return "Success"
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def create_or_update(self, token, experiment, filename, content, content_type, append=False):
        """
        Creates or updates a file under an experiment

        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :param filename: the name of the file to update/create
        :param content: the content of the file
        :param content_type: the content type of the file i.e. text/plain or
                             application/octet-stream
        :param append: append to file or create new file
        """
        try:
            append_query = "?append=true" if append else ""
            request_url = '{proxy_url}/storage/{experiment}/{filename}{appendquery}'.format(
                proxy_url=self.__proxy_url,
                experiment=experiment,
                filename=filename,
                appendquery=append_query)

            res = requests.post(request_url,
                                headers={'content-type': content_type,
                                         'Authorization': 'Bearer ' + token},
                                data=content)

            if res.status_code < 200 or res.status_code >= 300:
                raise Exception('Failed to communicate with the storage server, status code ' +
                                str(res.status_code))
            else:
                return res.status_code
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def create_folder(self, token, experiment, name):
        """
        Creates a folder under an experiment. If the folder exists we reuse it
        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :param name: the name of the folder to create
        """
        try:
            request_url = '{proxy_url}/storage/{experiment}/{name}?type=folder'.format(
                proxy_url=self.__proxy_url,
                experiment=experiment,
                name=name
            )

            res = requests.post(request_url, headers={'Authorization': 'Bearer ' + token})

            if res.status_code == 400:
                logger.info('The folder with the name {0} already exists in the storage,reusing'
                            .format(name))
                return 200

            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code ' +
                    str(res.status_code))
            else:
                return res.json()
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def list_files(self, token, experiment, folder=None):
        """
        Lists all the files under an experiment based on the
        experiment name and the user token
        :param token: a valid token to be used for the request
        :param experiment: the name of the experiment
        :param folder: it is a boolean variable that indicates if list_files returns folders or not.

        :return: if successful, the files under the experiment
        """
        try:

            request_url = '{proxy_url}/storage/{experiment}'.format(
                proxy_url=self.__proxy_url,
                experiment=experiment
            )

            res = requests.get(request_url, headers={'Authorization': 'Bearer ' + token})

            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code ' +
                    str(res.status_code))
            else:
                # folder variable is added because copy_resources_folder needs
                # to list the folders too
                return [entry for entry in res.json()
                        if (entry['type'] == 'file' or folder) and
                        not self.check_file_extension(entry['name'], ['.swp'])]

        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def get_model_path(self, token, context_id, model):
        """
            Returns a custom model provided its path
            :param token: a valid token to be used for the request
            :param context_id: the context_id of the collab
            :param model: model object, check class Model
            :return: if found, the uuid of the named folder
        """
        try:
            request_url = '{proxy_url}/storage/models/path/{model_type}/{model_name}'.format(
                proxy_url=self.__proxy_url,
                model_type=ModelType.types[model.type],
                model_name=model.name
            )
            res = requests.get(request_url,
                               headers={'Authorization': 'Bearer ' + token,
                                        'context-id': context_id})

            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code {}'
                    .format(res.status_code))

            else:
                return res.content
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def get_model(self, token, context_id, model):
        """
        Returns a custom model provided its path
        :param token: a valid token to be used for the request
        :param context_id: the context_id of the collab
        :param model: the model object, check class Model
        :return: if found, returns the content of the model
        """
        try:
            request_url = '{proxy_url}/storage/models/{model_type}/{model_name}'.format(
                proxy_url=self.__proxy_url,
                model_type=ModelType.types[model.type],
                model_name=model.name
            )
            res = requests.get(request_url,
                               headers={'Authorization': 'Bearer ' + token,
                                        'context-id': context_id})

            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code {}'
                    .format(res.status_code))

            else:
                return res.content
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def get_models(self, token, context_id, model_type):
        """
        Returns the contents of a custom models folder provided its name
        :param token: a valid token to be used for the request
        :param context_id: the context_id of the collab
        :param model_type: the type of the model defined in ModelType
        :return: if found, list of Models objects
        """
        try:
            request_url = '{proxy_url}/storage/models/all/{modelType}'.format(
                proxy_url=self.__proxy_url,
                modelType=ModelType.types[model_type]
            )
            res = requests.get(request_url,
                               headers={'Authorization': 'Bearer ' + token,
                                        'context-id': context_id})
            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code ' +
                    str(res.status_code))
            else:
                list_models = []
                for model in res.json():
                    model_type_key = ModelType.getResourceType(model['type'])
                    model = Model(model['name'], model_type_key, model['path'])
                    list_models.append(model)
                return list_models

        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    def get_textures(self, experiment, token):
        """
        Returns the contents of the resources/textures experiment folder
        :param experiment: the name of the experiment
        :param token: a valid token to be used for the request
        :return: if found, the list of textures
        """
        try:
            request_url = '{proxy_url}/storage/{experiment}{textures_path}'.format(
                proxy_url=self.__proxy_url,
                experiment=experiment,
                textures_path=urllib.quote_plus('/resources/textures')
            )

            res = requests.get(request_url, headers={'Authorization': 'Bearer ' + token})

            if res.status_code < 200 or res.status_code >= 300:
                raise Exception(
                    'Failed to communicate with the storage server, status code ' +
                    str(res.status_code))
            else:
                return res.json()
        except requests.exceptions.ConnectionError, err:
            logger.exception(err)
            raise err

    # HELPER FUNCTIONS
    def clone_file(self, filename, token, experiment):
        """
        Clones a file according to a given filename to a simulation folder.
        The caller then has the responsibility of managing this folder.

        :param filename: The filename of the file to clone
        :param token: The token of the request
        :param experiment: The experiment which contains the file
        :return: The local path of the cloned file,
        """
        for folder_entry in self.list_files(token, experiment):
            if filename in folder_entry['name']:
                clone_destination = os.path.join(self._sim_dir, filename)
                with open(clone_destination, "w") as f:
                    f.write(self.get_file(token, experiment, filename, by_name=True))
                break
        else:
            return None  # filename not found

        return clone_destination

    def copy_file_content(self, token, src_folder, dest_folder, filename, is_fileobject=False):
        """
        copy the content of file located in the Storage into the proper tmp folder

        :param token: The token of the request
        :param src_folder: folder location where it will be copy from
        :param dest_folder: folder location where it will be copy to
        :param filename: name of the file to be copied.
        :param is_fileobject: flag to signal an object file.
        """
        _, ext = os.path.splitext(filename)
        # if the file is a texture we have to copy a file object, not a string
        if is_fileobject:
            with open(os.path.join(src_folder, filename), "wb") as texture_file:
                shutil.copyfileobj(
                    self.get_file(
                        token,
                        dest_folder, filename,
                        by_name=True, zipped=False, is_fileobject=is_fileobject),
                    texture_file
                )
        else:
            with open(os.path.join(src_folder, filename), "w") as f:
                f.write(
                    self.get_file(
                        token,
                        dest_folder, filename,
                        by_name=True, zipped=(ext.lower() == ".zip"), is_fileobject=is_fileobject
                    )
                )

    # pylint: disable=no-self-use
    def check_file_extension(self, filename, extensions):
        """
        checks if an file is of a certain extension

        :param filename: the file name
        :param extensions: the extensions list to check
        """
        return os.path.splitext(filename)[1].lower() in extensions

    def copy_folder_content_to_tmp(self, token, folder):
        """
        copy the content of the folder located in storage/experiment into sim_dir folder

        :param token: The token of the request
        :param folder: the folder in the storage folder to copy in tmp folder,
                       it has included the uuid of the experiment
        """
        folder['fullpath'] = folder['name']
        child_folders = [folder]
        folder_path = ''
        while child_folders:
            actual_folder = child_folders.pop()
            folder_path = actual_folder['fullpath']
            folder_uuid = urllib.quote_plus(actual_folder['uuid'])
            for folder_entry in self.list_files(token, folder_uuid, True):
                if folder_entry['type'] == 'folder' and folder_entry['name'] \
                        not in self.__filtered_resources:
                    folder_entry['fullpath'] = os.path.join(
                        folder_path, folder_entry['name'])
                    child_folders.append(folder_entry)
                if folder_entry['type'] == 'file':
                    folder_tmp_path = str(os.path.join(self._sim_dir, folder_path))
                    SimUtil.makedirs(folder_tmp_path)
                    is_fileobject = os.path.splitext(folder_entry['name'])[1].lower() == '.h5'
                    self.copy_file_content(
                        token, folder_tmp_path, folder_uuid, folder_entry['name'], is_fileobject=is_fileobject)

    def copy_resources_folder(self, token, experiment):
        """
        Copy the resources folder located in storage/experiment into simulation folder

        :param token: The token of the request
        :param experiment: The experiment which contains the resource folder
        """
        try:
            for folder_entry in self.list_files(token, experiment, True):
                if folder_entry['name'] == 'resources' and folder_entry['type'] == 'folder':
                    if os.path.exists(self.__resources_path):
                        SimUtil.clear_dir(self.__resources_path)
                    self.copy_folder_content_to_tmp(token, folder_entry)
        except Exception:
            logger.exception('An error happened trying to copy resources to tmp ')
            raise

    def create_material_from_textures(self, textures):
        """
        Algorithm to create a new material from the textures. For every texture
        we append a new default material which points to the texture.
        """
        # The material script is a stripped down version of an OGRE material script
        # see http://wiki.ogre3d.org/Materials for the documentation
        material_template = textwrap.dedent(
            '''
            material {texture_name}
            {{
                technique
                {{
                    pass
                    {{
                        ambient 1 1 1 1.000000
                        diffuse 1 1 1 1.000000
                        specular 0.03 0.03 0.03 1.000000
                        texture_unit
                        {{
                        texture {texture_name}
                        }}
                    }}
                }}
            }}
            ''')

        material_scripts = [
            material_template.format(texture_name=texture['name'])
            for texture in textures
        ]

        # Create a custom.material script and pass it to all the required directories
        for directory in self.__texture_directories:
            with open(os.path.join(directory, 'materials', 'scripts', 'custom.material'), 'w') as f:
                f.write(''.join(material_scripts))

    @staticmethod
    def filter_textures(textures):
        """
        Returns only the textures from /resources/textures
        """
        # Regexp checks if the texture is a png, gif, or jpeg
        return [t for t in textures if re.search(r"\.(jpe?g|png|gif)$", t["name"],
                                                 re.IGNORECASE) is not None]

    def create_textures_paths(self):
        """
        Generate the folder structure under assets. Looks like:
        relative_path/materials/scripts/custom.material
        relative_path/materials/textures/texture1...n
        """
        relative_path = ''
        for directory in self.__texture_directories:
            for path in ['materials']:
                SimUtil.makedirs(os.path.join(directory, relative_path))
                relative_path = os.path.join(relative_path, path)
            SimUtil.makedirs(os.path.join(directory, relative_path, 'scripts'))
            SimUtil.makedirs(os.path.join(directory, relative_path, 'textures'))
            relative_path = ''

    def generate_textures(self, experiment, token):
        """
        Clones all the contents of the textures folders to the temporary directory and
        creates the structure required by gazebo and gzweb.

        :param token: The token of the request
        :param experiment: The experiment which contains the textures folder
        """
        # Get the contents of the resources/textures folder from the proxy and filter them
        # to get only the images
        textures = self.filter_textures(self.get_textures(experiment, token))

        # If we find images we proceed to create paths in the backend
        if textures:

            # Create the paths in ~/.local/share/gazebo-7/media
            # $NRP/gzweb/http/client/assets/media | custom_textures
            self.create_textures_paths()

            # Generate a custom material file which points to all the textures
            self.create_material_from_textures(textures)

            # For the backend directories and for every texture copy over the textures
            for directory in self.__texture_directories:
                for texture in textures:
                    self.copy_file_content(token,
                                           os.path.join(directory, 'materials', 'textures'),
                                           urllib.quote_plus(experiment + '/resources/textures'),
                                           texture['name'],
                                           is_fileobject=True)

    # pylint: disable=broad-except, dangerous-default-value
    def clone_all_experiment_files(self, token, experiment, destination_dir=None, exclude=[]):
        """
        Clones all the experiment files to a simulation folder.
        The caller has then the responsibility of managing this folder.

        :param token: The token of the request
        :param experiment: The experiment to clone
        :param destination_dir: the directory in which to clone the files,
            if None is provided, clones in a temporary folder
        :param exclude: a list of folders of files not to clone (folder names ends with '/')
        :return: A dictionary containing the paths to the experiment files
        """

        # pylint: disable=too-many-locals
        # if something goes wrong while generating textures we just log the error
        # and continue like nothing happened
        try:
            self.generate_textures(experiment, token)
        except Exception as e:
            logger.info("Could not generate textures, error occurred : %s", (str(e)))

        destination_dir = destination_dir if destination_dir else tempfile.mkdtemp(prefix='nrp.')
        self._sim_dir = destination_dir
        self.__resources_path = os.path.join(self._sim_dir, "resources")

        exclude_files = [f for f in exclude if not f.endswith('/')]
        exclude_dirs = [os.path.dirname(d) for d in exclude if d.endswith('/')]

        for entry_to_clone in self.list_files(token, experiment, folder=True):
            # Filter out excluded folders and files
            if entry_to_clone['type'] == 'folder':
                if entry_to_clone['name'] in exclude_dirs:
                    continue
            else:
                if (entry_to_clone['name'] in exclude_files or
                        os.path.dirname(entry_to_clone['name']) in exclude_dirs):
                    continue

            if entry_to_clone['type'] == 'folder':
                self.copy_folder_content_to_tmp(token, entry_to_clone)
            else:  # == 'file'
                dest_file_path = os.path.join(destination_dir, entry_to_clone['name'])
                with open(dest_file_path, "w") as file_clone:

                    zipped = os.path.splitext(entry_to_clone['name'])[1].lower() == '.zip'
                    is_fileobject = os.path.splitext(entry_to_clone['name'])[1].lower() == '.h5'
                    file_contents = self.get_file(token, experiment, entry_to_clone['name'],
                                                  by_name=True, zipped=zipped, is_fileobject=is_fileobject)

                    file_clone.write(file_contents)

        return destination_dir

    @staticmethod
    def parse_and_check_file_is_valid(filepath, create_obj_function, instance_type):
        """
        Parses a file and checks if it corresponds to its instance type and
        can be created into its object

        :param filepath: The path of the file
        :param create_obj_function: The function to create the object
        :param instance_type: The required instance type of the file
        :return: An object containing the file content
        """
        with open(filepath) as file_content:
            try:
                file_obj = create_obj_function(file_content.read())
                if not isinstance(file_obj, instance_type):
                    raise NRPServicesGeneralException(
                        "{filepath} configuration file content is not valid."
                        .format(filepath=filepath), "File not valid")
            except (ValidationError, SAXParseException) as ve:
                raise NRPServicesGeneralException(
                    "Could not parse file {filepath} due to validation error: {validation_error}"
                    .format(filepath=filepath, validation_error=str(ve)), "File not valid")
        return file_obj

    def get_folder_uuid_by_name(self, token, context_id, folder_name):
        """
        Returns the uuid of a folder provided its name
        :param token: a valid token to be used for the request
        :param context_id: the context_id of the collab
        :param folder_name: the name of the folder
        :return: if found, the uuid of the named folder
        """
        folders = self.list_experiments(token, context_id, get_all=True, name=folder_name)

        for folder in (f for f in folders if f["name"] == folder_name):
            return folder["uuid"]
