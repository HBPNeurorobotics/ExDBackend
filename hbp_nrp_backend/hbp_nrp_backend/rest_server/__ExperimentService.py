"""
This module contains the REST implementation
for retrieving Experiment names and IDs, as well as some functions used by other
classes that retrieve experiment related files.
"""

__author__ = "Bernd Eckstein"

import os
import base64
import logging

from flask_restful import Resource, fields
from flask_restful_swagger import swagger
from hbp_nrp_backend.rest_server import NRPServicesClientErrorException, NRPServicesGeneralException
from hbp_nrp_backend.exd_config.generated import generated_experiment_api

# pylint: disable=R0201
# pylint: disable=E1103

LOG = logging.getLogger(__name__)


class ErrorMessages(object):
    """
    Definition of error strings
    """
    EXPERIMENT_NOT_FOUND_404 = "The experiment with the given ID was not found"
    EXPERIMENT_PREVIEW_NOT_FOUND_404 = "The experiment has no preview image"
    EXPERIMENT_FILE_NOT_FOUND_404 = "The experiment file was not found"
    VARIABLE_ERROR = "Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty"
    ERROR_SAVING_FILE_500 = "Error saving file"
    ERROR_IN_BASE64_400 = "Error in base64: {0}"


@swagger.model
class ExperimentData(object):
    """
    Swagger documentation object
    ExperimentData definition
    """
    resource_fields = {
        "name": fields.String(),
        "experimentConfiguration": fields.String(),
        "description": fields.String(),
        "timeout": fields.Integer(),
    }

    required = ['name', 'experimentConfiguration', 'description', 'timeout']


@swagger.model
@swagger.nested(exp_id_1=ExperimentData.__name__,
                exp_id_2=ExperimentData.__name__,
                exp_id_n=ExperimentData.__name__)
class ExperimentDictionary(object):
    """
    Swagger documentation object
    ExperimentDictionary ... tried to make it look like a dictionary for the swagger doc
    """
    resource_fields = {
        'exp_id_1': fields.Nested(ExperimentData.resource_fields),
        'exp_id_2': fields.Nested(ExperimentData.resource_fields),
        'exp_id_n': fields.Nested(ExperimentData.resource_fields)
    }


@swagger.model
@swagger.nested(data=ExperimentDictionary.__name__)
class Data(object):
    """
    Swagger documentation object
    Main Data Attribute for parsing convenience on the frontend side.
    """
    resource_fields = {
        'data': fields.Nested(ExperimentDictionary.resource_fields)
    }
    required = ['data']


class Experiment(Resource):
    """
    Implements the REST service for retrieving all stored experiments
    as a dictionary
    """

    @swagger.operation(
        notes="Gets dictionary of experiments",
        responseClass=Data.__name__,
        responseMessages=[
            {
                "code": 500,
                "message": "Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty"
            },
            {
                "code": 200,
                "message": "Success. The experiment list was sent."
            }
        ]
    )
    def get(self):
        """
        Gets dictionary of experiments stored on the server.

        :return: Dictionary with experiments
        :status 500: Error on server: environment variable: 'NRP_MODELS_DIRECTORY' is empty
        :status 200: Success. The dictionary was returned
        """

        experiments = dict(data=get_experiments())
        return experiments, 200


def get_experiments():
    """
    Read all available Experiments from disk, and add to dictionary
    :return dictionary: with string: ID, string: ExDConfig File
    """

    experiment_dir = "ExDConf"
    path = os.path.join(get_basepath(), experiment_dir)
    experiment_names = os.listdir(path)

    experiment_dict = {}

    for current in experiment_names:
        if current.lower().endswith('.xml'):
            # Parse Experiment
            experiment_conf = os.path.join(get_basepath(), experiment_dir, current)
            ex = generated_experiment_api.parse(experiment_conf, silence=True)
            if isinstance(ex, generated_experiment_api.ExD):
                current_exp = _make_experiment(current, ex, experiment_dir)
                experiment_dict[os.path.splitext(current)[0]] = current_exp

    return experiment_dict


def _make_experiment(experiment_file, experiment, experiment_dir):
    """
    Creates and returns an dictionary with the experiment data
    :param experiment_file: filename of the XML containing the experiment
    :param experiment: the parsed experiment
    :param experiment_dir: the directory containing the XML file of the experiment
    :return dictionary with the experiment data
    """

    _timeout = experiment.get_timeout()
    _name = experiment.get_name()
    _description = experiment.get_description()

    if _timeout is None:
        _timeout = 600
    if _name is None:
        _name = os.path.splitext(experiment_file)[0]
    if _description is None:
        _description = "No description available for this experiment."

    current_exp = dict(name=_name,
                       description=_description,
                       experimentConfiguration=os.path.join(experiment_dir, experiment_file),
                       timeout=_timeout)
    return current_exp


def save_file(base64_data, filename_abs):
    """
    Save a file, encoded as base64_data to specified filename_abs
    :param base64_data:  base64 encoded data
    :param filename_abs: the absolute filename
    :return true, when file is saved, false otherwise
    :exception NRPServicesClientErrorException, errorNo: 401: Error in base64
    """

    allowed_path = get_basepath()

    try:
        data = base64.decodestring(base64_data)
    except Exception as _ex:
        raise NRPServicesClientErrorException(ErrorMessages.ERROR_IN_BASE64_400.format(
            _ex.message), 400)

    base_path = os.path.abspath(filename_abs)

    print "Allowed: " + allowed_path
    print "Base: " + base_path

    if base_path.startswith(allowed_path):
        pass
    else:
        LOG.error("Path not accepted: '{0}' is not a subdirectory of allowed path: '{1}'."
                  .format(base_path, allowed_path))
        raise NRPServicesGeneralException(ErrorMessages.ERROR_SAVING_FILE_500,
                                          "Server Error: {0}, {1}".format(base_path,
                                                                          allowed_path), 500)

    filename = "{0}_new".format(filename_abs)
    with open(filename, 'wb') as _file:
        _file.write(data)


def get_basepath():
    """
    :return: path given in the environment variable 'NRP_MODELS_DIRECTORY'
    """

    path = os.environ.get('NRP_MODELS_DIRECTORY')
    if path is None:
        raise NRPServicesGeneralException(ErrorMessages.VARIABLE_ERROR, "Server Error", 500)

    return path


def get_experiment_rel(exp_id):
    """
    :return: relative name of the experiment xml
    """

    experiment_dict = get_experiments()
    if exp_id not in experiment_dict:
        raise NRPServicesClientErrorException(ErrorMessages.EXPERIMENT_NOT_FOUND_404, 404)

    # Get Experiments relative filename
    experiment_file = experiment_dict[exp_id]['experimentConfiguration']
    return experiment_file


def get_experiment_conf(exp_id):
    """
    :return Absolute path to experiment xml file
    """

    experiment_file = get_experiment_rel(exp_id)
    experiment_conf = os.path.join(get_basepath(), experiment_file)
    return experiment_conf


def get_bibi_file(exp_id):
    """
    return Absolute path to experiment bibi file
    """
    experiment_file = get_experiment_conf(exp_id)
    experiment = generated_experiment_api.parse(experiment_file, silence=True)
    assert isinstance(experiment, generated_experiment_api.ExD)
    bibi_file = experiment.get_bibiConf()
    bibi_conf = os.path.join(get_basepath(), bibi_file)
    return bibi_conf