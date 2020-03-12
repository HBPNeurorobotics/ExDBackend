# pylint: disable=too-many-lines
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
ROS wrapper around the CLE
"""

__author__ = "Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli, Georg Hinkel, Hossain Mahmud"

import json
import logging
import rospy
import numpy
import time
import sys
from std_srvs.srv import Empty
import textwrap
import re
import struct

from ._ExcBibiHandler import ExcBibiHandler
from ._RobotCallHandler import RobotCallHandler

from RestrictedPython.RCompile import RModule, RestrictionMutator

# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackages folder at the root of this CLE repository.
from hbp_nrp_cleserver.server.SimulationServer import SimulationServer
from cle_ros_msgs import srv
from cle_ros_msgs.msg import CLEError, ExperimentPopulationInfo
from cle_ros_msgs.msg import PopulationInfo, NeuronParameter, CSVRecordedFile
from . import SERVICE_GET_TRANSFER_FUNCTIONS, SERVICE_EDIT_TRANSFER_FUNCTION, \
    SERVICE_ADD_TRANSFER_FUNCTION, SERVICE_DELETE_TRANSFER_FUNCTION, SERVICE_GET_BRAIN, \
    SERVICE_SET_BRAIN, SERVICE_GET_POPULATIONS, SERVICE_SET_POPULATIONS, \
    SERVICE_GET_CSV_RECORDERS_FILES, SERVICE_CLEAN_CSV_RECORDERS_FILES, \
    SERVICE_ACTIVATE_TRANSFER_FUNCTION, SERVICE_CONVERT_TRANSFER_FUNCTION_RAW_TO_STRUCTURED, \
    SERVICE_ADD_ROBOT, SERVICE_GET_ROBOTS, SERVICE_DEL_ROBOT, SERVICE_SET_EXC_ROBOT_POSE, \
    SERVICE_PREPARE_CUSTOM_MODEL
from . import ros_handler
import hbp_nrp_cleserver.bibi_config.StructuredTransferFunction as StructuredTransferFunction
import hbp_nrp_cle.tf_framework as tf_framework
from hbp_nrp_cle.tf_framework import TFLoadingException, BrainParameterException
import base64
from tempfile import NamedTemporaryFile
from hbp_nrp_cleserver.bibi_config.notificator import Notificator, NotificatorHandler
from hbp_nrp_cleserver.server.SimulationServerLifecycle import SimulationServerLifecycle
from hbp_nrp_commons.bibi_functions import find_changed_strings
from hbp_nrp_cleserver.server.CSVLogger import CSVLogger

logger = logging.getLogger(__name__)

# We use the logger hbp_nrp_cle.user_notifications in the CLE to log
# information that is useful to know for the user.
# In here, we forward any info message sent to this logger to the notificator
gazebo_logger = logging.getLogger('hbp_nrp_cle.user_notifications')
gazebo_logger.setLevel(logging.INFO)
notificator_handler = NotificatorHandler()


def extract_line_number(tb, filename="<string>"):
    """
    Extracts the line number of the given traceback or returns -1

    :param tb: The traceback with the original error information
    :param filename: the absolute path to the file of the code
    that has raised the error, "<string>" otherwise.
    """
    # find the base of the traceback stack (where the exception has been raised)
    prev = current = tb
    while current is not None:
        prev = current
        current = current.tb_next

    if prev.tb_frame.f_code.co_filename == filename:
        return prev.tb_lineno
    return -1


# pylint: disable=R0902
# the attributes are reasonable in this case
class ROSCLEServer(SimulationServer):
    """
    A ROS server wrapper around the Closed Loop Engine.
    """

    def __init__(self, sim_id, timeout, timeout_type, gzserver, notificator):
        """
        Create the wrapper server

        :param sim_id: The simulation id
        :param timeout: The simulation timeout
        :param timeout_type: The type of simulation timeout
        :param gzserver: Gazebo simulator launch/control instance
        :param notificator: ROS state/error notificator interface
        """
        super(ROSCLEServer, self).__init__(sim_id, timeout, timeout_type, gzserver, notificator)
        self.__cle = None
        self._robotHandler = None
        self._excBibiHandler = None
        self._csv_logger = None
        self.__service_get_transfer_functions = None
        self.__service_add_transfer_function = None
        self.__service_edit_transfer_function = None
        self.__service_activate_transfer_function = None
        self.__service_convert_transfer_function_raw_to_structured = None
        self.__service_delete_transfer_function = None
        self.__service_get_brain = None
        self.__service_set_brain = None
        self.__service_set_populations = None
        self.__service_get_populations = None
        self.__service_get_CSV_recorders_files = None
        self.__service_clean_CSV_recorders_files = None
        self.__service_get_robots = None
        self.__service_add_robot = None
        self.__service_del_robot = None
        self.__service_set_exc_robot_pose = None

        self.__service_prepare_custom_model = None

        self._tuple2slice = (lambda x: slice(*x) if isinstance(x, tuple) else x)

    # pylint: disable=too-many-arguments
    def publish_error(self, source_type, error_type, message,
                      severity=CLEError.SEVERITY_ERROR, function_name="",
                      line_number=-1, offset=-1, line_text="", file_name="", do_log=True):
        """
        Publishes an error and takes appropriate action if necessary

        :param source_type: The module the error message comes from, e.g. "Transfer Function"
        :param error_type: The error type, e.g. "Compile"
        :param message: The error message description, e.g. unexpected indent
        :param severity: The severity of the error
        :param function_name: The function name, if available
        :param line_number: The line number where the error occurred
        :param offset: The offset
        :param line_text: The text of the line causing the error
        :param file_name:
        :param do_log: log the error on the default logger, do not log otherwise
        """
        if do_log and severity >= CLEError.SEVERITY_ERROR:
            logger.exception("Error in {0} ({1}): {2}".format(source_type, error_type, message))

        self._notificator.publish_error(
            CLEError(severity, source_type, error_type, message,
                     function_name, line_number, offset, line_text, file_name))

        if self.lifecycle is not None:
            if severity == CLEError.SEVERITY_MAJOR:
                self.lifecycle.accept_command('paused')
            elif severity == CLEError.SEVERITY_CRITICAL:
                self.lifecycle.failed()

    def __tf_except_hook(self, tf, tf_error, tb):
        """
        Handles an exception in the Transfer Functions

        :param tf: The transfer function that crashed
        :param tf_error: The exception that was thrown
        :param tb: The original traceback
        """
        if tf.updated:
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, "Runtime", str(tf_error),
                               severity=CLEError.SEVERITY_ERROR, function_name=tf.name,
                               line_number=extract_line_number(tb))

    @property
    def simulation_time(self):
        return int(self.__cle.simulation_time)

    @property
    def cle(self):
        """
        Gets the Closed Loop Engine referenced by this server
        """
        return self.__cle

    @cle.setter
    def cle(self, value):
        """
        Sets the Closed Loop Engine of this server
        """
        self.__cle = value

    def setup_handlers(self, assembly):  # pragma: no cover
        """
        Sets up different call handlers for the current assembly.

        :param assembly: CLE assembly of the current simulation
        :return: -
        """
        self._robotHandler = RobotCallHandler(assembly)
        self._excBibiHandler = ExcBibiHandler(assembly)
        self._csv_logger = CSVLogger(assembly)

    def _create_lifecycle(self, except_hook):
        """
        Creates the lifecycle for the current simulation
        :param except_hook: An exception handler
        :return: The created lifecycle
        """
        return SimulationServerLifecycle(self.simulation_id, self.cle, self, except_hook)

    def prepare_simulation(self, except_hook=None):
        """
        The CLE will be initialized within this method and ROS services for
        starting, pausing, stopping and resetting are setup here.

        :param cle: the closed loop engine
        :param except_hook: A handler method for critical exceptions
        """
        super(ROSCLEServer, self).prepare_simulation(except_hook)

        logger.info("Registering ROS Service handlers")

        self.__service_get_transfer_functions = rospy.Service(
            SERVICE_GET_TRANSFER_FUNCTIONS(self.simulation_id), srv.GetTransferFunctions,
            self.__get_transfer_function_sources_and_activation
        )

        self.__service_add_transfer_function = rospy.Service(
            SERVICE_ADD_TRANSFER_FUNCTION(self.simulation_id), srv.AddTransferFunction,
            self.__add_transfer_function
        )

        self.__service_edit_transfer_function = rospy.Service(
            SERVICE_EDIT_TRANSFER_FUNCTION(self.simulation_id), srv.EditTransferFunction,
            self.__edit_transfer_function
        )

        self.__service_activate_transfer_function = rospy.Service(
            SERVICE_ACTIVATE_TRANSFER_FUNCTION(self.simulation_id), srv.ActivateTransferFunction,
            self.__activate_transfer_function
        )

        self.__service_convert_transfer_function_raw_to_structured = rospy.Service(
            SERVICE_CONVERT_TRANSFER_FUNCTION_RAW_TO_STRUCTURED(self.simulation_id),
            srv.ConvertTransferFunctionRawToStructured,
            self.__convert_transfer_function_raw_to_structured
        )

        self.__service_delete_transfer_function = rospy.Service(
            SERVICE_DELETE_TRANSFER_FUNCTION(self.simulation_id), srv.DeleteTransferFunction,
            self.__delete_transfer_function
        )

        self.__service_get_brain = rospy.Service(
            SERVICE_GET_BRAIN(self.simulation_id), srv.GetBrain,
            self.__get_brain
        )

        self.__service_set_brain = rospy.Service(
            SERVICE_SET_BRAIN(self.simulation_id), srv.SetBrain,
            self.__try_set_brain
        )

        self.__service_get_populations = rospy.Service(
            SERVICE_GET_POPULATIONS(self.simulation_id), srv.GetPopulations,
            self.__get_populations
        )

        self.__service_set_populations = rospy.Service(
            SERVICE_SET_POPULATIONS(self.simulation_id), srv.SetPopulations,
            self.__try_set_populations
        )

        self.__service_get_CSV_recorders_files = rospy.Service(
            SERVICE_GET_CSV_RECORDERS_FILES(self.simulation_id), srv.GetCSVRecordersFiles,
            self.__get_CSV_recorders_files
        )

        self.__service_clean_CSV_recorders_files = rospy.Service(
            SERVICE_CLEAN_CSV_RECORDERS_FILES(self.simulation_id), Empty,
            self.__clean_CSV_recorders_files
        )

        self.__service_get_robots = rospy.Service(
            SERVICE_GET_ROBOTS(self.simulation_id), srv.GetRobots,
            self.__get_robots
        )

        self.__service_add_robot = rospy.Service(
            SERVICE_ADD_ROBOT(self.simulation_id), srv.AddRobot,
            self.__add_robot
        )

        self.__service_del_robot = rospy.Service(
            SERVICE_DEL_ROBOT(self.simulation_id), srv.DeleteRobot,
            self.__delete_robot
        )

        self.__service_set_exc_robot_pose = rospy.Service(
            SERVICE_SET_EXC_ROBOT_POSE(self.simulation_id), srv.ChangePose,
            self.__set_robot_initial_pose
        )

        self.__service_prepare_custom_model = rospy.Service(
            SERVICE_PREPARE_CUSTOM_MODEL(self.simulation_id), srv.Resource,
            self.__prepare_custom_model
        )

        tf_framework.TransferFunction.excepthook = self.__tf_except_hook
        tf_framework.TF_API.set_ros_cle_server(self)

    # pylint: disable=unused-argument
    def __get_populations(self, request):
        """
        Gets the populations available in the neural network
        """
        return_val = srv.GetPopulationsResponse(
            [PopulationInfo(str(p.name), str(p.celltype),
                            ROSCLEServer.__convert_parameters(
                                p.parameters), p.gids, p.indices)
             for p in self.__cle.bca.get_populations()]
        )
        return return_val

    # pylint: disable=unused-argument, no-self-use
    def __get_CSV_recorders_files(self, request):
        """
        Return the recorder file names, headers, and values
        """
        return srv.GetCSVRecordersFilesResponse([CSVRecordedFile(recorded_file[0],
                                                                 recorded_file[1],
                                                                 recorded_file[2])
                                                 for recorded_file in
                                                 tf_framework.dump_csv_recorder_to_files()])

    # pylint: disable=unused-argument, no-self-use
    @ros_handler
    def __clean_CSV_recorders_files(self, request):
        """
        Remove temporary recorders files from the server
        """
        tf_framework.clean_csv_recorders_files()

    @staticmethod
    def __convert_parameters(parameters):
        """
        Converts the given parameters to ROS types

        :param parameters: A parameter dictionary
        :return: A list of ROS-compatible neuron parameters
        """
        ros_parameters = []
        for k, v in parameters.iteritems():
            if isinstance(v, float):
                try:
                    # tests if it can be packaged as float. This operation is performed by ROS
                    # service proxy before # sending a request
                    struct.pack('<f', v)
                except OverflowError:
                    # replaces v by maximum float which can be packaged. More info here:
                    # https://mail.python.org/pipermail/python-list/2000-March/058582.html
                    v = 3.4e38
                    logger.info("Neuron Parameter {} with value {} can't be converted to a "
                                "ROS type".format(k, v))
                finally:
                    ros_parameters += [NeuronParameter(str(k), v)]

        return ros_parameters

    # pylint: disable=unused-argument, no-self-use
    def __get_brain(self, request):
        """
        Returns the current neuronal network model. By default we do assume that if the sources are
        not available, the model comes from a h5 file. This has to be refined once we will be more
        confident in the fate of the h5 files.

        :param request: The rospy request parameter
        :return: an array compatible with the GetBrain.srv ROS service
        """
        braintype = "h5"
        data_type = "base64"
        brain_code = "N/A"
        if tf_framework.get_brain_source():
            braintype = "py"
            data_type = "text"
            brain_code = tf_framework.get_brain_source()

        return [braintype, brain_code, data_type,
                json.dumps(tf_framework.get_brain_populations())]

    @staticmethod
    def change_transfer_function_for_population(change_population_mode, old_population_name,
                                                new_population_name, transfer_functions):
        """
        Modifies a single population name. Returns a value if needs user input.

        :param change_population_mode: The change population mode
                                      (as defined in SetPopulationsRequest)
        :param old_population_name: The old name of the population
        :param new_population_name: The new name of the population
        :param transfer_functions: The transfer functions to which the population change should be
                                   applied to.
        """
        for tf in transfer_functions:

            tf_has_changed = False

            for i in range(1, len(tf.params)):
                if hasattr(tf.params[i], 'spec'):
                    spec = tf.params[i].spec

                    if hasattr(spec, 'neurons'):
                        mapping = spec.neurons

                        if mapping is not None:
                            node = None
                            # required item is either neurons or its parent
                            if str(mapping.name) == old_population_name:
                                node = mapping
                            elif str(mapping.parent.name) == old_population_name:
                                node = mapping.parent
                            if node is not None:
                                if change_population_mode == \
                                        srv.SetPopulationsRequest.ASK_RENAME_POPULATION:
                                    # here we send a reply to the frontend to ask
                                    # the user a permission to change TFs
                                    return ["we ask the user if we change TFs", 0, 0, 1]
                                elif change_population_mode == \
                                        srv.SetPopulationsRequest.DO_RENAME_POPULATION:
                                    # permission granted, so we change TFs
                                    node.name = new_population_name
                                    tf_has_changed = True
            if tf_has_changed:
                source = tf.source
                pattern = re.compile(r'(?:(?<=\W)|^)(' + old_population_name + r')(?:(?=\W)|$)')
                modified_source = pattern.sub(new_population_name, source)
                if not modified_source == source:
                    tf.source = modified_source

    def change_transfer_functions(self, change_population, old_changed, new_added,
                                  transfer_functions):
        """
        Modifies population names. Returns a value if needs user input.

        :param change_population: The change population mode (as defined in SetPopulationsRequest)
        :param old_changed: A list of old population names
        :param new_added: A list of new population names
        :param transfer_functions: The transfer functions to which the changes should be applied
        """

        for changed_i in range(len(old_changed)):
            r = self.change_transfer_function_for_population(change_population,
                                                             str(old_changed[changed_i]),
                                                             str(new_added[changed_i]),
                                                             transfer_functions)
            if r is not None:
                return r

    def __try_set_populations(self, request):
        """
        Tries set the neuronal network according to the given request. If it fails, it restores
        the previous neuronal network.

        :param request: The mandatory rospy request parameter
        """
        running = self.__cle.running
        if running:
            self.__cle.stop()

        return_value = self.__set_populations(request.brain_populations,
                                              request.data_type,
                                              request.change_population)

        if not return_value:
            # failed to set new brain, we re set previous valid brain
            _, _, prev_data_type, prev_brain_pops_json = \
                self.__get_brain(None)
            self.__set_populations(prev_brain_pops_json, prev_data_type, request.change_population)
        else:
            self._notificator.publish_state(json.dumps({"action": "updatePopulations"}))

        if running:
            self.__cle.start()

        return return_value

    def __set_populations(self, brain_populations, data_type, change_population):
        # pragma: no cover
        """
        Sets the populations into the brain

        :param brain_populations: A JSON formatted dictionary indexed by population names and
                                  containing neuron indices. Neuron indices could be defined by
                                  individual integers, lists of integers.
        :param data_type: Type of the brain_data field ('text' or 'base64')
        :param change_population: a flag to select an action on population name change, currently
                                  possible values are:
                                  0 ask user for permission to replace;
                                  1 (permission granted) replace old name with a new one;
                                  2 proceed with no replace action
        """
        return_value = ''
        try:
            self.__cle.load_populations(**json.loads(brain_populations))
            if change_population:
                self.__rename_populations(data_type, brain_populations, change_population)
        except AttributeError as e:
            logger.exception(e)
            return_value = e.message
        except Exception, e:
            logger.exception(e)
            return_value = e.message

        return return_value

    def __rename_populations(self, data_type, brain_populations, change_population):
        # pragma: no cover
        """
        Checks whether the given brain change request load_brain_and_populations( is valid
        :param data_type: The data type
        :param brain_populations: The brain populations
        :param change_population: A flag indicating whether populations should be changed
        :return: Success in case everything is alright, otherwise an error message tuple
        """
        if data_type != "text" and data_type != "base64":
            return ["Data type {0} is invalid".format(data_type), 0, 0, 0]

        new_populations = [str(item)
                           for item in json.loads(brain_populations).keys()]
        old_populations = [
            str(item) for item in tf_framework.get_brain_populations().keys()]
        old_changed = find_changed_strings(old_populations, new_populations)
        new_added = find_changed_strings(new_populations, old_populations)
        if len(new_added) == len(old_changed) >= 1:
            transfer_functions = tf_framework.get_transfer_functions(
                flawed=False)
            check_var_name_re = re.compile(r'^([a-zA-Z_]+\w*)$')
            for item in new_added:
                if not check_var_name_re.match(item):
                    return "Provided name \"" + item + "\" is not a valid population name"

            return self.change_transfer_functions(change_population,
                                                  old_changed,
                                                  new_added,
                                                  transfer_functions)
        elif old_changed and len(new_added) > len(old_changed):
            return "Renamed populations must be applied separately. \
                     Please apply the renaming before proceeding with other changes."

        return 'Success'

    def __try_set_brain(self, request):
        """
        Tries set the neuronal network according to the given request. If it fails, it restores
        the previous neuronal network.

        :param request: The mandatory rospy request parameter
        """

        running = self.__cle.running
        if running:
            self.__cle.stop()

        return_value = self.__set_brain(request.brain_type,
                                        request.data_type,
                                        request.brain_data,
                                        request.brain_populations)

        prev_braintype, prev_brain_code, prev_data_type, prev_brain_pops = self.__get_brain(None)
        if return_value[0] != "":
            # failed to set new brain, we reset previous valid brain
            self.__set_brain(prev_braintype, prev_data_type, prev_brain_code, prev_brain_pops)
        else:
            self._notificator.publish_state(json.dumps({"action": "setbrain"}))

        if running:
            self.__cle.start()
        return return_value

    def __set_brain(self, brain_type, data_type, brain_data, brain_populations):  # pragma: no cover
        """
        Sets the neuronal network according to the given parameters

        :param brain_type: Type of the brain file ('h5' or 'py')
        :param data_type: Type of the brain_data field ('text' or 'base64')
        :param brain_data: Contents of the brain file. Encoding given in field data_type
        :param brain_populations: A dictionary indexed by population names and containing neuron
                                  indices. Neuron indices could be defined by individual integers,
                                  lists of integers or python slices. Python slices are defined by a
                                  dictionary containing the 'from', 'to' and 'step' values.
        """
        try:
            return_value = ["", 0, 0]
            with NamedTemporaryFile(prefix='brain', suffix='.' + brain_type, delete=False) as tmp:
                with tmp.file as brain_file:
                    if data_type == "text":
                        brain_file.write(brain_data)
                    else:
                        brain_file.write(base64.decodestring(brain_data))

                self.__cle.load_brain(tmp.name, **json.loads(brain_populations))

        except ValueError, e:
            logger.exception(e)
            return_value = ["Population format is invalid: " + str(e), 0, 0]
        except SyntaxError, e:
            logger.exception(e)
            return_value = ["The new brain could not be parsed: " + str(e), e.lineno, e.offset]
        except AttributeError as e:
            logger.exception(e)
            line_no = extract_line_number(sys.exc_info()[2], tmp.name)
            return_value = ["The new brain has an error: " + e.message, line_no, 0]
        except BrainParameterException as e:
            logger.exception(e)
            return_value = [e.message, -1, -1]
        except Exception, e:
            logger.exception(e)
            line_no = extract_line_number(sys.exc_info()[2], tmp.name)
            return_value = ["Error changing neuronal network: " + str(e), line_no, 0]

        return return_value

    # pylint: disable=unused-argument
    def __get_transfer_function_sources_and_activation(self, request, send_errors=True):
        """
        Return the source code of the transfer functions and its activity state
        in a tuple of parallel lists.

        :param request: The mandatory rospy request parameter
        :param send_errors: when available, send error messages on the cle_error topic
        :return A tuple (tf_sources, tf_activation_state)
        """
        tfs = tf_framework.get_transfer_functions()

        tf_arr = []
        arr_active_mask = []

        for tf in tfs:
            tf_arr.append(tf.source.encode('UTF-8'))
            arr_active_mask.append(tf.active)

            if send_errors and hasattr(tf, 'error'):  # tf may not have an error member
                self._publish_error_from_exception(tf.error, tf.name)

        return numpy.array(tf_arr), numpy.array(arr_active_mask)

    @staticmethod
    def __get_transfer_function_activation(tf_name):  # pragma: no cover
        """
        Return the activation status of the transfer function
        :param tf_name: string holding the name of a transfer function
        :return boolean value which is True if the transfer function is activated, False otherwise
        :raise: A ValueError exception is raised if the transfer function tf_name is not found
        """
        for tf in tf_framework.get_transfer_functions():
            if tf.name == tf_name:
                return tf.active

        raise ValueError(
            "__get_transfer_function_activation: TF {} not found".format(tf_name))

    def _publish_error_from_exception(self, exception, tf_name):
        """
        Publish an error message on the error topic, populating the message using the
        information in exception

        :param exception: the exception from which to create the message to be sent
        :param tf_name: the name of the transfer function that caused the error
        """
        if isinstance(exception, ValueError):
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION,
                               "Loading",
                               "Duplicate Definition Name",
                               function_name=tf_name,
                               do_log=False)
        elif isinstance(exception, SyntaxError):
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION,
                               "Compile",
                               str(exception),
                               severity=CLEError.SEVERITY_ERROR,
                               function_name=tf_name,
                               line_number=exception.lineno, offset=exception.offset,
                               line_text=exception.text, file_name=exception.filename,
                               do_log=False)
        elif isinstance(exception, Exception):
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION,
                               "Compile",
                               str(exception),
                               severity=CLEError.SEVERITY_ERROR,
                               function_name=tf_name,
                               do_log=False)

    def __check_duplicate_tf_name(self, tf_name):
        """
        Internal helper to check whether a transfer function name already exists, excluding
        flawed transfer function.

        :param tf_name: transfer function name to be checked.
        :return: Raises a ValueError if name already exists. Returns nothing otherwise.
        """
        tfs, _ = self.__get_transfer_function_sources_and_activation(None, send_errors=False)

        for tf_name_ in [self.get_tf_name(tf)[1] for tf in tfs]:
            # compare the names but only if it's not a flawed TF
            if tf_name_ == tf_name and not tf_framework.get_flawed_transfer_function(tf_name_):
                raise ValueError("Duplicate definition name")

    def __add_transfer_function(self, request):
        """
        Adds a new transfer function

        :param request: The ROS Service request message (cle_ros_msgs.srv.AddTransferFunction)
        :return: empty string for a successful compilation in restricted mode
                 (executed synchronously), an error message otherwise.
        """
        return self.__set_transfer_function(request, True)

    def __edit_transfer_function(self, request):
        """
        Modifies an existing transfer function

        :param request: The mandatory rospy request parameter
        :return: empty string for a successful compilation in restricted mode
                 (executed synchronously), an error message otherwise.
        """
        if tf_framework.get_flawed_transfer_function(request.transfer_function_name):
            return self.__set_flawed_transfer_function(request)
        else:
            return self.__set_transfer_function(request, False)

    def __activate_transfer_function(self, request):
        """
        Set the activation state of a Transfer Function

        :param request: The mandatory rospy request parameter
        :return: empty string if successful, an error message otherwise.
        """
        original_name = request.transfer_function_name
        activate = request.activate

        tf_to_activate = tf_framework.get_transfer_function(original_name)

        message = ""

        if tf_to_activate is None:
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, "NoOrMultipleNames",
                               original_name, severity=CLEError.SEVERITY_ERROR)
            message = "Can't change activation status: Transfer Function not found"
        else:
            tf_framework.activate_transfer_function(tf_to_activate, activate)

        return message

    def __set_flawed_transfer_function(self, request):
        """
        Patch a flawed transfer function.
        This method is called when editing a flawed transfer function.
        If the new code is loaded without any error,
        the flawed transfer function is "promoted" to a proper TF.

        :param request: The mandatory rospy request parameter
        :return: empty string if successful, an error message otherwise.
        """

        tf_name = request.transfer_function_name

        tf_framework.delete_flawed_transfer_function(tf_name)

        return self.__set_transfer_function(request, True)

    def __compile(self, source):
        """
        Compiles code source with customized rules
        """

        class FileRestrictionMutator(RestrictionMutator):
            """
            Custom RestrictionMutator
            ie: accepts accessing to private attribute '__file__'
            """
            ALLOWED_ATTRS = set(['__file__'])

            def checkAttrName(self, node):
                if node.attrname in self.ALLOWED_ATTRS:
                    return
                return RestrictionMutator.checkAttrName(self, node)

        gen = RModule(source, '<string>')
        gen.rm = FileRestrictionMutator()
        gen.compile()
        return gen.getCode()

    # pylint: disable=R0911,too-many-branches,too-many-statements,too-many-locals
    def __set_transfer_function(self, request, new):
        """
        Patch a transfer function. This method is called when adding or editing a transfer function.

        :param request: The mandatory rospy request parameter
        :param new: A boolean indicating whether a transfer function is being added or modified.
        :return: empty string for a successful compilation
                 in restricted mode (executed synchronously), an error message otherwise.
        """
        new_source = textwrap.dedent(request.transfer_function_source)
        logger.info("About to compile transfer function with the following python code: \n" +
                    repr(new_source))

        # Make sure the TF has exactly one function definition
        get_tf_name_outcome, get_tf_name_outcome_value = self.get_tf_name(new_source)
        if get_tf_name_outcome is True:
            new_name = get_tf_name_outcome_value
        else:
            error_message = get_tf_name_outcome_value

            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION,
                               "NoOrMultipleNames",
                               error_message,
                               severity=CLEError.SEVERITY_ERROR,
                               function_name=getattr(request, 'transfer_function_name', "no_name"))

            return error_message

        if not new:
            original_name = request.transfer_function_name
        else:
            original_name = None

        # Make sure function name is not a duplicate
        if new or (new_name != original_name):
            try:
                self.__check_duplicate_tf_name(new_name)
            except ValueError as value_e:
                message = "duplicate Transfer Function name"
                # Select the correct function to highlight
                function_name = new_name if original_name is None else original_name

                self._publish_error_from_exception(value_e, new_name)

                if new:
                    tf_framework.set_flawed_transfer_function(new_source, function_name, value_e)

                return message

        # Compile (synchronously) transfer function's new code in restricted mode
        new_code = None
        try:
            new_code = self.__compile(new_source)
        # pylint: disable=broad-except
        except Exception as e:
            message = "Error while compiling the updated" \
                      + " transfer function named " + new_name \
                      + " in restricted mode.\n" \
                      + str(e)
            if new:
                tf_framework.set_flawed_transfer_function(new_source, new_name, e)

            self._publish_error_from_exception(e, new_name)

            return message

        # Make sure CLE is stopped. If already stopped, these calls are harmless.
        # (Execution of updated code is asynchronous)
        running = self.__cle.running
        err = ""

        if running:
            self.__cle.stop()

        # Try to load the TF
        try:
            # When editing a TF (i.e. not new), delete it before loading the new one
            activation = True
            if not new and original_name is not None:
                activation = self.__get_transfer_function_activation(original_name)
                tf_framework.delete_transfer_function(original_name)
            tf_framework.set_transfer_function(
                new_source, new_code, new_name, activation=activation
            )
        except TFLoadingException as e:
            self.publish_error(CLEError.SOURCE_TYPE_TRANSFER_FUNCTION, "Loading", e.message,
                               severity=CLEError.SEVERITY_ERROR,
                               function_name=new_name)
            err = e.message
            tf_framework.set_flawed_transfer_function(new_source, new_name, e)

        if running:
            self.__cle.start()

        return err

    @staticmethod
    def get_tf_name(source):
        """
        Check whether the function has a single definition name

        :param source: Transfer function source code
        :return: True and Function name if found, False and error message if none or multiple found
        """
        m = re.findall(r"(?:\n|^)def\s+(\w+)\s*\(", source)
        if len(m) != 1:
            if len(m) == 0:
                error_msg = "New Transfer Function has no definition name."
            else:
                error_msg = "New Transfer Function has multiple definition names."
            error_msg += " Compilation aborted"
            return False, error_msg
        return True, m[0]

    def __set_structured_transfer_function(self, request):
        """
        Patch a structured transfer function

        :param request: The mandatory rospy request parameter
        :return: empty string for a successful compilation in restricted mode
                 (executed synchronously), an error message otherwise.
        """

        edit_tf_request = srv.EditTransferFunctionRequest()

        edit_tf_request.transfer_function_name = request.transfer_function.name
        edit_tf_request.transfer_function_source = \
            StructuredTransferFunction.generate_code_from_structured_tf(request.transfer_function)

        return self.__set_transfer_function(edit_tf_request, False)

    def __convert_transfer_function_raw_to_structured(self, request):
        """
        Convert a raw tf to structured format

        :param request: The mandatory rospy request parameter
        :return: The converted TF.__conv
        """
        new_source = textwrap.dedent(request.transfer_function_source)
        return StructuredTransferFunction.extract_structure(new_source), None

    def __delete_transfer_function(self, request):
        """
        Delete an existing transfer function

        :param request: The mandatory rospy request parameter
        :return: True if the delete was successful, False if the transfer function does not exist.
        """
        running = self.__cle.running
        if running:
            self.__cle.stop()
        ret = tf_framework.delete_transfer_function(request.transfer_function_name)
        if running:
            self.__cle.start()
        return ret

    def _create_state_message(self):
        return {
            'realTime': int(self.__cle.real_time),
            'transferFunctionsElapsedTime': self.__cle.tf_elapsed_time(),
            'brainsimElapsedTime': self.__cle.brainsim_elapsed_time(),
            'robotsimElapsedTime': self.__cle.robotsim_elapsed_time()
        }

    def shutdown(self):
        """
        Shutdown the cle server
        """
        super(ROSCLEServer, self).shutdown()

        # kill the csv logger thread
        if self._csv_logger is not None:
            self._csv_logger.shutdown()

        # the cle and services are initialized in prepare_simulation, which is not
        # guaranteed to have occurred before shutdown is called
        if self.__cle is not None:
            logger.info("Shutting down the closed loop service")
            self.__cle.shutdown()
            time.sleep(2)
            logger.info("Shutting down get_transfer_functions service")
            self.__service_get_transfer_functions.shutdown()
            self.__service_convert_transfer_function_raw_to_structured.shutdown()
            logger.info("Shutting down activate_transfer_function service")
            self.__service_activate_transfer_function.shutdown()
            logger.info("Shutting down add_transfer_function service")
            self.__service_add_transfer_function.shutdown()
            logger.info("Shutting down set_transfer_function services")
            self.__service_edit_transfer_function.shutdown()
            logger.info("Shutting down delete_transfer_function services")
            self.__service_delete_transfer_function.shutdown()
            logger.info("Shutting down get_brain service")
            self.__service_get_brain.shutdown()
            logger.info("Shutting down set_brain service")
            self.__service_set_brain.shutdown()
            logger.info("Shutting down get_populations service")
            self.__service_set_populations.shutdown()
            logger.info("Shutting down get_populations service")
            self.__service_get_populations.shutdown()
            logger.info("Shutting down get_CSV_recorders_files service")
            self.__service_get_CSV_recorders_files.shutdown()
            logger.info("Shutting down clean_CSV_recorders_files service")
            self.__service_clean_CSV_recorders_files.shutdown()
            logger.info("Shutting down get_robots service")
            self.__service_get_robots.shutdown()
            logger.info("Shutting down add_robot service")
            self.__service_add_robot.shutdown()
            logger.info("Shutting down delete_robot service")
            self.__service_del_robot.shutdown()
            logger.info("Shutting down set_robot_initial_pose service")
            self.__service_set_exc_robot_pose.shutdown()
            logger.info("Shutting down prepare_custom_model service")
            self.__service_prepare_custom_model.shutdown()

    def _reset_world(self, request):
        """
        Helper function for reset() call, it handles the RESET_WORLD message.

        :param request: the ROS service request message (cle_ros_msgs.srv.ResetSimulation).
        """
        with self._notificator.task_notifier("Resetting Environment", "Emptying 3D world"):
            if request.world_sdf is not None and request.world_sdf is not "":
                sdf_world_string = request.world_sdf
                self.__cle.reset_world(sdf_world_string)
            else:
                self.__cle.reset_world()

    def _reset_brain(self, request):
        """
        Helper function for reset() call, it handles the RESET_BRAIN message.

        :param request: the ROS service request message (cle_ros_msgs.srv.ResetSimulation).
        """
        if request.brain_path is not None and request.brain_path is not "":
            brain_temp_path = request.brain_path
            neurons_conf = request.populations
            network_conf_orig = {p.name: self._get_population_value(p) for p in neurons_conf}
            self.__cle.reset_brain(brain_temp_path, network_conf_orig)
        else:
            self.__cle.reset_brain()

    def _reset_robot_pose(self):
        """
        Helper function for reset() call, it handles the RESET_ROBOT_POSE message.
        """
        self.__cle.reset_robot_pose()
        with self._notificator.task_notifier("Resetting the robot pose", ""):
            self._notificator.update_task("Restoring the robot pose", False, True)
            self.__cle.reset_robot_pose()

    def _reset_full(self, request):
        """
        Helper function for reset() call, it handles the RESET_FULL message.

        :param request: the ROS service request message (cle_ros_msgs.srv.ResetSimulation).
        """

        self.__cle.reset_robot_pose()
        if request.world_sdf is not None and request.world_sdf is not "":
            sdf_world_string = request.world_sdf
            brain_temp_path = request.brain_path
            neurons_conf = request.populations
            network_conf_orig = {
                p.name: self._get_population_value(p) for p in neurons_conf
            }
            with self._notificator.task_notifier("Resetting the simulation", ""):
                self._notificator.update_task("Restoring the 3D world", False, True)
                self.__cle.reset_world(sdf_world_string)
                self._notificator.update_task("Restoring the brain", False, True)
                self.__cle.reset_brain(brain_temp_path, network_conf_orig)
        else:
            with self._notificator.task_notifier("Resetting the simulation", ""):
                self._notificator.update_task("Restoring the 3D world", False, True)
                self.__cle.reset_world()
                self._notificator.update_task("Restoring the brain", False, True)
                self.__cle.reset_brain()

        # Member added by transitions library
        # pylint: disable=no-member
        self.lifecycle.initialized()

    def start_fetching_gazebo_logs(self):
        """
        Starts to fetch the logs from gazebo and putting them as notifications
        """
        gazebo_logger.handlers.append(notificator_handler)
        Notificator.register_notification_function(
            lambda subtask, update_progress: self._notificator.update_task(subtask,
                                                                           update_progress,
                                                                           True))

    def stop_fetching_gazebo_logs(self):
        """
        Stop fetching logs from gazebo and putting them as notifications
        """
        gazebo_logger.handlers.remove(notificator_handler)

    # pylint: disable=broad-except
    def reset_simulation(self, request):
        """
        Handler for the CLE reset() call.

        :param request: the ROS service request message (cle_ros_msgs.srv.ResetSimulation).
        """

        rsr = srv.ResetSimulationRequest
        reset_type = request.reset_type
        if reset_type == rsr.RESET_OLD:
            # Member added by transitions library
            # pylint: disable=no-member
            self.lifecycle.initialized()
        else:
            try:
                self.start_fetching_gazebo_logs()
                if reset_type == rsr.RESET_ROBOT_POSE:
                    self._reset_robot_pose()
                elif reset_type == rsr.RESET_WORLD:
                    self._reset_world(request)
                elif reset_type == rsr.RESET_BRAIN:
                    self._reset_brain(request)
                elif reset_type == rsr.RESET_FULL:
                    self._reset_full(request)
                # reset csv loggers
                if self._csv_logger:
                    self._csv_logger.reset()
            except Exception as e:
                return False, str(e)
            finally:
                self.stop_fetching_gazebo_logs()

        return True, ""

    @staticmethod
    def _get_population_value(population_info):
        """
        Retrieves the population value given the type of the population

        :param population_info: The serialized population info object
        :return: The population value
        """
        if population_info.type == ExperimentPopulationInfo.TYPE_POPULATION_SPINNAKER:
            return population_info.name
        elif population_info.type == ExperimentPopulationInfo.TYPE_ENTIRE_POPULATION:
            return None
        elif population_info.type == ExperimentPopulationInfo.TYPE_POPULATION_SLICE:
            return slice(population_info.start, population_info.stop, population_info.step)
        elif population_info.type == ExperimentPopulationInfo.TYPE_POPULATION_LISTVIEW:
            return population_info.ids

    def __get_robots(self, request):
        """

        :param request: rospy parameters sent from ROSCLEClient defined at
                        GetRobot.srv in GazeboRosPackages
        :return: robots: List of robots
        """
        robots = self._robotHandler.get_robots()
        return srv.GetRobotsResponse(robots)

    def __add_robot(self, request):
        """
        :param request: rospy parameters sent from ROSCLEClient defined at
            AddRobot.srv in GazeboRosPackages
        :return: Successfully added. (success, error_message) = (True, None)
            or (False, error_message)
        """
        rinfo = request.robot
        ret, status = self._robotHandler.add_robot(robot_id=rinfo.robot_id,
                                                   robot_model=rinfo.robot_model,
                                                   is_custom=rinfo.is_custom,
                                                   pose=rinfo.pose)
        if ret:
            # add new tag into bibi and exc
            try:
                self._excBibiHandler.add_robotpose(rinfo.robot_id, rinfo.pose)
                self._excBibiHandler.add_bodymodel(rinfo.robot_id, status,
                                                   rinfo.is_custom,
                                                   rinfo.robot_model)
            except Exception as e:
                logger.error("An error occurred while updating exc and bibi for the newly added "
                             "robot" + str(e))

        return srv.AddRobotResponse(success=ret, error_message=status)

    def __delete_robot(self, request):
        """
        Deletes a robot from the simulation

        :param request: rospy parameters sent from ROSCLEClient defined at
                        DeleteRobot.srv in GazeboROSPackages
        :return: Successfully added. (success, error_message) = (True, None)
            or (False, error_message)
        """
        try:
            ret, status = self._robotHandler.delete_robot(robot_id=request.robot_id)
        except Exception as e:
            logger.error("An error occurred while updating exc and bibi for the newly added "
                         "robot" + str(e))
        if ret:
            # removes tags from the bibi and exc
            try:
                self._excBibiHandler.delete_robotpose(request.robot_id)
                self._excBibiHandler.delete_bodymodel(request.robot_id)
            except Exception as e:
                logger.error("An error occurred while updating exc and bibi for the newly added "
                             "robot" + str(e))

        return srv.DeleteRobotResponse(success=ret, error_message=status)

    def __set_robot_initial_pose(self, request):
        """

        :param request: rospy parameters sent from ROSCLEClient defined at
            ChangePose.srv in GazeboROSPackages
        :return: Successfully added. (success, error_message) = (True, None)
            or (False, error_message)
        """
        try:
            ret, status = self._excBibiHandler.update_robotpose(request.object_id,
                                                                request.new_pose)
        except Exception as e:
            logger.error("An error occurred while updating exc and bibi for the newly added "
                         "robot" + str(e))

        return srv.ChangePoseResponse(success=ret, error_message=status)

    def __prepare_custom_model(self, request):
        """

        :param request: rospy parameters sent from ROSCLEClient defined at
            ChangePose.srv in GazeboRosPackages
        :return: Successfully added. (success, error_message) = (True, None)
            or (False, error_message)
        """
        if request.resource_type == "robots":
            try:
                ret, status = self._robotHandler.prepare_custom_robot(request.resource_path)
            # pylint: disable=broad-except
            except Exception as e:
                logger.error("An error occurred while preparing custom model" + str(e))
        else:
            raise NotImplementedError("Not implemented")

        return srv.ResourceResponse(success=ret, error_message=status)
