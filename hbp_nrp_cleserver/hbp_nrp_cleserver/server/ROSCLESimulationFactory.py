"""
Advertise a ROS service to start new simulations.
"""

import logging
import rospy
import imp
import threading
import os
import argparse
import sys
import hbp_nrp_cle
import traceback
import signal
# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackage folder at the root of this CLE repository.
from cle_ros_msgs import srv
from hbp_nrp_cleserver.server import ROS_CLE_NODE_NAME, SERVICE_CREATE_NEW_SIMULATION, \
    SERVICE_VERSION, SERVICE_HEALTH, SERVICE_IS_SIMULATION_RUNNING

from hbp_nrp_commons.generated import bibi_api_gen
from hbp_nrp_commons.generated import exp_conf_api_gen
from pyxb import ValidationError

__author__ = "Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli"

logger = logging.getLogger('hbp_nrp_cleserver')

# Warning: We do not use __name__  here, since it translates to __main__
# when this file is run directly (such as python ROSCLESimulationFactory.py)


class ROSCLESimulationFactory(object):
    """
    The purpose of this class is to start simulation thread and to
    provide a ROS service for that. Only one simulation can run at a time.
    """

    def __init__(self):
        """
        Create a CLE simulation factory.
        """
        logger.debug("Creating new CLE server.")
        self.running_simulation_thread = None
        self.__is_running_simulation_terminating = False
        self.simulation_initialized_event = threading.Event()
        self.simulation_terminate_event = threading.Event()
        self.simulation_exception_during_init = None
        self.__simulation_count = 0
        self.__failed_simulation_count = 0

    @property
    def simulation_count(self):
        """
        Gets the total number of simulations ran on this server yet
        """
        return self.__simulation_count

    @property
    def failed_simulation_count(self):
        """
        Gets the number of failed simulations on this server so far
        """
        return self.__failed_simulation_count

    def initialize(self):
        """
        Initializes the Simulation factory
        """
        rospy.init_node(ROS_CLE_NODE_NAME)
        rospy.Service(
            SERVICE_CREATE_NEW_SIMULATION, srv.CreateNewSimulation, self.create_new_simulation
        )
        rospy.Service(
            SERVICE_IS_SIMULATION_RUNNING, srv.IsSimulationRunning, self.is_simulation_running
        )
        rospy.Service(SERVICE_VERSION, srv.GetVersion, self.get_version)
        rospy.Service(SERVICE_HEALTH, srv.Health, self.health)

    @staticmethod
    def run():
        """
        Start the factory and wait indefinitely. (see rospy.spin documentation)
        """
        rospy.spin()

    # service_request is an unused but mandatory argument
    # pylint: disable=unused-argument
    @staticmethod
    def get_version(service_request):
        """
        Handler for the ROS service. Retrieve the CLE version.
        Warning: Multiprocesses can not be used: https://code.ros.org/trac/ros/ticket/972

        :param: service_request: ROS service message (defined in hbp ROS packages)
        """
        return str(hbp_nrp_cle.__version__)

    # service_request is an unused but mandatory argument
    # pylint: disable=unused-argument
    def health(self, service_request):
        """
        Handler for the ROS service that returns the health of the whole program.
        This health is made of two part:
        - A status string containing OK, WARNING or CRITICAL
        - A explanation about why the status has been set to a particular value

        :param service_request: ROS service message (defined in hbp ROS packages)
        :return: an array containing the status and the explanation
        """
        status = ''
        if self.__failed_simulation_count == 0:
            status = 'OK'
        elif self.__failed_simulation_count <= self.__simulation_count / 2:
            status = 'WARNING'
        else:
            status = 'CRITICAL'
        info = "%d error(s) in %d simulations" % \
               (self.__failed_simulation_count, self.__simulation_count)
        return [status, info]

    # service_request is an unused but mandatory argument
    # pylint: disable=unused-argument
    def is_simulation_running(self, request):
        """
        Handler for the ROS service to retrieve information whether there is a simulation running

        :param request: The ROS Service message
        :return: True, if a simulation is running, otherwise False
        """
        return self.running_simulation_thread is not None and \
            self.running_simulation_thread.is_alive()

    def create_new_simulation(self, service_request):
        """
        Handler for the ROS service. Spawn a new simulation.
        Warning: Multiprocesses can not be used: https://code.ros.org/trac/ros/ticket/972

        :param service_request: ROS service message (defined in hbp ROS packages)
        """
        logger.info("Create new simulation request")
        error_message = ""
        result = True

        if self.__is_running_simulation_terminating:
            logger.info("Waiting for previous simulation to terminate")
            self.simulation_terminate_event.wait()

        if (self.running_simulation_thread is None) or \
                (not self.running_simulation_thread.is_alive()):
            logger.info("No simulation running, starting a new simulation.")

            self.__simulation_count += 1

            # In the future, it would be great to move the CLE script generation logic here.
            # For the time beeing, we rely on the calling process to send us this thing.
            self.simulation_initialized_event.clear()

            self.running_simulation_thread = threading.Thread(
                target=self.__simulation,
                args=(service_request.environment_file,
                      service_request.generated_cle_script_file,
                      service_request.gzserver_host,
                      service_request.sim_id)
            )

            self.running_simulation_thread.daemon = True
            logger.info("Spawning new thread that will manage the experiment execution.")
            self.running_simulation_thread.start()
            self.simulation_initialized_event.wait()
            # Raising the exception here will send it back through ROS to the ExDBackend.
            if self.simulation_exception_during_init:
                # Known pylint bug: goo.gl/WNg0TJ
                # pylint: disable=raising-bad-type
                self.__failed_simulation_count += 1
                raise self.simulation_exception_during_init[1], None, \
                    self.simulation_exception_during_init[2]
        else:
            error_message = "Trying to initialize a new simulation even though the " \
                            "previous one has not been terminated."
            logger.error(error_message)
            result = False

        logger.debug("create_new_simulation return with status: " + str(result))
        return [result, error_message]

    def __simulation(self, environment_file, generated_cle_or_exd_config, gzserver_host, sim_id):
        """
        Main simulation method. Start the simulation from the given script file.

        :param: environment_file: Gazebo world file containing
                                  the environment (without the robot)
        :param: generated_cle_script_file: Generated CLE python script (main loop)
        """

        # todo: remove print
        print("create new simulation {0} with for {1}".format(sim_id, gzserver_host))

        if generated_cle_or_exd_config.endswith('.py'):
            return self.__simulation_py(environment_file, generated_cle_or_exd_config)
        elif generated_cle_or_exd_config.endswith('.xml'):
            return self.__simulation_xml(environment_file, generated_cle_or_exd_config,
                                         gzserver_host, sim_id)
        else:
            raise Exception("given file was neither py nor xml.")

    def __simulation_xml(self, environment_file, exd_config_file, gzserver_host, sim_id):
        """
        Main simulation method. Start the simulation from the given script file.

        :param: environment_file: Gazebo world file containing
                                  the environment (without the robot)
        :param: exd_config_file: The ExD Configuration file
        """
        self.__is_running_simulation_terminating = False
        self.simulation_terminate_event.clear()
        logger.info(
            "Preparing new simulation with environment file: %s "
            "and ExD config file: %s.",
            environment_file, exd_config_file
        )
        logger.info("Starting the experiment closed loop engine.")
        cle_server = models_path = gzweb = gzserver = None
        self.simulation_exception_during_init = None

        # We want any exception raised during initialization in this tread
        # to be pass to the main thread so that it can be handled properly.
        # noinspection PyBroadException
        try:
            logger.info("Read XML Files")
            exd, bibi = get_experiment_data(exd_config_file)

            logger.info("Create CLELauncher object")

            # This import starts NEST. Don't move it to the imports at the top of the file,
            # because NEST shall be started on the simulation thread.
            from hbp_nrp_cleserver.server.CLELauncher import CLELauncher

            cle_launcher = CLELauncher(exd, bibi, get_basepath(), gzserver_host, sim_id)
            [cle_server, models_path, gzweb, gzserver] = \
                cle_launcher.cle_function_init(environment_file)

            if cle_server is None:
                raise Exception("Error in cle_function_init. Cannot start simulation.")

        # pylint: disable=broad-except
        except Exception:
            logger.exception("Initialization failed")
            self.simulation_exception_during_init = sys.exc_info()
            self.simulation_initialized_event.set()
            return

        logger.info("Initialization done")
        self.simulation_initialized_event.set()

        cle_server.main()
        self.__is_running_simulation_terminating = True
        try:
            logger.info("Shutdown simulation")
            cle_launcher.shutdown(cle_server, models_path, gzweb, gzserver)
        finally:
            self.running_simulation_thread = None
            self.__is_running_simulation_terminating = False
            self.simulation_terminate_event.set()

    def __simulation_py(self, environment_file, generated_cle_script_file):
        """
        Main simulation method. Start the simulation from the given script file.

        :param: environment_file: Gazebo world file containing
                                  the environment (without the robot)
        :param: generated_cle_script_file: Generated CLE python script (main loop)
        """
        self.__is_running_simulation_terminating = False
        self.simulation_terminate_event.clear()
        logger.info(
            "Preparing new simulation with environment file: %s "
            "and generated script file %s.",
            environment_file, generated_cle_script_file
        )
        logger.info("Starting the experiment closed loop engine.")
        cle_server = models_path = gzweb = gzserver = None
        self.simulation_exception_during_init = None
        experiment_generated_script = imp.load_source(
            'experiment_generated_script', generated_cle_script_file)
        logger.info("Executing script: " + generated_cle_script_file)
        # We want any exception raised during initialization in this tread
        # to be pass to the main thread so that it can be handled properly.
        # noinspection PyBroadException
        try:
            [cle_server, models_path, gzweb, gzserver] = experiment_generated_script. \
                cle_function_init(environment_file)
        # pylint: disable=broad-except
        except Exception:
            logger.exception("Initialization failed")
            self.simulation_exception_during_init = sys.exc_info()
            self.simulation_initialized_event.set()
            return

        logger.info("Initialization done")
        self.simulation_initialized_event.set()

        cle_server.main()
        self.__is_running_simulation_terminating = True
        try:
            logger.info("Shutdown simulation")
            experiment_generated_script.shutdown(cle_server, models_path, gzweb, gzserver)
        finally:
            self.running_simulation_thread = None
            self.__is_running_simulation_terminating = False
            self.simulation_terminate_event.set()


def get_basepath():
    """
    :return: path given in the environment variable 'NRP_MODELS_DIRECTORY'
    """

    path = os.environ.get('NRP_MODELS_DIRECTORY')
    if path is None:
        raise Exception('Environment Variable NRP_MODELS_DIRECTORY is not set on the server')

    return path


def get_experiment_data(experiment_file):
    """
    Parse experiment and bibi and return the objects

    @param experiment_file: :param: environment_file: Gazebo world file containing
                                  the environment (without the robot)
    @return experiment, bibi: types: exp_conf_api_gen.ExD_, bibi_api_gen.BIBIConfiguration
    """
    experiment_dir = "ExDConf"
    path = os.path.join(get_basepath(), experiment_dir)
    experiment_file_abs = os.path.join(path, experiment_file)

    with open(experiment_file_abs) as exd_file:
        try:
            experiment = exp_conf_api_gen.CreateFromDocument(exd_file.read())
        except ValidationError, ve:
            raise Exception("Could not parse experiment configuration {0:s} due to validation "
                            "error: {0:s}".format(experiment_file_abs, str(ve)))

    bibi_file = experiment.bibiConf.src
    logger.info("Bibi: " + bibi_file)

    bibi_file_abs = os.path.join(get_basepath(), bibi_file)
    logger.info("BibiAbs:" + bibi_file_abs)
    with open(bibi_file_abs) as b_file:
        try:
            bibi = bibi_api_gen.CreateFromDocument(b_file.read())
        except ValidationError, ve:
            raise Exception("Could not parse experiment configuration {0:s} due to validation "
                            "error: {0:s}".format(bibi_file_abs, str(ve)))

    return experiment, bibi


# pylint: disable=unused-argument
def print_full_stack_trace(sig, frame):
    """
    Log the stack trace of all the threads
    :param sig: The received signal
    :param frame: The current stack frame
    """
    logger.warn("*** STACKTRACE - START ***")
    code = []
    # pylint: disable=protected-access
    for threadId, stack in sys._current_frames().items():
        code.append("# ThreadID: %s" % threadId)
        for filename, lineno, name, line in traceback.extract_stack(stack):
            code.append('File: "%s", line %d, in %s' % (filename,
                                                        lineno, name))
            if line:
                code.append("  %s" % (line.strip()))
    for line in code:
        logger.warn(line)
    logger.warn("*** STACKTRACE - END ***")


def set_up_logger(logfile_name, verbose=False):
    """
    Configure the root logger of the CLE application
    :param: logfile_name: name of the file created to collect logs
    :param: verbose: increase logging verbosity
    """
    # We initialize the logging in the startup of the whole CLE application.
    # This way we can access the already set up logger in the children modules.
    # Also the following configuration can later be easily stored in an external
    # configuration file (and then set by the user).
    log_format = '%(asctime)s [%(threadName)-12.12s] [%(name)-12.12s] [%(levelname)s]  %(message)s'

    try:
        file_handler = logging.FileHandler(logfile_name)
        file_handler.setFormatter(logging.Formatter(log_format))
        logging.root.addHandler(file_handler)
    except (AttributeError, IOError) as _:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(logging.Formatter(log_format))
        logging.root.addHandler(console_handler)
        logger.warn("Could not write to specified logfile or no logfile specified, "
                    "logging to stdout now!")
    logging.root.setLevel(logging.DEBUG if verbose else logging.INFO)


def main():
    """
    Main function of ROSCLESimulationFactory
    """
    if os.environ["ROS_MASTER_URI"] == "":
        raise Exception("You should run ROS first.")

    signal.signal(signal.SIGUSR1, print_full_stack_trace)
    parser = argparse.ArgumentParser()
    parser.add_argument('--logfile', dest='logfile', help='specify the CLE logfile')
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                        action="store_true")
    parser.add_argument('-p', '--pycharm',
                        dest='pycharm',
                        help='debug with pyCharm. IP adress and port are needed.',
                        nargs='+')
    args = parser.parse_args()

    if (args.pycharm):
        # pylint: disable=import-error
        import pydevd

        pydevd.settrace(args.pycharm[0],
                        port=int(args.pycharm[1]),
                        stdoutToServer=True,
                        stderrToServer=True)

    server = ROSCLESimulationFactory()
    server.initialize()
    set_up_logger(args.logfile, args.verbose)
    server.run()
    logger.info("CLE Server exiting.")


if __name__ == '__main__':  # pragma: no cover
    main()