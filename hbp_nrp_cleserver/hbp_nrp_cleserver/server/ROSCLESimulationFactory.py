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
Advertise a ROS service to start new simulations.
"""

import logging
import rospy
import threading
import os
import argparse
import sys
import hbp_nrp_cle
import traceback
import signal
import dateutil.parser as datetime_parser
# This package comes from the catkin package ROSCLEServicesDefinitions
# in the GazeboRosPackage folder at the root of this CLE repository.
from cle_ros_msgs import srv
from hbp_nrp_cleserver.server import ROS_CLE_NODE_NAME, SERVICE_CREATE_NEW_SIMULATION, \
    SERVICE_VERSION, SERVICE_IS_SIMULATION_RUNNING
from hbp_nrp_cleserver.server.PlaybackServer import PlaybackSimulationAssembly

from hbp_nrp_cleserver.server import ServerConfigurations
import gc
from hbp_nrp_cleserver.server.__signal_patch import patch_signal
from hbp_nrp_cleserver.server.SimulationServer import TimeoutType
from hbp_nrp_commons.sim_config.SimConfig import SimConfig, SimulationType

__author__ = "Lorenzo Vannucci, Stefan Deser, Daniel Peppicelli, Hossain Mahmud"

# Warning: We do not use __name__  here, since it translates to __main__
# when this file is run directly (such as python ROSCLESimulationFactory.py)
logger = logging.getLogger('hbp_nrp_cleserver')


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
        self.simulation_terminate_event = threading.Event()

        self.__create_simulation_service = None

    def initialize(self):
        """
        Initializes the Simulation factory
        """
        rospy.init_node(ROS_CLE_NODE_NAME, anonymous=True)
        self.__create_simulation_service = rospy.Service(
            SERVICE_CREATE_NEW_SIMULATION, srv.CreateNewSimulation, self.create_new_simulation
        )
        rospy.Service(
            SERVICE_IS_SIMULATION_RUNNING, srv.IsSimulationRunning, self.is_simulation_running
        )
        rospy.Service(SERVICE_VERSION, srv.GetVersion, self.get_version)

    def except_hook(self, e):
        """
        Gets called when the simulation server has to shut down due to an exception

        :param e: The exception that caused the simulation server to give up
        """
        logger.exception(e)
        logger.info("Giving up the simulation server")
        self.__create_simulation_service.shutdown()

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
    def is_simulation_running(self, request):
        """
        Handler for the ROS service to retrieve information whether there is a simulation running

        :param request: The ROS Service message
        :return: True, if a simulation is running, otherwise False
        """
        return (self.running_simulation_thread is not None and
                self.running_simulation_thread.is_alive())

    # pylint: disable=too-many-locals, too-many-statements, too-many-branches
    def create_new_simulation(self, service_request):
        """
        Handler for the ROS service. Spawn a new simulation.
        Warning: Multiprocesses can not be used: https://code.ros.org/trac/ros/ticket/972

        :param: service_request: ROS service message (defined in hbp ROS packages)
        """
        logger.info("Create new simulation request")

        if self.__is_running_simulation_terminating:
            logger.info("Waiting for previous simulation to terminate")
            self.simulation_terminate_event.wait()

        if self.is_simulation_running(None):
            error_message = "CLE server is already running an experiment. Cannot initialize another"
            logger.error(error_message)
            raise Exception(error_message)

        exc_config_file = service_request.exd_config_file
        logger.info("Preparing new simulation with {} config file".format(exc_config_file))

        gzserver_host = service_request.gzserver_host
        reservation = service_request.reservation
        sim_id = service_request.sim_id
        timeout_type = service_request.timeout_type
        timeout = self.__get_timeout(service_request.timeout, timeout_type)
        playback_path = service_request.playback_path
        token = service_request.token
        ctx_id = service_request.ctx_id
        experiment_id = service_request.experiment_id
        brain_processes = service_request.brain_processes

        sim_config = SimConfig(exc_config_file,
                               sim_id=sim_id,
                               gzserver_host=gzserver_host,
                               reservation=reservation,
                               timeout=timeout,
                               timeout_type=timeout_type,
                               playback_path=playback_path,
                               context_id=ctx_id,
                               token=token,
                               experiment_id=experiment_id,
                               brain_processes=brain_processes)

        # simulation playback
        if playback_path:
            assembly = PlaybackSimulationAssembly

        # single brain process launch
        elif sim_config.num_brain_processes == 1:
            if sim_config.simulation_type is SimulationType.NEST_SYNC:
                assembly = ServerConfigurations.SynchronousNestSimulation
            elif sim_config.simulation_type is SimulationType.SPINNAKER_SYNC:
                assembly = ServerConfigurations.SynchronousSpinnakerSimulation
            elif sim_config.simulation_type is SimulationType.NENGO_SYNC:
                assembly = ServerConfigurations.SynchronousNengoSimulation
            elif sim_config.simulation_type is SimulationType.ROBOT_ROS_SYNC:
                assembly = ServerConfigurations.SynchronousRobotRosNest
            elif sim_config.simulation_type is SimulationType.NEST_DIRECT_SYNC:
                assembly = ServerConfigurations.SynchronousDirectNestSimulation

        # distributed, multi-process launch (inline imports to avoid circular dependencies)
        else:
            # Nest distributed simulation
            if sim_config.simulation_type is SimulationType.NEST_DIST:
                logger.info("Creating distributed NestLauncher object")
                from hbp_nrp_distributed_nest.launch.NestLauncher import NestLauncher
                assembly = NestLauncher

        # Initializing the simulation
        try:
            # create and initialize the selected launcher
            launcher = assembly(sim_config)

            try:
                logger.info("Starting the experiment closed loop engine.")
                launcher.initialize(self.except_hook)
            # pylint: disable=broad-except
            except Exception:
                launcher.shutdown()
                raise

        # pylint: disable=broad-except
        except Exception:
            logger.exception("Initialization failed")
            print sys.exc_info()
            raise

        logger.info("Initialization done")

        self.running_simulation_thread = threading.Thread(target=self.__simulation,
                                                          args=(launcher, ))
        self.running_simulation_thread.daemon = True
        logger.info("Spawning new thread that will manage the experiment execution.")
        self.running_simulation_thread.start()

        return []

    @staticmethod
    def __get_timeout(timeout, timeout_type):
        """
        Gets the timeout for the simulation based on timeout_type
        (one of the flags defined in TimeoutType)

        :param timeout: A string denoting the timeout in seconds.
        :param timeout_type: The timeout type, one of the flags defined inTimeoutType.
        :return: The time when the simulation will end or None, if no timeout was specified
        """
        if timeout == "":
            return None
        elif timeout_type == TimeoutType.SIMULATION:
            return int(float(timeout))
        return datetime_parser.parse(timeout)

    def __simulation(self, launcher):
        """
        Main simulation method. Start the simulation from the given script file.

        :param: launcher: The assembled simulation
        """

        self.simulation_terminate_event.clear()

        # pylint: disable=broad-except
        try:
            launcher.run()  # This is a blocking call, not to be confused with
                            # threading.Thread.start
        except Exception, e:
            logger.error("Exception during simulation")
            logger.exception(e)
        # always attempt to shutdown cleanly before raising Exception
        finally:
            self.__is_running_simulation_terminating = True
            try:
                logger.info("Shutdown simulation")
                launcher.shutdown()
            except Exception, e:
                logger.error("Exception during shutdown")
                logger.exception(e)
            finally:
                self.running_simulation_thread = None
                self.__is_running_simulation_terminating = False
                self.simulation_terminate_event.set()
                gc.collect()  # NRRPLT-5374


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


def __except_hook(ex_type, value, ex_traceback):
    """
    Logs the unhandled exception

    :param ex_type: The exception type
    :param value: The exception value
    :param ex_traceback: The traceback
    """
    logger.critical("Unhandled exception of type {0}: {1}".format(ex_type, value))
    logger.exception(ex_traceback)


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

    sys.excepthook = __except_hook


def main():
    """
    Main function of ROSCLESimulationFactory
    """
    if os.environ["ROS_MASTER_URI"] == "":
        raise Exception("You should run ROS first.")

    signal.signal(signal.SIGUSR1, print_full_stack_trace)
    patch_signal()
    parser = argparse.ArgumentParser()
    parser.add_argument('--logfile', dest='logfile',
                        help='specify the CLE logfile')
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                        action="store_true")
    parser.add_argument("--vsdebug",
                        default=os.environ.get('CLE_DEBUG', None),
                        help="enable vscode debugging",
                        action="store_true")
    parser.add_argument('-p', '--pycharm',
                        dest='pycharm',
                        help='debug with pyCharm. IP adress and port are needed.',
                        nargs='+')
    args = parser.parse_args()

    if args.vsdebug:  # pragma: no cover
        import ptvsd
        ptvsd.enable_attach("my_secret", address=('0.0.0.0', 9992))

    if args.pycharm:  # pragma: no cover
        # pylint: disable=import-error
        import pydevd

        pydevd.settrace(args.pycharm[0],
                        port=int(args.pycharm[1]),
                        stdoutToServer=True,
                        stderrToServer=True,
                        suspend=False)

    server = ROSCLESimulationFactory()
    server.initialize()
    set_up_logger(args.logfile, args.verbose)
    server.run()
    logger.info("CLE Server exiting.")


if __name__ == '__main__':  # pragma: no cover
    main()
