"""
This module contains helper functionality to convert a CLE Transfer Function into a structured
format used to pass transfer functions via ROS
"""

from cle_ros_msgs.msg import TransferFunction, Device, Topic, Variable, ExperimentPopulationInfo
from hbp_nrp_cle.tf_framework import Neuron2Robot, Robot2Neuron, NeuronMonitor, MapRobotPublisher, \
    poisson, population_rate, leaky_integrator_alpha, leaky_integrator_exp, fixed_frequency,\
    nc_source, dc_source, ac_source, spike_recorder
import logging
import textwrap

__author__ = "Georg Hinkel"

Neuron2Robot.structured_type = 2
Robot2Neuron.structured_type = 1
NeuronMonitor.structured_type = 3

device_types = {
    ac_source: "ACSource",
    dc_source: "DCSource",
    fixed_frequency: "FixedFrequency",
    leaky_integrator_alpha: "LeakyIntegratorAlpha",
    leaky_integrator_exp: "LeakyIntegratorExp",
    nc_source: "NCSource",
    poisson: "Poisson",
    spike_recorder: "SpikeRecorder",
    population_rate: "PopulationRate"
}

device_type_infos = {
    "ACSource": ("nrp.ac_source", False),
    "DCSource": ("nrp.dc_source", False),
    "FixedFrequency": ("nrp.fixed_frequency", False),
    "LeakyIntegratorAlpha": ("nrp.leaky_integrator_alpha", True),
    "LeakyIntegratorExp": ("nrp.leaky_integrator_exp", True),
    "NCSource": ("nrp.nc_source", False),
    "Poisson": ("nrp.poisson", False),
    "SpikeRecorder": ("nrp.spike_recorder", True),
    "PopulationRate": ("nrp.population_rate", True)
}


logger = logging.getLogger(__name__)
indentation = ' ' * 4


def indent(code):
    """
    Indents the given code with four spaces

    :param code: The given code
    :return: An indented string
    """
    return indentation + ('\n' + indentation).join(code.split('\n'))


def __get_spec(device):
    """
    Gets the specification of a given parameter device

    :param device: The parameter device
    :return: the specification for teh given parameter
    """
    if hasattr(device, 'spec'):
        return device.spec
    else:
        return device


def __get_specs(tf):
    """
    Gets all parameter specifications of the given transfer function

    :param tf: The transfer function
    :return: The parameter specifications
    """
    return map(__get_spec, tf.params[1:])


def __extract_neurons(neurons):
    """
    Extracts a population specification from the given property path

    :param neurons: The given neurons
    :return: A population specification
    """
    index = None
    if hasattr(neurons, "index"):
        index = neurons.index
        neurons = neurons.parent
    name = neurons.name
    if index is None:
        return ExperimentPopulationInfo(name=name,
                                        type=ExperimentPopulationInfo.TYPE_ENTIRE_POPULATION,
                                        ids=[], start=0, stop=0, step=0)
    if isinstance(index, slice):
        return ExperimentPopulationInfo(name=name,
                                        type=ExperimentPopulationInfo.TYPE_POPULATION_SLICE,
                                        ids=[], start=index.start, stop=index.stop,
                                        step=index.step or 1)
    if isinstance(index, int):
        return ExperimentPopulationInfo(name=name,
                                        type=ExperimentPopulationInfo.TYPE_POPULATION_SLICE,
                                        ids=[], start=index, stop=index + 1, step=1)
    if isinstance(index, list):
        return ExperimentPopulationInfo(name=name,
                                        type=ExperimentPopulationInfo.TYPE_POPULATION_LISTVIEW,
                                        ids=index or [], start=0, stop=0, step=0)
    raise Exception("Could not parse neurons " + repr(neurons))


def __generate_neurons(neurons):
    """
    Generates the code for a structured neuron information

    :param neurons: The population info
    :return: The code to be generated
    """
    if neurons.type == ExperimentPopulationInfo.TYPE_ENTIRE_POPULATION:
        return "nrp.brain.{0}".format(neurons.name)
    elif neurons.type == ExperimentPopulationInfo.TYPE_POPULATION_SLICE:
        if neurons.stop == neurons.start + 1:
            return "nrp.brain.{0}[{1}]".format(neurons.name, neurons.start)
        else:
            return "nrp.brain.{0}[slice({1},{2},{3})]"\
                .format(neurons.name, neurons.start, neurons.stop, neurons.step)
    elif neurons.type == ExperimentPopulationInfo.TYPE_POPULATION_LISTVIEW:
        return "nrp.brain.{0}[[{1}]]".format(neurons.name, ", ".join(neurons.ids))


def __extract_devices(tf):
    """
    Converts the specification of the devices from the given transfer function

    :param tf: The transfer function
    :return: A list of device specifications
    """
    return [
        Device(
            name=d.name,
            type=device_types[d.device_type],
            neurons=__extract_neurons(d.neurons)
        )
        for d in __get_specs(tf) if d.is_brain_connection
    ]


def __generate_device(device):
    """
    Generates the code for a neuron device

    :param device:
    :return:
    """
    info = device_type_infos[device.type]
    if info[1]: # device is spike sink
        return '@nrp.MapSpikeSink("{0}", {1}, {2})\n'\
            .format(device.name, __generate_neurons(device.neurons), info[0])
    else:
        return '@nrp.MapSpikeSource("{0}", {1}, {2})\n'\
            .format(device.name, __generate_neurons(device.neurons), info[0])


def __extract_type_name(topic_type):
    """
    Extracts the type name of a given ROS message type

    :param topic_type: The message type
    """
    name = topic_type.__name__
    module = topic_type.__module__
    if module.endswith(name) and '.' in module:
        module = module[:module.rindex('.')]
    return module + "." + name


def __extract_topics(tf):
    """
    Converts the topics of the given transfer function

    :param tf: The transfer function
    :return: A list of converted topic specifications
    """
    topics = [
        Topic(
            name=t.name,
            topic=t.topic.name,
            type=__extract_type_name(t.topic.topic_type),
            publishing=isinstance(t, MapRobotPublisher)
        )
        for t in __get_specs(tf) if t.is_robot_connection
        ]
    if hasattr(tf, 'topic'):
        return_topic = __get_spec(tf.topic)
        if return_topic is not None:
            topics.append(Topic(
                name='__return__',
                topic=return_topic.name,
                type=__extract_type_name(return_topic.topic_type),
                publishing=True
            ))
    return topics


def __extract_variables(tf):
    """
    Converts the variables of the given transfer function

    :param tf: The transfer function
    :return: A list of converted variables
    """
    return [
        Variable(
            name=v.name,
            type=type(v.initial_value).__name__,
            initial_value=str(v.initial_value)
        )
        for v in __get_specs(tf) if not v.is_robot_connection and not v.is_brain_connection
    ]


def __get_tf_code(tf):
    """
    Extracts the body from the given in-memory transfer function

    :param tf: The transfer function
    :return: The extracted body of the transfer function
    """
    source = tf.source.splitlines()
    start = None
    stop = None
    for i in range(0, len(source)):
        if start is None:
            if source[i].startswith("def " + tf.name):
                start = i
        else:
            if not source[i].startswith(" ") and not source[i] == "":
                stop = i
                break
    if start is None:
        return tf.source
    if stop is None:
        stop = len(source)
    return textwrap.dedent("\n".join(source[start + 1:stop]))


def extract_structure(transfer_function):
    """
    Converts the given transfer function into a structured format

    :param transfer_function: The transfer function
    :return: A structured format that can be sent via ROS
    """
    try:
        return TransferFunction(
            name=transfer_function.name,
            code=__get_tf_code(transfer_function),
            type=type(transfer_function).structured_type,
            devices=__extract_devices(transfer_function),
            topics=__extract_topics(transfer_function),
            variables=__extract_variables(transfer_function)
        )
    # pylint: disable=broad-except
    except Exception as e:
        logger.exception(e)
        return None


def __get_initial_value_string(var):
    """
    Gets the value string for the initial value for the given variable

    :param var: The variable
    """
    if var.type == "str":
        return '"' + str(var.initial_value) + '"'
    else:
        return str(var.initial_value)


def generate_code_from_structured_tf(transfer_function):
    """
    Generates PyTF-code from the given transfer function in a structured form

    :param transfer_function:
    :return: The code that represents the transfer function
    """
    devices = ""
    code = ""
    return_topic = None
    monitor_device = None
    for dev in transfer_function.devices:
        if transfer_function.type == NeuronMonitor.structured_type and dev.name == "device":
            monitor_device = dev
        else:
            devices += ", " + dev.name
            code += __generate_device(dev)
    for topic in transfer_function.topics:
        if topic.name == "__return__":
            return_topic = topic
        elif transfer_function.type == NeuronMonitor.structured_type and topic.name == "publisher":
            # This topic is inferred by the NeuronMonitor and therefore ignored
            pass
        else:
            devices += ", " + topic.name
            code += __generate_topic(topic)
    for var in transfer_function.variables:
        devices += ", " + var.name
        code += '@nrp.MapVariable("{0}", initial_value={1})\n'\
            .format(var.name, __get_initial_value_string(var))
    code += __generate_transfer_function_annotation(transfer_function, monitor_device, return_topic)
    code += "def {0}(t{1}):\n".format(transfer_function.name, devices)
    code += indent(transfer_function.code)
    return code


def __generate_topic(topic):
    """
    Generate the code for the given topic

    :param topic: The given topic
    """
    if topic.publishing:
        return '@nrp.MapRobotPublisher("{0}", Topic("{1}", {2}))\n' \
            .format(topic.name, topic.topic, topic.type)
    else:
        return '@nrp.MapRobotSubscriber("{0}", Topic("{1}", {2}))\n' \
            .format(topic.name, topic.topic, topic.type)


def __generate_transfer_function_annotation(transfer_function, monitor_device, return_topic):
    """
    Generate the main transfer function annotation

    :param monitor_device: The monitor device (only required for neuron monitors)
    :param return_topic: The return topic (only applicable for Neuron2Robot)
    :param transfer_function:  The transfer function
    """
    if transfer_function.type == Neuron2Robot.structured_type:
        if return_topic is None:
            return "@nrp.Neuron2Robot()\n"
        else:
            return '@nrp.Neuron2Robot(Topic("{0}", {1}))\n' \
                .format(return_topic.topic, return_topic.type)
    elif transfer_function.type == Robot2Neuron.structured_type:
        return "@nrp.Robot2Neuron()\n"
    else:
        monitor_info = device_type_infos[monitor_device.type]
        return "@nrp.NeuronMonitor({0}, {1})\n" \
            .format(__generate_neurons(monitor_device.neurons), monitor_info[0])
