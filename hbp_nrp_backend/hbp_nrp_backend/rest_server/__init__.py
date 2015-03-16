"""
This package contains the implementation of the REST server to control experiments
"""

__author__ = 'GeorgHinkel'


from flask import Flask
from flask_restful import Api
from flask_restful_swagger import swagger
from hbp_nrp_cle.cle.ROSCLEClient import ROSCLEClientException


class NRPServicesGeneralException(Exception):
    """
    General exception class that can be used to return meaningful messages
    to the HBP frontend code.

    :param message: message displayed to the end user.
    :param error_type: Type of error (like 'CLE Error')
    """
    def __init__(self, message, error_type):
        super(NRPServicesGeneralException, self).__init__(message)
        # This field is handled by the error handling HBP frontend code.
        self.error_type = error_type

    def __str__(self):
        return "" + repr(self.message) + " (" + self.error_type + ")"


class NRPServicesExtendedApi(Api):
    """
    Extend Flask Restful error handling mechanism so that we
    can still use original Flask error handlers (defined in
    __ErrorHandlers.py)
    """
    def error_router(self, original_handler, e):
        """
        Route the error

        :param original_handler: Flask handler
        :param e: Error
        """
        return original_handler(e)

app = Flask(__name__)
api = swagger.docs(NRPServicesExtendedApi(app), apiVersion='0.1')

# Import REST APIs
# pylint: disable=W0401
import hbp_nrp_backend.rest_server.__ErrorHandlers
from hbp_nrp_backend.rest_server.__SimulationService import SimulationService
from hbp_nrp_backend.rest_server.__SimulationState import SimulationState
from hbp_nrp_backend.rest_server.__SimulationControl import SimulationControl, LightControl, \
    CustomEventControl

api.add_resource(SimulationService, '/simulation')
api.add_resource(SimulationControl, '/simulation/<int:sim_id>')
api.add_resource(SimulationState, '/simulation/<int:sim_id>/state')
api.add_resource(CustomEventControl, '/simulation/<int:sim_id>/interaction')
api.add_resource(LightControl, '/simulation/<int:sim_id>/interaction/light')
