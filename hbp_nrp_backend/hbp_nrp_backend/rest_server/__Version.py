# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
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
This module contains the REST implementation
for retrieving the Closed Loop Engine and
the Experiment Designer Back-end versions
"""

__author__ = 'AxelVonArnim, LucGuyot'

from flask_restful import Resource, fields
from flask_restful_swagger import swagger
from hbp_nrp_backend.rest_server.RestSyncMiddleware import RestSyncMiddleware

import hbp_nrp_backend # pylint: disable=unused-import
import hbp_nrp_cle # pylint: disable=unused-import
import hbp_nrp_cleserver # pylint: disable=unused-import
import hbp_nrp_commons # pylint: disable=unused-import
import hbp_nrp_excontrol # pylint: disable=unused-import
import hbp_nrp_music_xml # pylint: disable=unused-import
import hbp_nrp_music_interface # pylint: disable=unused-import

# pylint: disable=R0201


class Version(Resource):
    """
    Implements the REST service providing the user with the versions
    of all NRP python packages.
    """

    @swagger.model
    class _Version(object):
        """
        Version object containing python package versions
        Only used for swagger documentation
        """

        resource_fields = {
            'hbp_nrp_cle': fields.String(),
            'hbp_nrp_backend': fields.String(),
            'hbp_nrp_cleserver': fields.String(),
            'hbp_nrp_commons': fields.String(),
            'hbp_nrp_excontrol': fields.String(),
            'hbp_nrp_music_xml': fields.String(),
            'hbp_nrp_music_interface': fields.String()
        }
        required = ['hbp_nrp_cle', 'hbp_nrp_backend', 'hbp_nrp_cleserver', 'hbp_nrp_commons',
                    'hbp_nrp_excontrol', 'hbp_nrp_music_xml', 'hbp_nrp_music_interface']

    @swagger.operation(
        notes='Gets the versions of all the backend Neurororobotics \
        python packages.'
        ' Possible values are: x.y.z, x.y.z.devW \
        where x,y,z and W are numbers',
        responseClass=_Version.__name__,
        responseMessages=[
            {
                "code": 200,
                "message": "Success. The versions were retrieved"
            }
        ]
    )
    @RestSyncMiddleware.threadsafe
    def get(self):
        """
        Returns the versions of all NRP python packages.

        :status 200: Success. The versions were retrieved
        """

        packages = ['hbp_nrp_cle', 'hbp_nrp_backend', 'hbp_nrp_cleserver', 'hbp_nrp_commons',
                    'hbp_nrp_excontrol', 'hbp_nrp_music_xml', 'hbp_nrp_music_interface']
        return {name: eval(name + ".__version__") for name in packages}, 200
