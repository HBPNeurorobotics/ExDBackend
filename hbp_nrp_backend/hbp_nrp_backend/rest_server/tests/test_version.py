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
Unit tests for the service that retrieves python versions
"""

__author__ = 'AxelVonArnim, LucGuyot'

import hbp_nrp_backend
import hbp_nrp_cle
import hbp_nrp_cleserver
import hbp_nrp_commons
import hbp_nrp_excontrol

import unittest
import json
from mock import patch, MagicMock
from hbp_nrp_backend.rest_server.tests import RestTest


class TestVersion(RestTest):

    def test_version_get(self):
        response = self.client.get('/version')
        self.assertEqual(response.status_code, 200)
        expected_response = {'hbp_nrp_cle': str(hbp_nrp_cle.__version__), 'hbp_nrp_backend': str(hbp_nrp_backend.__version__),
                             'hbp_nrp_cleserver': str(hbp_nrp_cleserver.__version__), 'hbp_nrp_commons': str(hbp_nrp_commons.__version__),
                             'hbp_nrp_excontrol': str(hbp_nrp_excontrol.__version__)
                            }
        erd = json.dumps(expected_response)
        self.assertEqual(response.data.strip(), erd)

if __name__ == '__main__':
    unittest.main()
