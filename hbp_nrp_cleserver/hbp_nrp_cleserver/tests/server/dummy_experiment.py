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
This is a dummy experiment script used for testing
"""

__author__ = 'georghinkel'

import hbp_nrp_cleserver.tests.server.dummy_experiment_validation as val
from mock import Mock

def cle_function_init(environment_file):
    val.experiment_cle_init_called = True
    return [Mock(), "foobar", Mock(), Mock()]

def shutdown(cle_server, models_path, gzweb, gzserver):
    val.experiment_shutdown_called = True