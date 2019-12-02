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
Unit tests for the sim config model
"""

import unittest
import os
from mock import patch, Mock, MagicMock, ANY, mock_open
from hbp_nrp_commons.MockUtil import MockUtil

from hbp_nrp_commons.sim_config.SimConfig import SimConfig, _TF

__author__ = 'Hossain Mahmud'

_base_path = 'hbp_nrp_commons.sim_config.SimConfig.'


@patch(_base_path + 'logging', new=MagicMock())
class TestSimConfig(unittest.TestCase):
    def setUp(self):
        # mock os
        self.m_os = MockUtil.fakeit(self, _base_path + 'os')
        # restore os.path.join and os.path.dirname
        self.m_os.path.join = os.path.join
        self.m_os.path.dirname = os.path.dirname

        self.m_robot = MockUtil.fakeit(self, _base_path + 'Robot')
        self.m_exc_parser = MockUtil.fakeit(self, _base_path + 'exc_parser')
        self.m_bibi_parser = MockUtil.fakeit(self, _base_path + 'bibi_parser')

        self.m_sconf_util = MockUtil.fakeit(self, _base_path + 'SimConfUtil')
        self.m_gentf = MockUtil.fakeit(self, _base_path + 'generate_tf')
        self.m_gettfname = MockUtil.fakeit(self, _base_path + 'get_tf_name')

        self.m_exc = MagicMock()
        self.m_exc_parser.CreateFromDocument.return_value = self.m_exc
        self.m_bibi = MagicMock()
        self.m_bibi_parser.CreateFromDocument.return_value = self.m_bibi

    def tearDown(self):
        pass

    def test_parsing(self):
        # create test object
        self.exc_abs_path = '/my/experiment/config.exc'
        self.exc_text = '''exc'''
        self.bibi_text = '''bibi'''

        m_exc_file = MagicMock()
        m_exc_file.read.return_value = self.exc_text

        m_bibi_file = MagicMock()
        m_bibi_file.read.return_value = self.bibi_text

        files = {
            '/my/experiment/config.exc': m_exc_file,
            '/my/experiment/config.bibi': m_bibi_file
        }

        # monkey patch internal function to limit call stack
        SimConfig._read_dom_data = lambda _self: True

        mopen = MagicMock(side_effect=lambda v: files[v])

        exc = self.m_exc_parser.CreateFromDocument.return_value
        exc.bibiConf.src = 'config.bibi'

        # test normal execution
        with patch("__builtin__.open", mopen):
            self.sim_config = SimConfig(self.exc_abs_path)

        self.m_exc_parser.CreateFromDocument.called_once_with(self.exc_text)
        self.assertEqual(self.sim_config._bibi_abs_path, '/my/experiment/config.bibi')
        self.m_bibi_parser.CreateFromDocument.called_once_with(self.bibi_text)

        # test exceptions
        from pyxb import ValidationError, NamespaceError
        self.m_exc_parser.CreateFromDocument.side_effect = ValidationError
        with patch("__builtin__.open", mopen):
            self.assertRaises(Exception, SimConfig, self.exc_abs_path)

        self.m_exc_parser.CreateFromDocument.side_effect = None
        self.m_bibi_parser.CreateFromDocument.side_effect = ValidationError
        with patch("__builtin__.open", mopen):
            self.assertRaises(Exception, SimConfig, self.exc_abs_path)

        self.m_bibi_parser.CreateFromDocument.side_effect = NamespaceError
        with patch("__builtin__.open", mopen):
            self.assertRaises(Exception, SimConfig, self.exc_abs_path)

    def test_read_data(self):
        pass

    def test_properties(self):
        pass

    def test_priority_is_int(self):
        # test _TF priority value is always initialized to an integer
        tf_1 = _TF("dummy_name", "dummy_code",priority=None)
        tf_2 = _TF("dummy_name", "dummy_code", priority="string_value")
        tf_3 = _TF("dummy_name", "dummy_code", priority=False)
        tf_4 = _TF("dummy_name", "dummy_code", priority=3.1416)

        self.assertTrue(type(tf_1.priority) == int)
        self.assertTrue(type(tf_2.priority) == int)
        self.assertTrue(type(tf_3.priority) == int)
        self.assertTrue(type(tf_4.priority) == int)



if __name__ == '__main__':
    unittest.main()
