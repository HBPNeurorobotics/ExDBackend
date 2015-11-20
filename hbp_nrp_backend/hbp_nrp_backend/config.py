"""
Configuration file for databases
"""


class Config(object):
    """
    Base config
    SQLALCHEMY_DATABASE_URI is The database URI that should be used for the connection. Examples:
    mysql://username:password@server/db
    sqlite:////tmp/test.db
    More info can be found here: https://pythonhosted.org/Flask-SQLAlchemy/config.html
    """
    SQLALCHEMY_DATABASE_URI = 'postgresql://bbpdbsrv03.epfl.ch:5432/neurorobotics_collab'


class ConfigTest(Config):
    """
    Config for unit testing
    """
    SQLALCHEMY_DATABASE_URI = (
        'postgresql://neurorobotics_collab_test:WRITE_THE_TEST_USER_PASSWORD_HERE'
        '@bbpdbsrv03.epfl.ch:5432/neurorobotics_collab_test')


class ConfigStaging(Config):
    """
    Staging (and dev) database config
    """
    SQLALCHEMY_DATABASE_URI = (
        'postgresql://neurorobotics_collab:WRITE_THE_USER_PASSWORD_HERE'
        '@bbpdbsrv03.epfl.ch:5432/neurorobotics_collab')
