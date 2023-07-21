import pytest
from json import load


with open('test_setup.json') as json_file:
    setup = load(json_file)

    pytest.hostname = setup['hostname']
    pytest.password = setup['password']
    pytest.serial_port = setup['serial_port']

connections = list()

if pytest.hostname != '':
    connections.append(' tcp ')

if pytest.serial_port != '':
    connections.append(' serial ')


@pytest.fixture(scope='module', params=connections)
def nucleus_driver_connection(request):

    yield request.param
