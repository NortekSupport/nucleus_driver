import pytest
import sys

sys.path.append('../nucleus_node')

from nucleus_node import client_command


def test_client_command():

    response = client_command.main(args=['GETHW\r\n'])

    assert response.reply != b''