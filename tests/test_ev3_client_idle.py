import json
import socket
import unittest
from robot.ev3.server.ev3_server import Ev3JsonServer
from tests.test_ev3_server import FakeClock, make_controller

class Connection:
    def __init__(self, clock, events):
        self.clock=clock;self.events=iter(events);self.closed=False;self.sent=[]
    def settimeout(self,value):self.timeout=value
    def recv(self,size):
        seconds,data=next(self.events)
        self.clock.advance(seconds)
        if data is None:raise socket.timeout()
        return data
    def sendall(self,data):self.sent.append(json.loads(data))
    def close(self):self.closed=True

class IdleClientTests(unittest.TestCase):
    def setUp(self):
        self.clock=FakeClock();self.controller,self.motors=make_controller(clock=self.clock)
        self.server=Ev3JsonServer(self.controller,clock=self.clock)
    def test_silent_old_connection_closes_and_new_bridge_can_read(self):
        old=Connection(self.clock,[(1,None)]*5)
        self.server._handle_client(old)
        self.assertTrue(old.closed)
        self.assertEqual('client-disconnect',self.controller.last_stop_reason)
        new=Connection(self.clock,[(0,b'{"command":"status"}\n'),(0,b'')])
        self.server._handle_client(new)
        self.assertEqual('ok',new.sent[0]['status'])
    def test_periodic_status_keeps_connection_alive_beyond_timeout(self):
        c=Connection(self.clock,[(2,b'{"command":"status"}\n')]*6+[(0,b'')])
        self.server._handle_client(c)
        self.assertEqual(6,len(c.sent))
        self.assertTrue(c.closed)
    def test_partial_bytes_do_not_hold_the_only_connection_forever(self):
        c=Connection(self.clock,[(1,b'{')]*5)
        self.server._handle_client(c)
        self.assertTrue(c.closed)
        self.assertEqual([],c.sent)

if __name__=='__main__':unittest.main()
