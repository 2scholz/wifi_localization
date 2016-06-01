#!/usr/bin/env python

class Wifi:
    def __init__(self, mac=None, ssid=None, strength=None):
        self.mac = mac
        self.ssid = ssid
        self.strength = strength

    def __eq__(self, other):
        if not isinstance(other, Wifi):
            return False
        if self.mac != other.mac:
            return False
        if self.ssid != other.ssid:
            return False

    def __hash__(self):
        return "%s_%s" % (self.mac, self.ssid)

    def __str__(self):
        return "%s\t:\t%s\t@\t%s" % (self.mac, self.ssid, self.strength)

    def __repr__(self):
        return str(self)

