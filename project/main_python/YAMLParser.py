#!/usr/bin/env python
# _*_ encoding=utf-8 _*_

import yaml


class YAMLParser(object):

    def __init__(self, filename):
        self.file = filename
        self.yaml_file = open(self.file, "r")
        check = self.yaml_file.readline()
        print "yaml check" , check
        if ("YAML:1.0" in check ):
            read_yaml_file = self.yaml_file.read().replace('YAML:1.0', 'YAML 1.0')
            read_yaml_file = read_yaml_file.replace(':', ': ')
            read_yaml_file = read_yaml_file.replace('.Inf', '.nan')
            self.yaml_file.close()
            self.yaml_file = open(self.file, "w")
            self.yaml_file.write(read_yaml_file)
            self.yaml_file.close()
        exit(0)


    def close(self):
        if ( self.yaml_file != None ):
            self.yaml_file.close()

    def load(self):
        yaml_load = yaml.load(open(self.file))
        return yaml_load

