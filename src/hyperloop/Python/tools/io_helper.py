import json
from pprint import pprint
import os
"""
Basics:
I chose JSON because:
a) hierarchy doens't make a whole lot of sense in CSV
b) XML is a pain to comprehend if one is unfamiliar with it
c) JSON is easily and quickly parsed, and can be edited in a easy-to-use viewer
    that displays everything in a simple tree viewer
"""
class InputHelper(object):

    def __init__(self, file_name):
        self.data = None
        parent_dir = os.path.dirname(os.path.dirname(__file__))
        abs_path = os.path.join(parent_dir, 'configs/' + file_name)
        with open(abs_path) as data_file:
            self.data = json.load(data_file)

    def get_config(self, member):
        return self.data[member]

# if __name__ == '__main__':
#     x = InputHelper('default.JSON')
#     pprint(x.get_config('tunnel_data'))