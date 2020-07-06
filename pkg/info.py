   
class LinkInfo:
    def __init__(self, Toff, axis, ltype, lname, parent):
        self.Toff, self.axis, self.ltype, self.lname, self.parent = \
            Toff, axis, ltype, lname, parent
    
    def get_tuple(self):
        return self.Toff, self.axis, self.ltype, self.lname, self.parent
    

class RobotInfo:
    def __init__(self, link_info_list, rname, base_name):
        self.link_info_list, self.rname, self.base_name = \
            link_info_list, rname, base_name