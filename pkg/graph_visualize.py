from graphviz import Digraph


class Node:
    def __init__(self, id, childs):
        self.id = id
        self.childs = childs
    
    def get_available_childs(self):
        return self.childs
        
        
class VisualGraph:
    def __init__(self, filename="graph.gv"):
        self.filename=filename
        self.node_dict = {}
        
    def add_node(self, node_id, child_names):
        if node_id in self.node_dict:
            self.node_dict[node_id].childs += child_names
        else:
            self.node_dict[node_id] = Node(node_id, child_names)
        
    def remove_node(self, node_id):
        del self.node_dict[node_id]
    
    def draw(self, unique=False):
        self.graph = Digraph('G', filename=self.filename)
        for node in self.node_dict.values():
            childs = node.get_available_childs()
            if unique:
                childs = list(set(childs))
            for child in childs:
                self.graph.edge(str(node.id), str(child))
        return self.graph
    