from abc import *
from ...utils.utils import *
from ..dash_launcher import IDENTIFY_COL, TAB_BUTTON

__metaclass__ = type

CONFIG_DIR = os.path.join(os.getcwd(), 'configs')
try: os.mkdir(CONFIG_DIR)
except: pass

class TableInterface:
    HEADS = None
    HILIGHT_KEY = None
    CUSTOM_BUTTONS = []

    def __init__(self, graph):
        self.graph = graph

    @abstractmethod
    def get_items(self):
        pass

    @abstractmethod
    def get_items_dict(self):
        pass

    @abstractmethod
    def add_item(self, value):
        pass

    @abstractmethod
    def delete_item(self, active_row):
        pass

    @abstractmethod
    def update_item(self, item, active_col, value):
        pass

    @abstractmethod
    def serialize(self, item):
        pass

    @abstractmethod
    def highlight_item(self, item, color=None):
        pass

    def select(self, selected_row_ids, active_row, active_col):
        self.graph.clear_highlight([self.HILIGHT_KEY])
        item_dict = self.get_items_dict()
        if active_row in item_dict:
            item = item_dict[active_row]
            self.highlight_item(item, color=(1, 0.3, 0.3, 0.5))
        for row in selected_row_ids:
            if row != active_row and row in item_dict:
                item = item_dict[row]
                self.highlight_item(item, color=(0.3, 0.3, 1, 0.5))


    def update(self, active_row, active_col, value, add=False, delete=False):
        res, msg = True, ""
        if add:
            try:
                self.add_item(value)
            except Exception as e:
                res, msg = False, str(e)
            return res, msg

        items_dict = self.get_items_dict()
        if active_row not in items_dict:
            return True, ""

        res, msg = True, ""

        if delete:
            try:
                self.delete_item(active_row)
            except Exception as e:
                res, msg = False, str(e)
        else:
            col_idx = self.HEADS.index(active_col)
            item = items_dict[active_row]
            val_cur = self.serialize(item)[col_idx]
            if val_cur != value:
                res, msg = self.update_item(item, active_col, value)

        return res, msg

    def button(self, button, filename=None, data=None):
        if button == TAB_BUTTON.SAVE:
            save_json(os.path.join(CONFIG_DIR, filename), data)
            pass
        elif button == TAB_BUTTON.LOAD:
            data_new = load_json(os.path.join(CONFIG_DIR, filename))
            dict_old = {dtem[IDENTIFY_COL]: dtem for dtem in data}
            dict_new = {dtem[IDENTIFY_COL]: dtem for dtem in data_new}
            names_old, names_new = set(dict_old.keys()), set(dict_new.keys())
            names_add, names_del = names_new-names_old, names_old-names_new
            names_update = names_new - names_add - names_del
            names_update, names_add, names_del = list(names_update), list(names_add), list(names_del)
            for name in names_del:
                print("del {}".format(name))
                self.update(name, None, None, delete=True)
            for name in names_add:
                print("add {}".format(name))
                self.update(None, None, dict_new[name], add=True)
            items_dict = self.get_items_dict()
            for name in names_update:
                tem_new = dict_new[name]
                arr_old = self.serialize(items_dict[name])
                for v_old, col in zip(arr_old, self.HEADS):
                    v_new = tem_new[col]
                    if v_old != v_new:
                        print("update {} - {} : {} -> {}".format(name, col, v_old, v_new))
                        self.update(name, col, v_new)

    def get_table(self):
        return (self.HEADS, [self.serialize(item) for item in self.get_items()])