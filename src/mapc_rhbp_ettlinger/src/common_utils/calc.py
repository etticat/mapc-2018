import copy

from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import Item
from mapc_rhbp_ettlinger.msg import KeyIntValue


class CalcUtil:

    @staticmethod
    def dict_sum(dict1, dict2):
        return {k: dict1.get(k, 0) + dict2.get(k, 0) for k in set(dict1) | set(dict2)}

    @staticmethod
    def dict_diff(dict1, dict2, normalize_to_zero=False):
        if normalize_to_zero:
            return {k: max(dict1.get(k, 0) - dict2.get(k, 0), 0) for k in set(dict1) | set(dict2)}
        else:
            return {k: dict1.get(k, 0) - dict2.get(k, 0) for k in set(dict1) | set(dict2)}

    @staticmethod
    def dict_max_diff(finished_product_dict, ingredient_dict):

        max_nr = 999

        for k in finished_product_dict.keys():
            finished_product_required_ingredients = finished_product_dict.get(k, 0)
            nr_possible = ingredient_dict.get(k, 0) / finished_product_required_ingredients

            if nr_possible < max_nr:
                max_nr = nr_possible

        return max_nr
    @staticmethod
    def items_intersect(items1, items2):
        res = []

        for item1 in items1:
            for item2 in items2:
                if item1.name == item2.name:
                    res.append(Item(name=item1.name,amount=min(item1.amount, item2.amount)))
                    break
        return res
    @staticmethod
    def list_intersect(a, b):
        res = []
        for u in set(a):
            for k in range(min(a.count(u), b.count(u))):
                res.append(u)

        return res

    @staticmethod
    def list_diff(a, b):
        a = copy.copy(a)
        for item in b:
            if item in a:
                a.remove(item)
        return a

    @staticmethod
    def get_dict_from_items(items, attrs=None):

        if attrs is None:
            attrs = ["amount"]

        res = {}
        for item in items:
            for attr in attrs:
                res[item.name] = res.get(item.name, 0) + getattr(item, attr)
        return res
    @staticmethod
    def get_list_from_items(items):
        res = []
        for item in items:
            for i in range(item.amount):
                res.append(item.name)
        return res

    @classmethod
    def contains_items(cls, container, items):
        for key in items.keys():
            if container.get(key, 0) < items[key]:
                return False

        return True

    @classmethod
    def get_list_from_dict(cls, items_to_pickup):
        res = []
        for item, count in items_to_pickup.iteritems():
            for i in range(count):
                res += [item]
        return res

    @classmethod
    def dict_from_strings(cls, items):
        res = {}
        for item in items:
            res[item] = res.get(item, 0) + 1

        return res

    @classmethod
    def dict_intersect(cls, dict1, dict2):
        return {k: min(dict1.get(k, 0), dict2.get(k, 0)) for k in set(dict1) | set(dict2)}

    @classmethod
    def dict_from_key_int_values(cls, amounts):
        """

        :param amounts:
        :type amounts: KeyIntValue[]
        :return:
        """
        res = {}

        for key_values in amounts:
            res[key_values.key] = key_values.value

        return res

    @classmethod
    def key_int_values_from_dict(cls, dict_inst):
        res = []

        for key, value in dict_inst.iteritems():
            res.append(KeyIntValue(key=key,value=value))

        return res

    @classmethod
    def negate_dict(cls, item_dict):
        return {key: {key_: -val_ for key_, val_ in val.items()}
         for key, val in item_dict.iteritems()}