import copy
import re

from mapc_rhbp_ettlinger.msg import KeyIntValue


class CalcUtil(object):

    @staticmethod
    def dict_sum(dict1, dict2):
        """
        Returns a dictionary with all values of the input dictionaries summed up
        :param dict1: First dictionary
        :type dict1: dict
        :param dict2: Second dictionary
        :type dict2: dict
        :return: dict
        """
        return {k: dict1.get(k, 0) + dict2.get(k, 0) for k in set(dict1) | set(dict2)}

    @staticmethod
    def dict_diff(dict1, dict2, normalize_to_zero=False):
        """
        Returns a dictionary with all values of the second dictionary subtracted from the first
        :param dict1: First dictionary
        :type dict1: dict
        :param dict2: Second dictionary
        :type dict2: dict
        :param normalize_to_zero: determines if the result should have minus values or if they should be normalised to 0
        :type normalize_to_zero: bool
        :return: dict
        """
        if normalize_to_zero:
            return {k: max(dict1.get(k, 0) - dict2.get(k, 0), 0) for k in set(dict1) | set(dict2)}
        else:
            return {k: dict1.get(k, 0) - dict2.get(k, 0) for k in set(dict1) | set(dict2)}

    @staticmethod
    def list_intersect(list1, list2):
        """
        Returns a list of items that are in both lists
        :param list1: list
        :param list2: list
        :return: list
        """
        res = []
        for u in set(list1):
            for k in range(min(list1.count(u), list2.count(u))):
                res.append(u)

        return res

    @staticmethod
    def list_diff(list1, list2):
        """
        Subtracts all items of list2 from a copy of list1
        :param list1: list
        :param list2: list
        :return: list
        """
        list1 = copy.copy(list1)
        for item in list2:
            if item in list1:
                list1.remove(item)
        return list1

    @staticmethod
    def get_dict_from_items(items, attributes=None):
        """
        Turns a dictionary into item objects
        :param items: items
        :type items: Item[]
        :param attributes: The attributes of the items to consider. by default this is only amount
        :type attributes: list
        :return: dict
        """
        if attributes is None:
            attributes = ["amount"]

        res = {}
        for item in items:
            for attr in attributes:
                res[item.name] = res.get(item.name, 0) + getattr(item, attr)
        return res

    @staticmethod
    def get_list_from_items(items):
        """
        Returns a string containing item names from a list of items
        :param items: Item[]
        :return: list
        """
        res = []
        for item in items:
            for i in range(item.amount):
                res.append(item.name)
        return res

    @classmethod
    def contains_items(cls, container, items):
        """
        Checks if dict is contained in a container dict
        :param container:
        :type container: dict
        :param items:
        :type items: dict
        :return:
        """
        for key in items.keys():
            if container.get(key, 0) < items[key]:
                return False

        return True

    @classmethod
    def get_string_list_from_dict(cls, items_to_pickup):
        """
        returns a list of item strings of a dict
        :param items_to_pickup:
        :param items_to_pickup: dict
        :return: list
        """
        res = []
        for item, count in items_to_pickup.iteritems():
            for i in range(count):
                res += [item]
        return res

    @classmethod
    def dict_from_string_list(cls, items):
        """
        Returns a dictionary made from a list of strings. Values describe the number of occurrences in the list
        :param items:
        :return:
        """
        res = {}
        for item in items:
            res[item] = res.get(item, 0) + 1

        return res

    @classmethod
    def dict_intersect(cls, dict1, dict2):
        """
        Returns a dictionary, describing where the two input dictionaries intersect
        :param dict1: dict
        :param dict2: dict
        :return: dict
        """
        return {k: min(dict1.get(k, 0), dict2.get(k, 0)) for k in set(dict1) | set(dict2)}

    @classmethod
    def dict_from_key_int_values(cls, amounts):
        """
        Converts a KeyIntValue array into a dictionary
        :param amounts:
        :type amounts: KeyIntValue[]
        :return: dict
        """
        res = {}

        for key_values in amounts:
            res[key_values.key] = key_values.value

        return res

    @classmethod
    def key_int_values_from_dict(cls, dict_inst):
        """
        Converts a dictionary into a KeyIntValue array
        :param dict_inst: dict
        :return: KeyIntValue[]
        """
        res = []

        for key, value in dict_inst.iteritems():
            res.append(KeyIntValue(key=key, value=value))

        return res

    @classmethod
    def negate_dict(cls, input_dict):
        """
        Returns an array with all values of input dict negated
        :param input_dict: dict
        :return: dict
        """
        res = {}
        for key_, val_ in input_dict.iteritems():
            res[key_] = -val_
        return res

    @staticmethod
    def atoi(text):
        """
        Converts a text into int if it only contains digits
        :param text: str
        :return:
        """
        return int(text) if text.isdigit() else text

    @staticmethod
    def natural_keys(text):
        """
        Returns parts of the text separated by values that are numbers and those, that are not
        alist.sort(key=natural_keys) sorts in human order
        http://nedbatchelder.com/blog/200712/human_sorting.html
        (See Toothy's implementation in the comments)
        Taken from https://stackoverflow.com/questions/5967500/how-to-correctly-sort-a-string-with-a-number-inside
        :param text:
        :return:
        """
        return [CalcUtil.atoi(c) for c in re.split('(\d+)', text)]

    @classmethod
    def multiply_dict_by_factor(cls, dict_, factor):
        res = {}
        for key in dict_:
            res[key] = dict_[key] * factor
        return res
