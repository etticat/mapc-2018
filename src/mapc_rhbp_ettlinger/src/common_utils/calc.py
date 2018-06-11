

class CalcUtil:

    @staticmethod
    def dict_sum(dict1, dict2):
        return {k: dict1.get(k, 0) + dict2.get(k, 0) for k in set(dict1) | set(dict2)}

    @staticmethod
    def dict_diff(dict1, dict2):
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