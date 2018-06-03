

class CalcUtil:

    @staticmethod
    def dict_sum(dict1, dict2):
        return {k: dict1.get(k, 0) + dict2.get(k, 0) for k in set(dict1) | set(dict2)}