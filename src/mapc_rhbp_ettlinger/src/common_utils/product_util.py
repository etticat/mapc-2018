

class ProductUtil(object):


    @staticmethod
    def get_items_and_roles_from_bids(subset):
        item_dict = {}
        roles = []
        for bid in subset:
            for item in bid.items:
                item_dict[item] = item_dict.get(item, 0) + 1
            roles.append(bid.role)
        return item_dict, roles