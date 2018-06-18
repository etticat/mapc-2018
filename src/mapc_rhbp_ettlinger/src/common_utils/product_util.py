

class ProductUtil(object):


    @staticmethod
    def get_items_and_roles_from_bids(subset):
        item_dict = {}
        roles = []
        for bid in subset:
            for item in bid.items:
                item_dict[item.name] = item_dict.get(item.name, 0) + item.amount
            roles.append(bid.role)
        return item_dict, roles