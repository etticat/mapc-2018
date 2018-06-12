import rospy

from behaviour_components.behaviours import BehaviourBase
from provider.product_provider import ProductProvider


class ChooseIngredientBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(ChooseIngredientBehaviour, self).__init__(**kwargs)
        self._product_provider = ProductProvider(agent_name=agent_name)

    def do_step(self):
        item_to_focus = None

        # Selecting the item that we need the most of
        # At least 1 of these itesm needs to fit into the stock
        for item, already_in_stock_items in self._product_provider.ingredient_priority():
            product = self._product_provider.get_product_by_name(item)
            load_after_gathering = self._product_provider.load_free - product.volume
            if load_after_gathering >= 0:
                item_to_focus = item
                break

        if item_to_focus is not None:
            self._product_provider.start_gathering(item_to_focus)
            rospy.logerr("ChooseIngredientBehaviour:: Chosing item %s", item_to_focus)
        else:
            rospy.logerr("ChooseIngredientBehaviour:: Trying to choose item, but none fit in stock")
