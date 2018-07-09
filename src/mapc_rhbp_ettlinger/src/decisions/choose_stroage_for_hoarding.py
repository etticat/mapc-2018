import random

from provider.facility_provider import FacilityProvider
from so_data.patterns import DecisionPattern


class ChooseStorageMechanism(DecisionPattern):

    def __init__(self, agent_name):

        self.facility_provider = FacilityProvider()

        super(ChooseStorageMechanism, self).__init__(buffer=None, frame=None, requres_pos=False)

    def calc_value(self):
        if self.value is None:
            return [random.choice(self.facility_provider.get_storages().values()), self.state]
        return [self.value, self.state]