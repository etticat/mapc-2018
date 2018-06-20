import random

from provider.facility_provider import FacilityProvider


class ChooseStorageForHoarding(object):

    def __init__(self):
        self.facility_provider = FacilityProvider()

    def choose(self):
        # TODO: Look which storage has fewest of the ones we can offer
        # TODO: Take distance into account
        return random.choice(self.facility_provider.get_storages().values())