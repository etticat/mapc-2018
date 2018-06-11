from pip._vendor.requests.packages.urllib3.connectionpool import xrange


class TaskManager(object):


    def request_task_bids(self):
        """
        The first step of the protocol
        :return:
        """
        request_object = {
            "destination": "storage5",
            "items": {
                "item6": 1,
                "item7": 2,
                "item8": 2
            }
        }

    def process_bids(self, bids):
        """

        :param bids:
        :type bids: dict[]
        :return:
        """
        max_bid = 0
        combination = []

        # combine all bids
        for i in xrange(len(bids)):
            for j in xrange(len(bids)):
                for k in xrange(len(bids)):
                    for l in xrange(len(bids)):
                        current_combination = [i,j,k,l]
                        current_bid = self.calculate_task_team_value([i, j, k, l])

                        if current_bid > max_bid:
                            combination = current_combination
                            max_bid = current_bid
        self.assign_bids(bids, combination)


    def calculate_task_team_value(self, bids):
        combined_inredients = {} # TODO add up all items from the bids
        best_possible_product_values = {} # TODO calculate the best possible combination of finished products, that can be generated from the ingredients

        bset_prodcut_worth = 12

        # Take following items into account
        return bset_prodcut_worth + bids.min_duration

    def assign_bids(self, bids, combination):
        """
        Assignes tasks to assemble all items from the combinationarray
        :param bids:
        :param combination:
        :return:
        """

    def process_bid_acknoledgements(self, acknoledgements):
        for acknoledgement in acknoledgements:
            if acknoledgement.accepted == False
                self.cancel_assembly_task()

    def cancel_assembly_task(self):
        pass


