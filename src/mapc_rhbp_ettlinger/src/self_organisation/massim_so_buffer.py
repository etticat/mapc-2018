from provider.simulation_provider import SimulationProvider
from so_data.sobuffer import SoBuffer


class MassimSoBuffer(SoBuffer):
    """
    SoBuffer implementation, which injects the massim step as time instead of using the rospy time
    """

    def __init__(self, agent_name, aggregation=None, aggregation_distance=1.0,
                 min_diffusion=0.1, view_distance=1.5, id='',
                 moving_storage_size=2, store_all=True, framestorage=None,
                 pose_frame='robot', ev_thread=False, ev_time=1):

        super(MassimSoBuffer, self).__init__(aggregation=aggregation, aggregation_distance=aggregation_distance, min_diffusion=min_diffusion,
                                             view_distance=view_distance, id=id, moving_storage_size=moving_storage_size, store_all=store_all,
                                             framestorage=framestorage, pose_frame=pose_frame, ev_thread=ev_thread, ev_time=ev_time)
        self.simulation_provider = SimulationProvider(agent_name=agent_name)

    def get_current_time(self):
        """
        Use massim step instead of rospy time
        :return:
        """
        return self.simulation_provider.step

