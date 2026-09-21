__author__ = "Shamistan Karimov, Bach Nguyen"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

from typing import List, Tuple

from src.environments_wrappers.zenoh.transport.zenoh_pub import ZenohPubTransport


class Zenoh_BaseManager:
    def __init__(
        self,
        environment_cfg: dict = None,
        zenoh_cfg: dict = None,
        **kwargs,
    ) -> None:
        """
        Initializes the Zenoh environment manager.

        Args:
            environment_cfg (dict): Environment configuration.
            flares_cfg (dict): Flares configuration.
            **kwargs: Additional arguments.
        """

        self.trigger_reset = False

        self.modifications: List[Tuple[callable, dict]] = []

        self.robot_cfg = environment_cfg["robots_settings"]["parameters"]

        self.rocks_randomize_keyexpr = zenoh_cfg["keyexprs"]["randomize_rocks"]

        self.transports: List[ZenohPubTransport] = []

        self.sim_running_pub = ZenohPubTransport(
            keyexpr=zenoh_cfg["keyexprs"]["is_sim_running"],
            wire_format= self.robot_cfg ["zenoh"]["is_sim_running"]["wire_format"],
            is_logging= self.robot_cfg ["zenoh"]["is_sim_running"]["is_logging"],
            log_every_n= zenoh_cfg["pub_log_every_n"]
        )
        self.transports.append(self.sim_running_pub)

        self.transports_inited = False

    def periodic_update(self, dt: float) -> None:
        """
        Updates the lab.

        Args:
            dt (float): Time step
        """

        raise NotImplementedError

    def reset(self) -> None:
        """
        Resets the lab to its initial state.
        """

        raise NotImplementedError

    def clear_modifications(self) -> None:
        """
        Clears the list of modifications to be applied to the lab
        """

        self.modifications: List[Tuple[callable, dict]] = []

    def apply_modifications(self) -> None:
        """
        Applies the list of modifications to the lab
        """

        for mod in self.modifications:
            mod[0](**mod[1])
        self.clear_modifications()

    def pub_sim_is_running(self, is_running: bool) -> None:
        """
        Publish to Zenoh keyexpr to let subscribers know that the simulation is running
        """
        if self.transports_inited:
            self.sim_running_pub.publish({"is_running": is_running})

    def close(self) -> None:
        for t in self.transports:
            t.close()
