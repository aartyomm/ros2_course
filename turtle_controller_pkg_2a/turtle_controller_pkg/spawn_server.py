#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from turtle_controller_pkg.srv import SpawnTurtle
from turtlesim.srv import Spawn

# === bg ===
from turtle_controller_pkg.srv import ChangeBackground
from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# === rm ===
from turtle_controller_pkg.srv import RemoveTurtle
from turtlesim.srv import Kill

class SpawnServer(Node):
    def __init__(self):
        super().__init__('spawn_server')
        self.cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            SpawnTurtle, 'spawn_turtle', self.handle_request,
            callback_group=self.cb_group)
        self.spawn_cli = self.create_client(Spawn, '/spawn')
    
        # === bg ===
        self.bg_srv = self.create_service(
            ChangeBackground,
            'change_background',
            self.handle_background,
            callback_group=self.cb_group
        )

        # === bg ===
        self.clear_cli = self.create_client(Empty, '/clear')

        # === bg ===
        self.param_cli = self.create_client(SetParameters, '/turtlesim/set_parameters')

        # === rm ===
        self.rm_srv = self.create_service(
            RemoveTurtle,
            'remove_turtle',
            self.handle_remove,
            callback_group=self.cb_group
        )

        # === rm ===
        self.kill_cli = self.create_client(Kill, '/kill')

        # === rm ===
        self.turtles = ['turtle1']

    async def handle_request(self, req, resp):
        if not (0 <= req.x <= 11 and 0 <= req.y <= 11):
            resp.success = False
            resp.message = 'Invalid coords, out of range: (0-11)'
            return resp
        
        spawn_req = Spawn.Request()
        spawn_req.x = float(req.x)
        spawn_req.y = float(req.y)
        spawn_req.name = req.turtle_name
        future = self.spawn_cli.call_async(spawn_req)
        result = await future
        if result is not None:
            resp.success = True
            resp.message = f'Spawned {req.turtle_name}'
            self.turtles.append(req.turtle_name) # === rm ===
        else:
            resp.success = False
            resp.message = 'Spawn service call failed'
        return resp
    
    # === bg ===
    async def handle_background(self, req, resp):

        if not (0 <= req.r <= 255 and 0 <= req.g <= 255 and 0 <= req.b <= 255):
            resp.success = False
            resp.message = "RGB must be in range 0-255"
            return resp

        params = []

        for name, value in [
            ('background_r', req.r),
            ('background_g', req.g),
            ('background_b', req.b),
        ]:
            param = Parameter()
            param.name = name
            param.value = ParameterValue(
                type=ParameterType.PARAMETER_INTEGER,
                integer_value=value
            )
            params.append(param)

        param_req = SetParameters.Request()
        param_req.parameters = params

        future = self.param_cli.call_async(param_req)
        await future

        clear_req = Empty.Request()
        future = self.clear_cli.call_async(clear_req)
        await future

        resp.success = True
        resp.message = f'Background changed to RGB({req.r}, {req.g}, {req.b})'

        return resp
    
    # === rm ===
    async def handle_remove(self, req, resp):

        if not self.turtles:
            resp.success = False
            resp.message = "No turtles to remove"
            return resp

        removed = []

        for name in self.turtles:
            kill_req = Kill.Request()
            kill_req.name = name

            future = self.kill_cli.call_async(kill_req)
            await future

            removed.append(name)

        self.turtles.clear()

        resp.success = True
        resp.message = f"Removed turtles: {', '.join(removed)}"

        return resp


def main(args=None):
    rclpy.init(args=args)
    node = SpawnServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
