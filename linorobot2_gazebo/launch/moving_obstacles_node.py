#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

class MovingObstacles(Node):
    def __init__(self):
        super().__init__('moving_obstacles_node')

        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        self.get_logger().info('Waiting for /gazebo/set_entity_state...')
        self.cli.wait_for_service()
        self.get_logger().info('Connected to /gazebo/set_entity_state')

        # ================== CAU HINH CHUNG ==================
        # Chu ky cap nhat. Neu Gazebo lag, tang len 0.5 hoac 0.7.
        self.dt = 0.5
        self.t = 0.0

        # Day la toa do MAT TUONG THAT, khong phai toa do tam vat can.
        # Neu vat can van cham tuong, hay thu hep 2 gia tri nay lai.
        self.wall_A_x = 6.2
        self.wall_B_x = 9.3

        # Ban kinh vat can cylinder trong world.
        # Neu radius trong world la 0.50 thi de 0.50.
        self.obstacle_radius = 0.45

        # Khoang cach an toan cach tuong/cot.
        # Tang len neu vat can van bi lo qua tuong.
        self.safe_margin = 0.10

        # Gioi han cho TAM vat can.
        # Vật không được dùng trực tiếp wall_A_x -> wall_B_x,
        # vì đó chỉ là giới hạn tường, còn vật cản có bán kính.
        self.x_min = self.wall_A_x + self.obstacle_radius + self.safe_margin
        self.x_max = self.wall_B_x - self.obstacle_radius - self.safe_margin

        if self.x_min >= self.x_max:
            raise ValueError(
                'Khoang cach giua 2 tuong qua hep so voi radius + safe_margin. '
                'Hay giam obstacle_radius/safe_margin hoac sua wall_A_x/wall_B_x.'
            )

        # ================== CAU HINH 5 VAT CAN ==================
        # name, axis, fixed_y, speed, direction
        #
        # direction =  1: bat dau gan tuong B roi chay ve A
        # direction = -1: bat dau gan tuong A roi chay ve B
        #
        # 5 vat chay nguoc nhau lien tiep:
        # 0: A <-> B
        # 1: B <-> A
        # 2: A <-> B
        # 3: B <-> A
        # 4: A <-> B

        self.obstacles = [
            ('moving_obstacle_0', 'x', -6.31, 0.20, -1),
            ('moving_obstacle_1', 'x', -3.00, 0.20,  1),
            ('moving_obstacle_2', 'x',  0.00, 0.20, -1),
            ('moving_obstacle_3', 'x',  2.80, 0.20,  1),
            ('moving_obstacle_4', 'x',  5.00, 0.20, -1),
        ]

        self.get_logger().info(
            f'Center limit: x_min={self.x_min:.2f}, x_max={self.x_max:.2f}'
        )

        self.timer = self.create_timer(self.dt, self.update)

    def update(self):
        self.t += self.dt

        for name, axis, fixed_y, speed, direction in self.obstacles:
            center = (self.x_min + self.x_max) / 2.0
            amp = (self.x_max - self.x_min) / 2.0

            # Di chuyen qua lai, cac vat xen ke nguoc chieu nhau.
            x = center + direction * amp * math.cos(self.t * speed)

            # Khoa bien lan cuoi de tam vat can khong bao gio vuot qua gioi han.
            x = max(self.x_min, min(x, self.x_max))

            state = EntityState()
            state.name = name
            state.reference_frame = 'world'

            if axis == 'x':
                state.pose.position.x = x
                state.pose.position.y = fixed_y
            else:
                state.pose.position.x = fixed_y
                state.pose.position.y = x

            state.pose.position.z = 0.0
            state.pose.orientation.w = 1.0

            req = SetEntityState.Request()
            req.state = state

            self.cli.call_async(req)


def main():
    rclpy.init()
    node = MovingObstacles()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
