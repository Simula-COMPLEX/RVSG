import cv2
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetEntityState


class CameraSpawner(Node):

    def __init__(self):
        super().__init__('camera_spawner')

        self.spawn_srv = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_srv = self.create_client(DeleteEntity, '/delete_entity')
        self.state_srv = self.create_client(GetEntityState, '/get_entity_state')

        for cli, name in [
            (self.spawn_srv, '/spawn_entity'),
            (self.delete_srv, '/delete_entity'),
            (self.state_srv, '/get_entity_state'),
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name} service...')

        self.spawn_camera()

    def spawn_camera(self):
        camera_name = 'top_down_camera'
        self.check_and_delete(camera_name)

        sdf_path = './src/test_pal_robotics/test_pal_robotics/top_down_camera.sdf'
        with open(sdf_path, 'r') as f:
            sdf = f.read()

        req = SpawnEntity.Request()
        req.name = camera_name
        req.xml = sdf
        req.robot_namespace = '/top_down'
        req.reference_frame = 'world'
        req.initial_pose.position.x = -2.5
        req.initial_pose.position.y = -2.0
        req.initial_pose.position.z = 26.0
        req.initial_pose.orientation.x = 0.0
        req.initial_pose.orientation.y = 0.0
        req.initial_pose.orientation.z = 0.0
        req.initial_pose.orientation.w = 1.0

        self.future = self.spawn_srv.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result().success:
            self.get_logger().info('Camera spawned successfully')
        else:
            self.get_logger().error(f"Failed to spawn: {self.future.result().status_message}")

    def check_and_delete(self, model_name):
        req = GetEntityState.Request()
        req.name = model_name
        future = self.state_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result.success:
            self.get_logger().info(f"{model_name} already exists. Deleting...")
            del_req = DeleteEntity.Request()
            del_req.name = model_name
            del_future = self.delete_srv.call_async(del_req)
            rclpy.spin_until_future_complete(self, del_future)
            del_result = del_future.result()
            if del_result.success:
                self.get_logger().info(f"Deleted {model_name}")
            else:
                self.get_logger().warn(f"Failed to delete {model_name}: {del_result.status_message}")
        else:
            self.get_logger().info(f"{model_name} not found. No need to delete.")


def crop_image():
    img = cv2.imread('image1.png')
    crop_img = img[100:950, 0:1250]
    cv2.imwrite('image1_crop.png', crop_img)


def main(args=None):
    # ros2 run rqt_image_view rqt_image_view
    # rclpy.init(args=args)
    # node = CameraSpawner()
    # node.destroy_node()
    # rclpy.shutdown()

    crop_image()


if __name__ == '__main__':
    main()
