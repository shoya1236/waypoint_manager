import rclpy
from rclpy.node import Node
from waypoint_manager.srv import SpeakText

class SpeakTextClient(Node):
    def __init__(self):
        super().__init__('speak_text_client')
        self.client = self.create_client(SpeakText, 'speak_text')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービス待機中...')

        self.send_request(1)  # `text_id=1`（ウェイポイント到達）をリクエスト

    def send_request(self, text_id):
        request = SpeakText.Request()
        request.text_id = text_id
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        response = future.result()
        self.get_logger().info(f'🎤 音声応答: {response.text}')

def main(args=None):
    rclpy.init(args=args)
    node = SpeakTextClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
