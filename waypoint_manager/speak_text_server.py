import rclpy
from rclpy.node import Node
from waypoint_manager.srv import SpeakText  # 生成されたサービス型をインポート
import csv
import os

class SpeakTextServer(Node):
    def __init__(self):
        super().__init__('speak_text_server')
        self.srv = self.create_service(SpeakText, 'speak_text', self.speak_text_callback)

        # ID→テキストのマッピングをロード
        self.declare_parameter('id_to_text_csv', '')
        csv_path = self.get_parameter('id_to_text_csv').value
        self.id_to_text = self.load_id_to_text(csv_path)

    def load_id_to_text(self, filename):
        """CSV から ID と対応するテキストを読み込む"""
        id_to_text = {}
        if not os.path.exists(filename):
            self.get_logger().error(f"CSV ファイルが見つかりません: {filename}")
            return id_to_text

        try:
            with open(filename, mode='r', encoding='utf-8') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    id_to_text[int(row["id"])] = row["text"]
            self.get_logger().info("✅ CSV 読み込み成功")
        except Exception as e:
            self.get_logger().error(f"🚨 CSV 読み込みエラー: {e}")
        return id_to_text

    def speak_text_callback(self, request, response):
        """リクエストされた text_id に対応するテキストを返す"""
        text_id = request.text_id
        response.text = self.id_to_text.get(text_id, "不明なID")
        self.get_logger().info(f"🎤 音声応答: {response.text}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SpeakTextServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
