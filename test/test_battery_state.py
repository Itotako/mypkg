import unittest
from unittest.mock import patch
from mypkg.battery_state import BatteryStatePublisher

class TestBatteryStatePublisher(unittest.TestCase):

    @patch('subprocess.run')
    def test_battery_state(self, mock_run):
        # モックされるsubprocess.runの返り値を設定
        mock_run.return_value.stdout = "Battery 0: Discharging, 50%, 1.23 V"

        # BatteryStatePublisherのインスタンスを作成
        publisher = BatteryStatePublisher()

        # 実際にテスト対象の関数を呼び出す
        publisher.timer_callback()

        # モックが想定通りに呼ばれたかを確認
        mock_run.assert_called_once_with(['acpi'], capture_output=True, text=True)

        # バッテリー状態が想定通りに更新されているかを確認
        self.assertEqual(publisher.state_publisher_.get_published_message().data, 'Not Charging')
        self.assertEqual(publisher.level_publisher_.get_published_message().data, 50.0)

if __name__ == '__main__':
    unittest.main()

