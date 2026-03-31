import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32
import serial
import statistics
from collections import deque

class Uc4Publisher(Node):
    def __init__(self):
        super().__init__('uc4_adc_driver')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 460800) 
        self.declare_parameter('window_size', 5) 
        self.declare_parameter('median_size', 3)
        self.declare_parameter('adc_min', 0.0)
        self.declare_parameter('adc_max', 1023.0)
        self.declare_parameter('dist_min', 0.0)
        self.declare_parameter('dist_max', 1.0)
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value
        self.median_size = self.get_parameter('median_size').get_parameter_value().integer_value
        self.median_buffer = deque(maxlen=self.median_size)
        self.sma_buffer = deque(maxlen=self.window_size)
        self.get_logger().info(f'Median window: {self.median_size}, SMA window: {self.window_size}')
        self.publisher_ = self.create_publisher(Float32, 'distance', 10)
        self.ser = None
        self.timer = self.create_timer(0.01, self.timer_callback)

    def map_value(self, val):
        in_min = self.get_parameter('adc_min').value
        in_max = self.get_parameter('adc_max').value
        out_min = self.get_parameter('dist_min').value
        out_max = self.get_parameter('dist_max').value
        # Check to avoid division by zero
        if in_max == in_min:
            return out_min
        mapped = (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        return mapped

    def connect_serial(self):
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Connected to {port} at {baud} baud.')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'Reconnection failed: {e}', throttle_duration_sec=5.0)

    def timer_callback(self):
        # Check if connected
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
            return
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    parts = line.split(',')
                    if len(parts) >= 10:
                        raw_value = float(parts[-1])
                        # Median filter (reject spikes)
                        self.median_buffer.append(raw_value)
                        filtered_value = statistics.median(self.median_buffer)
                        # Simple moving average (smooth data)
                        self.sma_buffer.append(filtered_value)
                        smoothed_value = sum(self.sma_buffer) / len(self.sma_buffer)
                        # Linear mapping
                        distance_value = self.map_value(smoothed_value)
                        msg = Float32()
                        msg.data = distance_value
                        self.publisher_.publish(msg)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial lost: {e}')
            self.ser.close()
            self.ser = None # Trigger reconnection on next timer tick

def main(args=None):
    rclpy.init(args=args)
    node = Uc4Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser: node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()