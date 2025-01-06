import logging
import time

from nodes.sensors.imu.imu_ahrs_filter import FilteredIMU
from utils.logging_utils import Logger
from utils.messages.gyro_data_msg import GYRO_DATA_MSG
from utils.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from utils.mqtt_utils import MQTTPublisher
from utils.topic_to_message_type import TOPIC_GYRO, TOPIC_RAW_IMU, TOPIC_RAW_IMU_UNSCALED_BIASED


class IMU_Node(object):
    def __init__(self, logging_level=logging.INFO):
        """
        Initializes the IMU_Node with a logger, filtered IMU, and MQTT publisher.

        :param logging_level: The logging level for the logger.
        """
        self.logger = Logger('imu_node', 'logs/imu_node.log', level=logging_level)
        self.filtered_imu = FilteredIMU()
        self.mqtt_publisher = MQTTPublisher(broker_address="localhost",
                                            topic_to_message_map={TOPIC_GYRO: GYRO_DATA_MSG,
                                                                  TOPIC_RAW_IMU: RAW_IMU_DATA_MSG,
                                                                  TOPIC_RAW_IMU_UNSCALED_BIASED: RAW_IMU_DATA_MSG})

        # Set this to 400Hz but can only actually publish at about 280Hz due to availability of IMU data
        self.loop_rate_hz = 400

    def run(self):
        """
        Runs the main loop of the IMU node, publishing IMU data at a specified rate.
        """
        self.mqtt_publisher.run()

        init_cycle = True

        try:
            while True:
                start_time = time.time()
                if init_cycle:
                    # Calibrate the IMU on the first cycle
                    self.filtered_imu.calibrate()
                    init_cycle = False
                else:
                    # Update the IMU data
                    self.filtered_imu.update()
                    t = time.time()
                    pitch, roll, yaw = self.filtered_imu.get_orientation()

                    # Publish gyro data
                    gyro_msg = GYRO_DATA_MSG(timestamp=t, pitch=pitch, roll=roll, yaw=yaw)
                    self.mqtt_publisher.publish_msg(TOPIC_GYRO, gyro_msg)

                    # Get and publish raw IMU data
                    accel = self.filtered_imu.imu.get_accel()
                    gyro = self.filtered_imu.imu.get_gyro()
                    gyro_raw = self.filtered_imu.imu.get_gyro_raw()

                    raw_imu_data = RAW_IMU_DATA_MSG()
                    raw_imu_data.timestamp = t
                    raw_imu_data.accel_x = accel[0]
                    raw_imu_data.accel_y = accel[1]
                    raw_imu_data.accel_z = accel[2]
                    raw_imu_data.gyro_x = gyro[0]
                    raw_imu_data.gyro_y = gyro[1]
                    raw_imu_data.gyro_z = gyro[2]
                    self.mqtt_publisher.publish_msg(TOPIC_RAW_IMU, raw_imu_data)

                    # Publish unscaled and biased raw IMU data
                    raw_imu_data_unscaled_biased = RAW_IMU_DATA_MSG()
                    raw_imu_data_unscaled_biased.timestamp = t
                    raw_imu_data_unscaled_biased.accel_x = accel[0]
                    raw_imu_data_unscaled_biased.accel_y = accel[1]
                    raw_imu_data_unscaled_biased.accel_z = accel[2]
                    raw_imu_data_unscaled_biased.gyro_x = gyro_raw[0]
                    raw_imu_data_unscaled_biased.gyro_y = gyro_raw[1]
                    raw_imu_data_unscaled_biased.gyro_z = gyro_raw[2]
                    self.mqtt_publisher.publish_msg(TOPIC_RAW_IMU_UNSCALED_BIASED, raw_imu_data_unscaled_biased)

                end_time = time.time()
                # Calculate sleep time to maintain loop rate
                sleep_time = 1/self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            self.logger.info("IMU stopped by user.")
        finally:
            self.mqtt_publisher.stop()

if __name__ == "__main__":
    imu_node = IMU_Node()
    imu_node.run()
