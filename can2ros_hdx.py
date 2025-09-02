import rospy
import can  # python-can
import cantools  # DBC 파서
from sensor_msgs.msg import NavSatFix  # NavSatFix 메시지 사용
from threading import Thread
import subprocess  # subprocess 모듈 추가

# ---- 사용자 설정: ESP12 신호만 신호 -> 토픽 매핑 ----
SIGNAL_MAP = {
    "Latitude": {"topic": "/gps/latitude", "type": "float"},
    "Longitude": {"topic": "/gps/longitude", "type": "float"},
    # "Altitude": {"topic": "/gps/altitude", "type": "float"},
}

DBC_PATH = "hyundai_2015_ccan.dbc"  # DBC 파일 경로

class CanDBCNode:
    def __init__(self):
        # ROS1 노드 초기화
        rospy.init_node('can_dbc_to_gps_topics')

        # CAN 인터페이스 활성화
        self.configure_can_interface()

        # DBC 파일 로드
        self.db = cantools.database.load_file(DBC_PATH)

        # 퍼블리셔 딕셔너리 초기화
        self.publisher_lat = rospy.Publisher("/gps/latitude", NavSatFix, queue_size=10)
        self.publisher_lon = rospy.Publisher("/gps/longitude", NavSatFix, queue_size=10)
        self.publisher_alt = rospy.Publisher("/gps/altitude", NavSatFix, queue_size=10)

        # DBC 파일 내 신호 처리
        self._publishers = {}

        # CAN 버스 오픈 (SocketCAN)
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')

        # 수신 스레드
        self.running = True
        self.rx_thread = Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

    def configure_can_interface(self):
        """
        CAN 인터페이스를 자동으로 설정하는 함수
        """
        try:
            rospy.loginfo("Configuring can0 interface with 500000 bitrate...")
            # CAN 인터페이스 비트레이트 설정 (500kbps 예시)
            subprocess.run(["sudo", "ip", "link", "set", "can0", "down"], check=True)  # can0 인터페이스 다운
            subprocess.run(["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "500000", "restart-ms", "100"], check=True)  # 비트레이트 설정
            subprocess.run(["sudo", "ip", "link", "set", "can0", "up"], check=True)  # can0 인터페이스 업
            rospy.loginfo("can0 interface configured successfully.")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error in configuring can0: {e}")
            exit(1)

    def rx_loop(self):
        dbc_msg_cache = {}
        while self.running and not rospy.is_shutdown():
            msg = self.bus.recv(timeout=1.0)  # blocking
            if msg is None:
                continue

            data = bytes(msg.data)

            # 해당 ID 디코딩 가능한 DBC 메시지 찾기
            dbc_msg = None
            for m_candidate in self.db.messages:
                if m_candidate.frame_id == msg.arbitration_id:
                    dbc_msg = m_candidate
                    break

            if dbc_msg is None:
                continue  # DBC에 없는 ID면 패스

            try:
                decoded = dbc_msg.decode(data, decode_choices=True)
            except Exception as e:
                rospy.logwarn(f"Decode fail ID=0x{msg.arbitration_id:X}: {e}")
                continue

            # GPS 관련 신호 처리 (위도, 경도, 고도)
            gps_data = {}
            for signal_name in SIGNAL_MAP.keys():
                if signal_name in decoded:
                    val = decoded[signal_name]
                    gps_data[signal_name] = float(val)

            if gps_data:
                # 각 신호에 대해 NavSatFix 메시지 발행
                nav_msg = NavSatFix()
                nav_msg.header.stamp = rospy.Time.now()  # 현재 시간으로 타임스탬프 설정

                if "LATITUDE" in gps_data:
                    nav_msg.latitude = gps_data["LATITUDE"]
                    self.publisher_lat.publish(nav_msg)
                if "LONGITUDE" in gps_data:
                    nav_msg.longitude = gps_data["LONGITUDE"]
                    self.publisher_lon.publish(nav_msg)
                if "ALTITUDE" in gps_data:
                    nav_msg.altitude = gps_data["ALTITUDE"]
                    self.publisher_alt.publish(nav_msg)

    def destroy_node(self):
        self.running = False

def main():
    # 노드 생성 및 초기화
    node = CanDBCNode()
    
    try:
        rospy.spin()  # ROS1에서 메시지 처리 및 대기
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rospy.loginfo("Shutting down CAN node.")

if __name__ == "__main__":
    main()
