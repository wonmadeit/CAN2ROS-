import rospy
import can  # python-can
import cantools  # DBC parser
from std_msgs.msg import Int32
from threading import Thread
import subprocess 

# ---- 사용자 설정: 원하는 신호명 입력 ----
SIGNAL_MAP = {
    "SAS_Angle": {"topic": "/angle", "type": "int"},
    "LONG_ACCEL": {"topic": "/accel", "type": "int"},
}

# ---- 사용할 dbc파일명 입력 ----
DBC_PATH = "hyundai_2015_ccan.dbc"  # DBC 파일 경로

class CanDBCNode:
    def __init__(self):
        # ROS1 노드 초기화
        rospy.init_node('can_dbc_to_topics')

        # CAN 인터페이스 활성화
        self.configure_can_interface()

        # DBC 파일 로드
        self.db = cantools.database.load_file(DBC_PATH)

        # 퍼블리셔 딕셔너리 초기화
        self._publishers = {}

        # DBC 파일 내 신호 처리
        for message in self.db.messages:
            for signal in message.signals:
                # SIGNAL_MAP에 해당하는 신호만 처리
                if signal.name in SIGNAL_MAP:
                    topic_name = f"/can/{signal.name}"
                    msg_type = Int32  # ESP12 관련 신호는 int 타입으로 설정
                    publisher = rospy.Publisher(topic_name, msg_type, queue_size=10)
                    self._publishers[signal.name] = publisher
                    rospy.loginfo(f"Publishing {signal.name} on {topic_name}")

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

            # SIGNAL_MAP에서 정의한 신호만 퍼블리시 (ESP12 관련 신호만 처리)
            for signal_name, publisher in self._publishers.items():
                if signal_name in decoded:
                    val = decoded[signal_name]
                    msg_type = SIGNAL_MAP[signal_name]["type"]
                    if msg_type == "int":
                        pub_msg = Int32()
                        pub_msg.data = int(val)
                        publisher.publish(pub_msg)

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
