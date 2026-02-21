import rclpy
from rclpy.node import Node
import math
import cv2
from cv_bridge import CvBridge
import numpy as np

from sensor_msgs.msg import Image
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

class QRSuruOperasyonu(Node):
    def __init__(self):
        super().__init__('qr_suru_operasyonu_node')
        
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        
        self.operasyon_durumu = "KALKIS" 
        self.kilitli_hedef_x = 0.0
        self.kilitli_hedef_y = 0.0

       
        self.merkez_x = 0 
        self.merkez_y = 0
        self.kamera_ayari_yapildi = False
        
        self.dinamik_tolerans = 25 
        
        
        self.kp = 0.003  
        self.kd = 0.006  
        self.max_hiz = 0.8
        
       
        self.l_gordu = False; self.l_err_x = 0.0; self.l_err_y = 0.0; self.l_prev_x = 0.0; self.l_prev_y = 0.0
        self.l_alan = 0; self.l_kayip = 0
        
       
        self.y_gordu = False; self.y_err_x = 0.0; self.y_err_y = 0.0; self.y_prev_x = 0.0; self.y_prev_y = 0.0
        self.y_alan = 0; self.y_kayip = 0
        self.y_son_vx = 0.0; self.y_son_vy = 0.0

        
        self.lider_durum = 0; self.lider_arm = 0; self.lider_x = 0.0; self.lider_y = 0.0; self.lider_z = 0.0; self.lider_yaw = 0.0
        self.kanat_durum = 0; self.kanat_arm = 0; self.kanat_x = 0.0; self.kanat_y = 0.0; self.kanat_z = 0.0; self.kanat_yaw = 0.0
        
        self.lider_arama_x = 0.0; self.lider_arama_yonu = 1 
        self.yanci_arama_x = 0.0; self.yanci_arama_yonu = -1 

        self.lider_pose_sub = self.create_subscription(VehicleLocalPosition, '/px4_1/fmu/out/vehicle_local_position_v1', self.l_pose_cb, qos_profile)
        self.lider_status_sub = self.create_subscription(VehicleStatus, '/px4_1/fmu/out/vehicle_status_v1', self.l_status_cb, qos_profile)
        self.kanat_pose_sub = self.create_subscription(VehicleLocalPosition, '/px4_2/fmu/out/vehicle_local_position_v1', self.k_pose_cb, qos_profile)
        self.kanat_status_sub = self.create_subscription(VehicleStatus, '/px4_2/fmu/out/vehicle_status_v1', self.k_status_cb, qos_profile)

        self.lider_mode_pub = self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', 10)
        self.lider_traj_pub = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        self.lider_cmd_pub = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', 10)

        self.kanat_mode_pub = self.create_publisher(OffboardControlMode, '/px4_2/fmu/in/offboard_control_mode', 10)
        self.kanat_traj_pub = self.create_publisher(TrajectorySetpoint, '/px4_2/fmu/in/trajectory_setpoint', 10)
        self.kanat_cmd_pub = self.create_publisher(VehicleCommand, '/px4_2/fmu/in/vehicle_command', 10)

        self.bridge = CvBridge()
        self.qr_dedektor = cv2.QRCodeDetector()
        
        self.lider_cam_sub = self.create_subscription(Image, '/px4_1/camera/image_raw', self.lider_kamera_cb, qos_profile_sensor_data)
        self.kanat_cam_sub = self.create_subscription(Image, '/px4_2/camera/image_raw', self.kanat_kamera_cb, qos_profile_sensor_data)

        self.sayac = 0
        self.timer = self.create_timer(0.05, self.sistem_dongusu) 
        print("🦅 HYBRID SÜRÜ V15 AKTİF! (Çarpışma Engellendi, Aşağı İniş Düzeltildi!)")

    def l_status_cb(self, msg): self.lider_durum = msg.nav_state; self.lider_arm = msg.arming_state
    def k_status_cb(self, msg): self.kanat_durum = msg.nav_state; self.kanat_arm = msg.arming_state
    def l_pose_cb(self, msg): 
        self.lider_x = msg.x; self.lider_y = msg.y; self.lider_z = -msg.z; self.lider_yaw = msg.heading 
    def k_pose_cb(self, msg): 
        self.kanat_x = msg.x; self.kanat_y = msg.y; self.kanat_z = -msg.z; self.kanat_yaw = msg.heading 

    def komut_gonder(self, komut, drone_id, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = komut; msg.param1 = float(p1); msg.param2 = float(p2)
        msg.target_system = drone_id + 1  
        msg.target_component = 1; msg.source_system = 255; msg.source_component = 1; msg.from_external = True
        if drone_id == 1: self.lider_cmd_pub.publish(msg)
        else: self.kanat_cmd_pub.publish(msg)

   
    def setpoint_konum(self, x, y, z):
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position = [float(x), float(y), float(-z)] # Konum için -z çünkü yükseklik 
        sp.velocity = [float('nan'), float('nan'), float('nan')] 
        sp.yaw = 0.0
        return sp

    def hizala_ve_donustur(self, err_x, err_y, prev_x, prev_y, mevcut_yaw, vz_down=0.0):
        d_err_x = err_x - prev_x
        d_err_y = err_y - prev_y
        
        ham_vy = (self.kp * err_x) + (self.kd * d_err_x)
        ham_vx = -((self.kp * err_y) + (self.kd * d_err_y))
        
        vx_body = max(-self.max_hiz, min(self.max_hiz, ham_vx))
        vy_body = max(-self.max_hiz, min(self.max_hiz, ham_vy))
        
       
        vx_ned = (vx_body * math.cos(mevcut_yaw)) - (vy_body * math.sin(mevcut_yaw))
        vy_ned = (vx_body * math.sin(mevcut_yaw)) + (vy_body * math.cos(mevcut_yaw))
        
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position = [float('nan'), float('nan'), float('nan')] 
        
        
        sp.velocity = [float(vx_ned), float(vy_ned), float(vz_down)] 
        
        sp.yaw = float('nan')
        sp.yawspeed = 0.0
        return sp, vx_ned, vy_ned

    
    def lider_kamera_cb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        
        if not self.kamera_ayari_yapildi:
            h, w, _ = cv_image.shape
            self.merkez_x = w // 2
            self.merkez_y = h // 2
            self.kamera_ayari_yapildi = True

        cv2.putText(cv_image, "LIDER (GOZCU)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        data, bbox, _ = self.qr_dedektor.detectAndDecode(cv_image)
        if bbox is not None and len(bbox) > 0:
            self.l_gordu = True
            noktalar = np.int32(bbox[0])
            cv2.polylines(cv_image, [noktalar], True, (0, 255, 0), 2)
            
            x, y, bw, bh = cv2.boundingRect(noktalar)
            self.l_alan = bw * bh
            
            qr_x = x + bw // 2; qr_y = y + bh // 2
            self.l_err_x = float(qr_x - self.merkez_x); self.l_err_y = float(qr_y - self.merkez_y)
            
            cv2.putText(cv_image, f"ALAN: {int(self.l_alan)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.circle(cv_image, (self.merkez_x, self.merkez_y), 25, (255, 255, 255), 1)
            cv2.line(cv_image, (self.merkez_x, self.merkez_y), (qr_x, qr_y), (0, 255, 255), 2)
            
            if self.operasyon_durumu == "ARAMA_YAPILIYOR":
                self.operasyon_durumu = "LIDER_HIZALANIYOR"
        else:
            self.l_gordu = False
            cv2.circle(cv_image, (self.merkez_x, self.merkez_y), 25, (255, 255, 255), 1)

        cv2.putText(cv_image, f"MOD: {self.operasyon_durumu}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("LIDER EKRANI", cv_image); cv2.waitKey(1) 

    
    def kanat_kamera_cb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        data, bbox, _ = self.qr_dedektor.detectAndDecode(cv_image)
        
        cv2.putText(cv_image, "YANCI (VURUCU)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if bbox is not None and len(bbox) > 0:
            self.y_gordu = True
            noktalar = np.int32(bbox[0])
            cv2.polylines(cv_image, [noktalar], True, (0, 255, 0), 3)
            
            x, y, bw, bh = cv2.boundingRect(noktalar)
            self.y_alan = bw * bh 
            
            qr_x = x + bw // 2; qr_y = y + bh // 2
            self.y_err_x = float(qr_x - self.merkez_x); self.y_err_y = float(qr_y - self.merkez_y)
            
            self.dinamik_tolerans = max(20, int(np.sqrt(self.y_alan) * 0.2))
            
            cv2.putText(cv_image, f"ALAN: {int(self.y_alan)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(cv_image, f"TOLERANS: {self.dinamik_tolerans}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            renk = (0, 255, 0) if abs(self.y_err_x) < self.dinamik_tolerans and abs(self.y_err_y) < self.dinamik_tolerans else (0, 0, 255)
            cv2.circle(cv_image, (self.merkez_x, self.merkez_y), self.dinamik_tolerans, renk, 2)
            cv2.line(cv_image, (self.merkez_x, self.merkez_y), (qr_x, qr_y), (0, 255, 255), 2)

            if self.operasyon_durumu == "ARAMA_YAPILIYOR":
                self.operasyon_durumu = "YANCI_HIZALANIYOR"
        else:
            self.y_gordu = False
            cv2.circle(cv_image, (self.merkez_x, self.merkez_y), self.dinamik_tolerans, (255, 255, 255), 1)
            
        cv2.putText(cv_image, f"MOD: {self.operasyon_durumu}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("YANCI EKRANI", cv_image); cv2.waitKey(1)

    
    def sistem_dongusu(self):
        if self.operasyon_durumu != "GOREV_TAMAM":
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            offboard_msg.position = True; offboard_msg.velocity = True; offboard_msg.acceleration = False
            self.lider_mode_pub.publish(offboard_msg); self.kanat_mode_pub.publish(offboard_msg)

        lider_sp = TrajectorySetpoint(); kanat_sp = TrajectorySetpoint()

        
        if self.operasyon_durumu == "KALKIS":
            lider_sp = self.setpoint_konum(0.0, 0.0, 5.0); kanat_sp = self.setpoint_konum(0.0, 3.0, 5.0)
            if self.lider_z > 4.5 and self.kanat_z > 4.5: self.operasyon_durumu = "ARAMA_YAPILIYOR"

       
        elif self.operasyon_durumu == "ARAMA_YAPILIYOR":
            self.lider_arama_x += 0.02 * self.lider_arama_yonu 
            if self.lider_arama_x > 10.0: self.lider_arama_yonu = -1
            elif self.lider_arama_x < -2.0: self.lider_arama_yonu = 1

            self.yanci_arama_x += 0.02 * self.yanci_arama_yonu
            if self.yanci_arama_x < -10.0: self.yanci_arama_yonu = 1
            elif self.yanci_arama_x > 2.0: self.yanci_arama_yonu = -1

            lider_sp = self.setpoint_konum(self.lider_arama_x, -2.0, 5.0)
            kanat_sp = self.setpoint_konum(self.yanci_arama_x, 4.0, 5.0)

        
        elif self.operasyon_durumu == "LIDER_HIZALANIYOR":
            kanat_sp = self.setpoint_konum(self.kanat_x, self.kanat_y, self.kanat_z) 
            
            if self.l_gordu:
                self.l_kayip = 0
                lider_sp, _, _ = self.hizala_ve_donustur(self.l_err_x, self.l_err_y, self.l_prev_x, self.l_prev_y, self.lider_yaw, vz_down=0.0)
                self.l_prev_x = self.l_err_x; self.l_prev_y = self.l_err_y
                
                if abs(self.l_err_x) < 20 and abs(self.l_err_y) < 20:
                    self.kilitli_hedef_x = self.lider_x; self.kilitli_hedef_y = self.lider_y
                    self.operasyon_durumu = "ORTAK_HEDEF_YAKLASMA"
            else:
                self.l_kayip += 1
                if self.l_kayip < 20: lider_sp, _, _ = self.hizala_ve_donustur(0.0, 0.0, 0.0, 0.0, self.lider_yaw, vz_down=0.0) 
                else: self.operasyon_durumu = "ARAMA_YAPILIYOR"

       
        elif self.operasyon_durumu == "YANCI_HIZALANIYOR":
            lider_sp = self.setpoint_konum(self.lider_x, self.lider_y, self.lider_z) 
            
            if self.y_gordu:
                self.y_kayip = 0
                kanat_sp, _, _ = self.hizala_ve_donustur(self.y_err_x, self.y_err_y, self.y_prev_x, self.y_prev_y, self.kanat_yaw, vz_down=0.0)
                self.y_prev_x = self.y_err_x; self.y_prev_y = self.y_err_y
                
                if abs(self.y_err_x) < 20 and abs(self.y_err_y) < 20:
                    self.kilitli_hedef_x = self.kanat_x; self.kilitli_hedef_y = self.kanat_y
                    self.operasyon_durumu = "ORTAK_HEDEF_YAKLASMA"
            else:
                self.y_kayip += 1
                if self.y_kayip < 20: kanat_sp, _, _ = self.hizala_ve_donustur(0.0, 0.0, 0.0, 0.0, self.kanat_yaw, vz_down=0.0) 
                else: self.operasyon_durumu = "ARAMA_YAPILIYOR"

        
        elif self.operasyon_durumu == "ORTAK_HEDEF_YAKLASMA":
            lider_sp = self.setpoint_konum(self.kilitli_hedef_x, self.kilitli_hedef_y, 6.0) 
            kanat_sp = self.setpoint_konum(self.kilitli_hedef_x, self.kilitli_hedef_y, 3.0) 
            
            mesafe = math.sqrt((self.kilitli_hedef_x - self.kanat_x)**2 + (self.kilitli_hedef_y - self.kanat_y)**2)
            if mesafe < 0.5 and self.kanat_z < 3.5:
                self.operasyon_durumu = "VURUCU_INIYOR"

       
        elif self.operasyon_durumu == "VURUCU_INIYOR":
            lider_sp = self.setpoint_konum(self.kilitli_hedef_x, self.kilitli_hedef_y, 6.0) 
            
            if self.y_gordu:
                self.y_kayip = 0
                
               
                if self.y_alan > 120000 or self.kanat_z <= 0.8:
                    print(f"🛬 PİSTE TEMAS BEKLENİYOR! (Alan: {self.y_alan}) Motorlar Kesilecek!")
                    self.komut_gonder(VehicleCommand.VEHICLE_CMD_NAV_LAND, 2)
                    self.operasyon_durumu = "GOREV_TAMAM"
                else:
                   
                    if abs(self.y_err_x) > self.dinamik_tolerans or abs(self.y_err_y) > self.dinamik_tolerans:
                        kanat_sp, vx, vy = self.hizala_ve_donustur(self.y_err_x, self.y_err_y, self.y_prev_x, self.y_prev_y, self.kanat_yaw, vz_down=0.0)
                   
                    else:
                        kanat_sp, vx, vy = self.hizala_ve_donustur(self.y_err_x, self.y_err_y, self.y_prev_x, self.y_prev_y, self.kanat_yaw, vz_down=0.4)
                    
                    self.y_prev_x = self.y_err_x; self.y_prev_y = self.y_err_y
                    self.y_son_vx = vx; self.y_son_vy = vy
            else:
                self.y_kayip += 1
                if self.y_kayip < 20: 
                    kanat_sp, _, _ = self.hizala_ve_donustur(0.0, 0.0, 0.0, 0.0, self.kanat_yaw, vz_down=0.0)
                else:
                    if self.kanat_z < 1.5:
                        self.komut_gonder(VehicleCommand.VEHICLE_CMD_NAV_LAND, 2)
                        self.operasyon_durumu = "GOREV_TAMAM"
                    else:
                        self.operasyon_durumu = "ARAMA_YAPILIYOR"

      
        elif self.operasyon_durumu == "GOREV_TAMAM":
            if self.kanat_z <= 0.25 and self.kanat_arm == 2:
                self.komut_gonder(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 2, p1=0.0)
                print("🛑 VURUCU PİSTE TEMAS ETTİ! Motorlar Susturuldu.")

        if self.operasyon_durumu != "GOREV_TAMAM":
            self.lider_traj_pub.publish(lider_sp); self.kanat_traj_pub.publish(kanat_sp)

       
        if self.sayac < 20: self.sayac += 1; return 
        if self.operasyon_durumu == "KALKIS":
            if self.lider_arm != 2: self.komut_gonder(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, p1=1.0)
            if self.kanat_arm != 2: self.komut_gonder(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 2, p1=1.0)
            if self.lider_durum != 14: self.komut_gonder(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, p1=1.0, p2=6.0)
            if self.kanat_durum != 14: self.komut_gonder(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 2, p1=1.0, p2=6.0)

def main(args=None):
    rclpy.init(args=args)
    node = QRSuruOperasyonu()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: cv2.destroyAllWindows(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()