#include "control_step.h"
#include "sensor.h"

int pin_dung = 47; 

// --- THÔNG SỐ TỐC ĐỘ ---
int speed = 1200;             // Tốc độ chạy bình thường 
int slowSpeed = 8000;         // Chạy chậm dò vạch 2
int startSpeed = 13000;       // Tốc độ lúc mới khởi động
int he_so_giam_cua = 150;     // [MỚI] Hệ số hãm tốc khi vào cua (Tăng số này nếu cua bị văng/nhanh quá)

// --- THỜI GIAN GIA TỐC / GIẢM TỐC ---
unsigned long thoi_gian_tang_toc = 7000; 
unsigned long thoi_gian_giam_toc = 6000; // giảm tốc mượt vào trạm
unsigned long time_dung_lay_hang = 30000; // Đã đổi thành 30s
unsigned long thoi_gian_bat_dau = 0;     
unsigned long thoi_gian_bat_dau_giam = 0; 
unsigned long t_dung = 0; // Biến toàn cục mới để đếm thời gian dừng

bool dang_tang_toc = true;               
bool dang_giam_toc = false;               
float toc_do_khi_bat_dau_giam = 0;        

int trang_thai_14 = 0;        
float currentSpeedVal = startSpeed; 

// --- PID ---
static float lastError = 0;
static float integral = 0;
static float lastPidOutput = 0; 
float smoothBaseSpeed = 0; 
float smoothedError = 0; // Đưa lên làm biến toàn cục để dễ reset
float Ki = 0.02; 
float Kp_Soft = 150.0; float Kd_Soft = 250.0;  
float Kp_Hard = 200.0; float Kd_Hard = 500.0;  

void setup() 
{
  pinMode(pin_dung, OUTPUT);
  digitalWrite(pin_dung, HIGH); // Mặc định tắt tín hiệu
  
  pinMode(PUL_TRAI, OUTPUT); pinMode(DIR_TRAI, OUTPUT); pinMode(ENA_TRAI, OUTPUT);
  pinMode(PUL_PHAI, OUTPUT); pinMode(DIR_PHAI, OUTPUT); pinMode(ENA_PHAI, OUTPUT);
  
  pinMode(T4, INPUT); pinMode(T3, INPUT); pinMode(T2, INPUT); pinMode(T1, INPUT);
  pinMode(TG, INPUT); pinMode(PG, INPUT);   
  pinMode(P1, INPUT); pinMode(P2, INPUT); pinMode(P3, INPUT); pinMode(P4, INPUT);
  
  pinMode(cb1, INPUT_PULLUP);
  
  digitalWrite(ENA_TRAI, LOW); 
  digitalWrite(ENA_PHAI, LOW);
  
  currentSpeedVal = (float)startSpeed; 
  smoothBaseSpeed = (float)startSpeed; 
  thoi_gian_bat_dau = millis(); 
  dang_tang_toc = true;
}

void loop() {
  if (digitalRead(cb1) == LOW) {
      step_dc(false, false, HIGH, LOW, 0, 0); 
      
      dang_tang_toc = true;
      dang_giam_toc = false; 
      thoi_gian_bat_dau = millis(); 
      currentSpeedVal = (float)startSpeed;
      smoothBaseSpeed = (float)startSpeed; 
      integral = 0; 
      lastError = 0;
      smoothedError = 0; 
      trang_thai_14 = 0; 
      return; 
  }
  
  int error = In_SenSor();
  
  // BƯỚC 1: CHẠM VẠCH LẦN 1 -> Bắt đầu rà phanh từ từ
  if (error == 14 && trang_thai_14 == 0) {
      trang_thai_14 = 1; 
      dang_tang_toc = false; 
      dang_giam_toc = true;                  
      thoi_gian_bat_dau_giam = millis();     
      toc_do_khi_bat_dau_giam = smoothBaseSpeed; 
  }
  
  // BƯỚC 2: THOÁT VẠCH LẦN 1
  if (error != 14 && error != 13 && trang_thai_14 == 1) {
      trang_thai_14 = 2; 
  }
  if ((trang_thai_14 == 1 || trang_thai_14 == 2) && (millis() - thoi_gian_bat_dau_giam >= 10000)) {
      trang_thai_14 = 0;
      dang_giam_toc = false;      
      currentSpeedVal = (float)speed;
  }
  if (error == 14 && trang_thai_14 == 2) {
      step_dc(false, false, HIGH, LOW, 0, 0); 
      digitalWrite(pin_dung, LOW); // Báo hiệu đã dừng
      
      t_dung = millis(); 
      trang_thai_14 = 3; 
      return; 
  }

  if (trang_thai_14 == 3) {
      step_dc(false, false, HIGH, LOW, 0, 0); // Liên tục khóa bánh
      
      if (digitalRead(cb1) == LOW) {
          t_dung = millis(); 
      }

      if (millis() - t_dung >= time_dung_lay_hang) {
          digitalWrite(pin_dung, HIGH); 

          currentSpeedVal = (float)startSpeed; 
          smoothBaseSpeed = (float)startSpeed; 
          integral = 0; 
          lastError = 0;
          smoothedError = 0; 
          thoi_gian_bat_dau = millis(); 
          dang_tang_toc = true; 
          dang_giam_toc = false; 
          
          trang_thai_14 = 4; // Chuyển sang trạng thái 4 để bypass vạch
      }
      return; 
  }

  // BƯỚC 5: THOÁT KHỎI VẠCH TRẠM (BYPASS)
  if (trang_thai_14 == 4 && error != 14 && error != 13) {
      trang_thai_14 = 0; // Đã lăn bánh khỏi vạch, sẵn sàng bắt trạm mới
  }

  // =============================================================
  // MẤT LINE (CASE 13)
  // =============================================================
  if (error == 13) {
    step_dc(false, false, HIGH, LOW, 0, 0); 
    currentSpeedVal = (float)startSpeed;    
    smoothBaseSpeed = (float)startSpeed;    
    integral = 0;
    lastError = 0;
    smoothedError = 0; 
    thoi_gian_bat_dau = millis(); 
    dang_tang_toc = true; 
    dang_giam_toc = false; 
    trang_thai_14 = 0;                  
    return;
  }

  // TÍNH TOÁN VẬN TỐC THEO THỜI GIAN GIA/GIẢM TỐC
  if (dang_tang_toc && (trang_thai_14 == 0 || trang_thai_14 == 4)) {
      unsigned long t_chay = millis() - thoi_gian_bat_dau;
      if (t_chay < thoi_gian_tang_toc) {
          float phan_tram = (float)t_chay / (float)thoi_gian_tang_toc;
          currentSpeedVal = (float)startSpeed - ((float)(startSpeed - speed) * phan_tram);
      } else {
          currentSpeedVal = (float)speed;
          dang_tang_toc = false; 
      }
  } 
  else if (dang_giam_toc) {
      unsigned long t_giam = millis() - thoi_gian_bat_dau_giam;
      if (t_giam < thoi_gian_giam_toc) {
          float phan_tram = (float)t_giam / (float)thoi_gian_giam_toc;
          currentSpeedVal = toc_do_khi_bat_dau_giam + ((slowSpeed - toc_do_khi_bat_dau_giam) * phan_tram);
      } else {
          currentSpeedVal = (float)slowSpeed;
          dang_giam_toc = false; 
      }
  }

  float targetBaseSpeed = currentSpeedVal; 
  
  if (dang_giam_toc) {
      smoothBaseSpeed = targetBaseSpeed;
  } else if (targetBaseSpeed < smoothBaseSpeed) {
      smoothBaseSpeed = (targetBaseSpeed * 0.85) + (smoothBaseSpeed * 0.15); 
  } else {
      smoothBaseSpeed = (targetBaseSpeed * 0.1) + (smoothBaseSpeed * 0.9);
  }
  
  int baseSpeed = (int)smoothBaseSpeed;

  // =======================================================
  // BỘ TÍNH TOÁN PID KẾT HỢP ERROR SMOOTHING VÀ GIẢM TỐC CUA
  // =======================================================
  // Làm mượt sai số để xe lướt êm qua các mức cảm biến
  smoothedError = ((float)error * 0.35) + (smoothedError * 0.65); 

  int absError = abs(error);

  // Tự động giảm tốc nền khi lệch cảm biến (Vào cua)
  // Đã gọi biến he_so_giam_cua từ cấu hình tốc độ ở trên
  int finalBaseSpeed = baseSpeed + (absError * he_so_giam_cua);
  finalBaseSpeed = constrain(finalBaseSpeed, speed, slowSpeed); 
  
  // Cho phép mức 1,2,3 chạy êm ái. Chỉ khi lệch >= 4 mới lái gắt
  // [ĐÃ SỬA] Dùng biến Kp_Hard, Kd_Hard để dễ chỉnh sửa ở đầu file
  float Kp_Current = (absError <= 3) ? Kp_Soft : Kp_Hard; 
  float Kd_Current = (absError <= 3) ? Kd_Soft : Kd_Hard; 

  integral += smoothedError;
  integral = constrain(integral, -250, 250); 
  
  // Dùng sai số mượt để triệt tiêu dao động đạo hàm (Derivative Kick)
  float derivative = smoothedError - lastError; 
  
  float rawPid = (Kp_Current * smoothedError) + (Ki * integral) + (Kd_Current * derivative);
  lastError = smoothedError;
  
  float filteredPid = (rawPid * 0.4) + (lastPidOutput * 0.6); 
  lastPidOutput = filteredPid;

  // XUẤT XUNG DỰA TRÊN TỐC ĐỘ NỀN KHI CUA (finalBaseSpeed)
  int v_trai = constrain(finalBaseSpeed - (int)filteredPid, speed, startSpeed);
  int v_phai = constrain(finalBaseSpeed + (int)filteredPid, speed, startSpeed); 

  step_dc(true, true, HIGH, LOW, v_phai, v_trai);
}