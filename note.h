#include "control_step.h" // Thư viện điều khiển động cơ bước
#include "sensor.h"       // Thư viện đọc tín hiệu cảm biến line

// --- CẤU HÌNH HỆ THỐNG ---
const int pin_dung = 47;         // Chân điều khiển rơ-le hoặc tín hiệu dừng tại trạm
const int speed = 1200;          // Tốc độ chạy nhanh (Số nhỏ = Xung nhanh = Chạy nhanh)
const int slowSpeed = 12000;     // Tốc độ chạy chậm (Số lớn = Xung chậm = Chạy chậm)
const int startSpeed = 12000;    // Tốc độ khi vừa bắt đầu lăn bánh (Đề ba)

// --- CẤU HÌNH LÀM MƯỢT (RAMP) ---
const float THOI_GIAN_DE_BA = 2.0; // Giây: Thời gian để tăng tốc từ đứng yên lên max
const float THOI_GIAN_PHANH = 0.6; // Giây: Thời gian giảm tốc khi vào cua gắt
const int BU_ZONE_123 = 100;       // Độ bù tốc độ khi lệch nhẹ (Sensor 1,2,3)
const int BU_ZONE_456 = 600;       // Độ bù tốc độ khi lệch vừa (Sensor 4,5,6)
const int BU_ZONE_789 = 750;       // Độ bù tốc độ khi lệch nặng (Sensor 7,8,9)

// --- THÔNG SỐ PID (BỘ ĐIỀU KHIỂN PHẢN HỒI) ---
const float Kp_123 = 50.0;  const float Kd_123 = 400.0; // PID cho đoạn thẳng
const float Kp_456 = 200.0; const float Kd_456 = 700.0; // PID cho cua vừa
const float Kp_789 = 200.0; const float Kd_789 = 500.0; // PID cho cua gắt
const float Ki = 0.01;                                  // Hệ số tích phân (triệt tiêu sai số tĩnh)

// --- CÀI ĐẶT THỜI GIAN VẬN HÀNH ---
const unsigned long thoi_gian_tang_toc = 9000;  // ms: Tổng thời gian tăng tốc toàn hành trình
const unsigned long thoi_gian_giam_toc = 6000;  // ms: Tổng thời gian giảm tốc toàn hành trình
const unsigned long time_dung_lay_hang = 25000; // ms: Thời gian dừng tại trạm (25 giây)
const unsigned long THOI_GIAN_QUAY_MU = 4000;   // ms: Thời gian xoay xe khi không thấy line
const unsigned long time_hu_line = 300;         // ms: Thời gian cho phép mất line trước khi ngắt động cơ

// --- QUẢN LÝ TRẠNG THÁI XE ---
enum TrangThaiTram {
    CHAY_BINH_THUONG = 0,   // Đang bám line bình thường
    BAT_DAU_GIAM_TOC = 1,   // Vừa chạm vạch trạm, đang hãm phanh
    CHO_DUNG = 2,           // Đang bò chậm để chờ dừng chính xác
    DANG_DUNG_LAY_HANG = 3, // Đang đứng yên tại trạm
    ROI_KHOI_TRAM = 4,      // Đang bắt đầu rời trạm
    QUAY_DAU = 5            // Đang thực hiện quay xe 180 độ
};

// --- BIẾN TOÀN CỤC ---
TrangThaiTram trang_thai_14 = CHAY_BINH_THUONG; 
unsigned long thoi_gian_bat_dau = 0, thoi_gian_bat_dau_giam = 0, t_dung = 0; 
unsigned long t_chong_doi_tram = 0;     // Chống nhiễu khi đọc vạch trạm
unsigned long last_ramp_time = 0;       // Lưu mốc thời gian cập nhật tốc độ mượt
unsigned long thoi_gian_bat_dau_quay = 0; 
unsigned long thoi_gian_mat_line = 0; 
bool dang_mat_line = false;             // Cờ báo mất tín hiệu line
bool dang_dung_vi_vat_can = false;      // Cờ báo dừng do cảm biến vật cản
bool dang_dung_vi_mat_line = false;     // Cờ báo dừng do mất line quá lâu
bool dang_tang_toc = true, dang_giam_toc = false; 
bool is_special_code = false;           // Cờ báo gặp mã sensor đặc biệt (12, 13, 14)
int speed_quay_dau = 4000; 

// --- BIẾN TÍNH TOÁN ĐIỀU KHIỂN ---
float currentSpeedVal = startSpeed;     // Tốc độ tức thời (tính theo hàm ramp)
float smoothBaseSpeed = 0;              // Tốc độ nền sau khi lọc nhiễu
float smoothedError = 0;                // Sai số line sau khi lọc Low-pass
float lastError = 0, integral = 0, lastPidOutput = 0; 
float smoothed_offset_cua_f = 0;        // Độ bù tốc độ mượt khi vào cua
int raw_sensor = 0;                     // Giá trị sensor đọc trực tiếp
int steering_error = 0;                 // Sai số hướng lái
int last_valid_sensor = 0;              // Lưu lại vị trí line cuối cùng trước khi mất

void setup() {
    pinMode(pin_dung, OUTPUT); digitalWrite(pin_dung, HIGH); // Mặc định chân dừng ở mức cao
    // Cấu hình các chân Driver Step Motor
    pinMode(PUL_TRAI, OUTPUT); pinMode(DIR_TRAI, OUTPUT); pinMode(ENA_TRAI, OUTPUT);
    pinMode(PUL_PHAI, OUTPUT); pinMode(DIR_PHAI, OUTPUT); pinMode(ENA_PHAI, OUTPUT);
    // Cấu hình chân cảm biến quang (Digital Input)
    pinMode(T4, INPUT); pinMode(T3, INPUT); pinMode(T2, INPUT); pinMode(T1, INPUT);
    pinMode(TG, INPUT); pinMode(PG, INPUT);   
    pinMode(P1, INPUT); pinMode(P2, INPUT); pinMode(P3, INPUT); pinMode(P4, INPUT);
    pinMode(cb1, INPUT_PULLUP); // Cảm biến vật cản
    
    digitalWrite(ENA_TRAI, LOW); digitalWrite(ENA_PHAI, LOW); // Cho phép động cơ hoạt động
    
    KhoiTaoThongSoDeBa(); // Reset các thông số về ban đầu

    if (In_SenSor() == 14) {
        trang_thai_14 = ROI_KHOI_TRAM; // Nếu bật nguồn ngay tại trạm, cho phép rời trạm
    }
}

void loop() {
    raw_sensor = In_SenSor(); // Đọc giá trị cảm biến line liên tục
    
    if (KiemTraAnToanGap()) { // Kiểm tra vật cản & mất line (Ưu tiên cao nhất)
        return; // Nếu không an toàn, dừng toàn bộ các xử lý bên dưới
    }
    
    DocVaLocCamBien();   // Lọc nhiễu cảm biến và xử lý các trường hợp đặc biệt
    XuLyMayTrangThai();  // Quyết định xe đang ở giai đoạn nào (Chạy/Trạm/Quay đầu)
    QuyetDinhXuatXung(); // Tính toán tốc độ cuối cùng và xuất ra động cơ
}

/**
 * Hàm Reset toàn bộ thông số điều khiển
 * Giúp xe bắt đầu chạy lại một cách mượt mà nhất
 */
void KhoiTaoThongSoDeBa() {
    currentSpeedVal = (float)startSpeed; 
    smoothBaseSpeed = (float)startSpeed; 
    toc_do_khi_bat_dau_tang = (float)startSpeed; 
    thoi_gian_bat_dau = millis(); 
    last_ramp_time = millis(); 
    dang_tang_toc = true;
    dang_giam_toc = false;
    integral = 0; 
    lastError = 0; 
    smoothedError = 0; 
    smoothed_offset_cua_f = 0; 
    last_valid_sensor = 0; 
    lastPidOutput = 0;
}

/**
 * Kiểm tra các điều kiện dừng khẩn cấp
 */
bool KiemTraAnToanGap() {
    bool phai_dung_lai = false;
    
    // 1. Kiểm tra cảm biến vật cản (Sensor tiệm cận)
    if (digitalRead(cb1) == LOW) {
        step_dc(false, false, HIGH, LOW, 0, 0); // Ngắt xung động cơ ngay lập tức
        dang_dung_vi_vat_can = true;
        phai_dung_lai = true;
    } else { 
        if (dang_dung_vi_vat_can) { // Nếu vừa hết vật cản
            KhoiTaoThongSoDeBa(); // Khởi động lại từ từ
            dang_dung_vi_vat_can = false;
        }
    }
    
    if (phai_dung_lai) return true; 

    // 2. Kiểm tra lỗi mất line (Mã 13)
    if (raw_sensor == 13) {
        if (!dang_mat_line) {
            dang_mat_line = true;
            thoi_gian_mat_line = millis();
        }
        // Nếu mất line quá thời gian cho phép (300ms) thì mới dừng
        if (millis() - thoi_gian_mat_line >= time_hu_line) {
            step_dc(false, false, HIGH, LOW, 0, 0);
            dang_dung_vi_mat_line = true;
            phai_dung_lai = true;
        } 
    } else { 
        dang_mat_line = false;
        if (dang_dung_vi_mat_line) {
            KhoiTaoThongSoDeBa(); // Thấy lại line thì đề ba lại
            dang_dung_vi_mat_line = false;
        }
    }

    return phai_dung_lai;
}

/**
 * Lọc nhiễu cảm biến và xử lý logic bù line
 */
void DocVaLocCamBien() {
    // Nếu gặp vạch trạm (14) nhưng xe đang cua gắt (nhiễu) -> Bỏ qua vạch trạm, ép đi theo line
    if (raw_sensor == 14 && trang_thai_14 == CHAY_BINH_THUONG) {
        if (abs(last_valid_sensor) >= 6 || abs(smoothedError) > 5.5) {
            raw_sensor = (last_valid_sensor > 0) ? 9 : -9;
        }
    }

    // Kiểm tra xem có phải mã đặc biệt không
    is_special_code = (raw_sensor == 12 || raw_sensor == 13 || raw_sensor == 14);

    if (!is_special_code) {
        steering_error = raw_sensor; // Cập nhật sai số hướng lái bình thường
        last_valid_sensor = raw_sensor; 
    } else {
        steering_error = 0; // Gặp mã đặc biệt thì không tính toán hướng lái theo PID thông thường
    }
}

/**
 * Máy trạng thái - Logic di chuyển chính
 */
void XuLyMayTrangThai() {
    // 1. Xử lý QUAY ĐẦU (Mã 12)
    if (raw_sensor == 12 && trang_thai_14 == CHAY_BINH_THUONG) {
        trang_thai_14 = QUAY_DAU;
        thoi_gian_bat_dau_quay = millis(); 
    }
    
    if (trang_thai_14 == QUAY_DAU) {
        if (millis() - thoi_gian_bat_dau_quay >= THOI_GIAN_QUAY_MU) {
            if (raw_sensor == 0) { // Đã xoay xong và thấy line ở giữa
                trang_thai_14 = CHAY_BINH_THUONG; 
                KhoiTaoThongSoDeBa(); 
            }
        }
        return; 
    }
    
    // 2. Xử lý VÀO TRẠM (Mã 14)
    if (raw_sensor == 14) {
        if (trang_thai_14 == CHAY_BINH_THUONG) { 
            trang_thai_14 = BAT_DAU_GIAM_TOC; // Bước 1: Giảm tốc
            t_chong_doi_tram = millis(); 
            dang_tang_toc = false; dang_giam_toc = true;  
            thoi_gian_bat_dau_giam = millis(); 
            toc_do_khi_bat_dau_giam = smoothBaseSpeed; 
        } else if (trang_thai_14 == CHO_DUNG) { 
            if (millis() - t_chong_doi_tram > 500) {
                trang_thai_14 = DANG_DUNG_LAY_HANG; // Bước 2: Dừng hẳn
                t_dung = millis(); 
                digitalWrite(pin_dung, LOW); // Báo hiệu đã dừng trạm
            }
        }
    }
    
    // Chuyển từ Giảm tốc sang Chờ dừng
    if (trang_thai_14 == BAT_DAU_GIAM_TOC && !is_special_code && (millis() - t_chong_doi_tram > 150)) {
        trang_thai_14 = CHO_DUNG; 
    }
    
    // Sau 25s dừng, cho xe rời trạm
    if (trang_thai_14 == DANG_DUNG_LAY_HANG && (millis() - t_dung >= time_dung_lay_hang)) {
        trang_thai_14 = ROI_KHOI_TRAM; 
        digitalWrite(pin_dung, HIGH); 
        KhoiTaoThongSoDeBa(); 
    }
    
    if (trang_thai_14 == ROI_KHOI_TRAM && !is_special_code) {
        trang_thai_14 = CHAY_BINH_THUONG; 
    }
}

/**
 * Quyết định tốc độ cuối cùng cho 2 động cơ
 */
void QuyetDinhXuatXung() {
    if (trang_thai_14 == DANG_DUNG_LAY_HANG) {
        step_dc(false, false, HIGH, LOW, 0, 0); // Đứng yên
        last_ramp_time = millis(); 
        return; 
    } 

    if (trang_thai_14 == QUAY_DAU) {
        step_dc(true, true, LOW, LOW, speed_quay_dau, speed_quay_dau); // Xoay tại chỗ
        last_ramp_time = millis(); 
        return; 
    } 
    
    TinhToanBiendangTocDo(); // Tính tốc độ nền theo thời gian (Ramp)
    TinhToanVaXuatPID();      // Tính toán hướng lái bám line
}